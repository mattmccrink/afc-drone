#!/usr/bin/env python3
"""
bridge_node.py -- Pi ROS2 supervisor/bridge between MAVROS and the RP2350
valve/sensor node over USB-CDC.

  Pixhawk --MAVROS--> [THIS NODE] --serial(CMD,ARM)--> RP2350
  RP2350  --serial(CTRL_TLM,SENSOR_TLM)--> [THIS NODE] --> ROS2 topics --> rosbag

The node is a supervisor + bridge, NOT an in-loop controller: it forwards
pre-allocation control demand, relays the arm token with correct liveness
semantics, and republishes the tiny's telemetry as typed messages. Allocation
stays on the RP2350 (coanda-slot effectiveness + independent reversion); the FC
only supplies the normalised torque/thrust demand. All wire handling lives in
framing.py (byte-compatible with the tiny) and the arm-token safety logic in
arm_token.py; both are unit-tested off-hardware.

Command source (PX4 1.14+ control allocation, dual-stack):
  * CMD demand comes from uXRCE-DDS: vehicle_torque_setpoint (xyz = roll/pitch/
    yaw) + vehicle_thrust_setpoint (z = thrust), both normalised [-1,1] body FRD,
    ~250 Hz. These are the rate-controller output PRE-allocation -- exactly what
    the tiny's FT_CMD expects, since the tiny does its own allocation.
  * ARM stays on MAVROS (/mavros/state.armed) -- the working, validated path.

Fail-safe intent preserved on the Pi side:
  * CMD is forwarded ONLY while the setpoint stream is fresh -- if it goes stale
    we stop sending, letting the tiny's USB_CMD_TIMEOUT_MS revert to SBUS.
  * the arm token advances ONLY while /mavros/state is fresh (see arm_token.py).
Both degrade toward the tiny's own reversion rather than freezing a last value.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import Header

from afc_bridge import framing as F
from afc_bridge.arm_token import ArmTokenTx

# MAVROS (arm path) and px4_msgs (command path) are resolved lazily so a plain
# `colcon build` of this package doesn't hard-require either at build time --
# they only need to be present/built at run time.
try:
    from mavros_msgs.msg import State
except Exception:  # pragma: no cover - present at runtime on a MAVROS system
    State = None

try:
    from px4_msgs.msg import VehicleTorqueSetpoint, VehicleThrustSetpoint
except Exception:  # pragma: no cover - present at runtime once px4_msgs is built
    VehicleTorqueSetpoint = VehicleThrustSetpoint = None

from afc_bridge_msgs.msg import ValveNodeCtrl, ValveNodeSensor, ValveNodeHealth

try:
    import serial  # pyserial
except Exception:  # pragma: no cover
    serial = None


class BridgeNode(Node):
    def __init__(self):
        super().__init__("afc_bridge")

        # ---- parameters ----
        p = self.declare_parameter
        self.port_name = p("serial_port", "/dev/ttyACM0").value
        self.torque_topic = p("torque_topic", "/fmu/out/vehicle_torque_setpoint").value
        self.thrust_topic = p("thrust_topic", "/fmu/out/vehicle_thrust_setpoint").value
        self.state_topic = p("state_topic", "/mavros/state").value
        # FRD z is down, so a multicopter's climb thrust is NEGATIVE in
        # thrust_setpoint.xyz[2]; the CMD throttle slot wants a positive
        # magnitude. Default -1 flips it; CONFIRM against live data before trust.
        self.thrust_sign = float(p("thrust_sign", -1.0).value)
        self.cmd_rate_hz = float(p("cmd_rate_hz", 50.0).value)
        self.cmd_stale_s = float(p("cmd_stale_after_s", 0.1).value)
        self.mdot_target = float(p("mdot_target_gps", 40.0).value)
        self.thrust_lo = float(p("thrust_lo", 0.0).value)
        self.thrust_hi = float(p("thrust_hi", 1.0).value)
        self.arm_hb_hz = float(p("arm_heartbeat_hz", 1.0).value)
        self.arm_stale_s = float(p("arm_stale_after_s", 0.5).value)
        self.send_tlm_on = bool(p("send_tlm_handshake", True).value)
        self.ctrl_out = p("ctrl_topic", "/afc/ctrl_tlm").value
        self.sensor_out = p("sensor_topic", "/afc/sensor_tlm").value

        # ---- state ----
        self._ser = None
        self._reader = F.FrameReader()
        self._torque = None              # latest (x,y,z) = roll,pitch,yaw demand
        self._thrust = None              # latest (x,y,z); z is the thrust axis
        # Freshness keyed on the NEWER of the two setpoints: any setpoint arriving
        # refreshes the command. Both ride the same DDS client at ~250 Hz, so a
        # partial (one-topic) stall is unlikely; if you want strict both-fresh,
        # track the two rx times separately and test the older one instead.
        self._cmd_rx_mono = None         # monotonic time of the most recent setpoint
        self._armtx = ArmTokenTx(
            send=self._send_arm,
            heartbeat_s=1.0 / self.arm_hb_hz,
            stale_after_s=self.arm_stale_s,
        )
        self._tlm_offset_ms = None       # est. (node_ms - ros_ms), for logging
        self.health_out = p("health_topic", "/afc/health").value

        # ---- health/observability counters ----
        # With the console gone, a silent framing desync looks identical to
        # "no data" -- these make the difference visible. crc_errors lives on
        # the reader; the rest are tallied here and rolled into a 1 Hz rate.
        self._ctrl_count = 0             # CTRL_TLM frames since last health tick
        self._sensor_count = 0           # SENSOR_TLM frames since last health tick
        self._frames_ok = 0             # cumulative good frames
        self._last_source = 255          # from most recent CTRL_TLM (255 = none yet)
        self._last_armed = False
        self._health_t = time.monotonic()

        # ---- publishers ----
        self._ctrl_pub = self.create_publisher(ValveNodeCtrl, self.ctrl_out, 10)
        self._sensor_pub = self.create_publisher(ValveNodeSensor, self.sensor_out, 10)
        self._health_pub = self.create_publisher(ValveNodeHealth, self.health_out, 10)

        # ---- subscribers ----
        # SENSOR_DATA QoS = best-effort/volatile/keep-last, which matches BOTH
        # MAVROS and PX4's uXRCE-DDS publishers. A default (reliable) QoS here is
        # the classic "topic exists but no messages arrive" trap against PX4.
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        if State is not None:
            self.create_subscription(State, self.state_topic, self._on_state, sensor_qos)
        else:
            self.get_logger().error(
                "mavros_msgs not importable; ARM input disabled. Source MAVROS.")
        if VehicleTorqueSetpoint is not None:
            self.create_subscription(VehicleTorqueSetpoint, self.torque_topic,
                                     self._on_torque, sensor_qos)
            self.create_subscription(VehicleThrustSetpoint, self.thrust_topic,
                                     self._on_thrust, sensor_qos)
        else:
            self.get_logger().error(
                "px4_msgs not importable; CMD input disabled. Build px4_msgs.")

        # ---- timers ----
        self.create_timer(1.0 / self.cmd_rate_hz, self._cmd_timer)
        self.create_timer(0.05, self._arm_timer)          # 20 Hz drive; TX self-paces
        self.create_timer(0.002, self._rx_timer)          # ~500 Hz serial drain
        self.create_timer(1.0, self._reconnect_timer)     # port keepalive
        self.create_timer(1.0, self._health_timer)        # 1 Hz health + rosout

        self._open_serial()
        self.get_logger().info(
            f"afc_bridge up: port={self.port_name} "
            f"cmd<=torque({self.torque_topic})+thrust({self.thrust_topic}) "
            f"arm<={self.state_topic} cmd_rate={self.cmd_rate_hz}Hz "
            f"arm_hb={self.arm_hb_hz}Hz mdot={self.mdot_target}g/s "
            f"thrust_sign={self.thrust_sign:+.0f}")

    # ---------------------------------------------------------------- serial
    def _open_serial(self):
        if serial is None:
            self.get_logger().error("pyserial not installed (python3-serial).")
            return
        try:
            self._ser = serial.Serial(self.port_name, baudrate=115200, timeout=0)
            self.get_logger().info(f"opened {self.port_name}")
            if self.send_tlm_on:
                # flip the tiny from text console into binary frame mode BEFORE
                # streaming any binary frame, or the parser never sees them.
                self._ser.write(b"tlm on\n")
        except Exception as e:  # noqa: BLE001
            self._ser = None
            self.get_logger().warn(f"serial open failed ({e}); will retry")

    def _reconnect_timer(self):
        if self._ser is None:
            self._open_serial()

    def _write(self, data: bytes):
        if self._ser is None:
            return
        try:
            self._ser.write(data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"serial write failed ({e}); dropping port")
            self._ser = None

    # ------------------------------------------------------------ subscribers
    def _on_state(self, msg):
        # arm token liveness keys on receipt time, not the FCU header stamp.
        self._armtx.note_state(bool(msg.armed), time.monotonic())

    def _on_torque(self, msg):
        self._torque = (float(msg.xyz[0]), float(msg.xyz[1]), float(msg.xyz[2]))
        self._cmd_rx_mono = time.monotonic()

    def _on_thrust(self, msg):
        self._thrust = (float(msg.xyz[0]), float(msg.xyz[1]), float(msg.xyz[2]))
        self._cmd_rx_mono = time.monotonic()

    def _cmd_fresh(self, now: float) -> bool:
        # need one of each setpoint, and the newer arrival within the window.
        return (self._torque is not None and self._thrust is not None
                and self._cmd_rx_mono is not None
                and (now - self._cmd_rx_mono) < self.cmd_stale_s)

    # ----------------------------------------------------------------- timers
    def _cmd_timer(self):
        if not self._cmd_fresh(time.monotonic()):
            return  # stale/incomplete setpoints -> stop forwarding, tiny reverts
        roll, pitch, yaw = self._torque          # torque xyz -> r/p/y demand
        thrust = self.thrust_sign * self._thrust[2]   # thrust axis, sign-corrected
        self._write(F.build_cmd(roll, pitch, yaw, thrust, self.mdot_target,
                                thrust_lo=self.thrust_lo, thrust_hi=self.thrust_hi))

    def _arm_timer(self):
        self._armtx.tick(time.monotonic())

    def _send_arm(self, armed: bool, counter: int):
        self._write(F.build_arm(armed, counter))

    def _rx_timer(self):
        if self._ser is None:
            return
        try:
            n = self._ser.in_waiting
            chunk = self._ser.read(n or 1)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"serial read failed ({e}); dropping port")
            self._ser = None
            return
        if not chunk:
            return
        for fr in self._reader.feed(chunk):
            if fr.type == F.FT_CTRL_TLM:
                self._publish_ctrl(fr.payload)
            elif fr.type == F.FT_SENSOR_TLM:
                self._publish_sensor(fr.payload)

    # ------------------------------------------------------------- publishing
    def _stamp(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        return h

    def _publish_ctrl(self, payload: bytes):
        try:
            d = F.decode_ctrl_tlm(payload)
        except ValueError as e:
            self.get_logger().warn(str(e))
            return
        m = ValveNodeCtrl()
        m.header = self._stamp()
        m.source = d["source"]
        m.armed = d["armed"]
        m.comp_mode = d["comp_mode"]
        m.flow_fallback = d["flow_fallback"]
        m.rpm_target = d["rpm_target"]
        m.n_valid = d["n_valid"]
        m.valve = d["valve"]
        m.servo_us = d["servo_us"]
        self._ctrl_pub.publish(m)
        self._ctrl_count += 1
        self._frames_ok += 1
        self._last_source = d["source"]     # tiny's readback of our CMD path
        self._last_armed = d["armed"]       # tiny's readback of our arm token

    def _publish_sensor(self, payload: bytes):
        try:
            d = F.decode_sensor_tlm(payload)
        except ValueError as e:
            self.get_logger().warn(str(e))
            return
        m = ValveNodeSensor()
        m.header = self._stamp()
        m.node_stamp_ms = d["node_stamp_ms"]
        m.p_up = [float(x) for x in d["p_up"]]
        m.p_lo = [float(x) for x in d["p_lo"]]
        m.t_die = [float(x) for x in d["t_die"]]
        m.mdot = [float(x) for x in d["mdot"]]
        m.mdot_total = float(d["mdot_total"])
        m.valid = d["valid"]
        self._sensor_pub.publish(m)

        self._sensor_count += 1
        self._frames_ok += 1

        # running node<->ROS offset estimate (telemetry sanity / logging only).
        ros_ms = m.header.stamp.sec * 1000 + m.header.stamp.nanosec // 1_000_000
        self._tlm_offset_ms = int(d["node_stamp_ms"]) - ros_ms


    # ---------------------------------------------------------------- health
    def _health_timer(self):
        now = time.monotonic()
        dt = max(now - self._health_t, 1e-3)
        ctrl_hz = self._ctrl_count / dt
        sensor_hz = self._sensor_count / dt
        self._ctrl_count = 0
        self._sensor_count = 0
        self._health_t = now

        cmd_fresh = self._cmd_fresh(now)
        arm_fresh = not self._armtx.frozen

        m = ValveNodeHealth()
        m.header = self._stamp()
        m.serial_connected = self._ser is not None
        m.ctrl_hz = float(ctrl_hz)
        m.sensor_hz = float(sensor_hz)
        m.crc_errors = int(self._reader.crc_errors)
        m.frames_ok = int(self._frames_ok)
        m.last_source = int(self._last_source) if self._last_source != 255 else 255
        m.last_armed = bool(self._last_armed)
        m.cmd_fresh = bool(cmd_fresh)
        m.arm_fresh = bool(arm_fresh)
        m.arm_counter = int(self._armtx.counter)
        m.tlm_offset_ms = int(self._tlm_offset_ms) if self._tlm_offset_ms is not None else 0
        self._health_pub.publish(m)

        # human-readable 1 Hz line -- the 'health' console you lost, on rosout.
        src = {0: "PRIMARY", 1: "SBUS", 2: "SAFE", 255: "--"}.get(self._last_source, "?")
        self.get_logger().info(
            f"[health] ser={'up' if m.serial_connected else 'DOWN'} "
            f"ctrl={ctrl_hz:4.1f}Hz sens={sensor_hz:4.1f}Hz crc={m.crc_errors} "
            f"src={src} armed={m.last_armed} "
            f"cmd_fresh={m.cmd_fresh} arm_fresh={m.arm_fresh} "
            f"armctr={m.arm_counter} off={m.tlm_offset_ms}ms")


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
