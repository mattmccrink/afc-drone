#!/usr/bin/env python3
"""
bridge_node.py -- Pi ROS2 supervisor/bridge between MAVROS and the RP2350
valve/sensor node over USB-CDC.

  Pixhawk --MAVROS--> [THIS NODE] --serial(CMD,ARM)--> RP2350
  RP2350  --serial(CTRL_TLM,SENSOR_TLM)--> [THIS NODE] --> ROS2 topics --> rosbag

The node is a supervisor + bridge, NOT an in-loop controller: it forwards
pre-mix control demand, relays the arm token with correct liveness semantics,
and republishes the tiny's telemetry as typed messages. All wire handling lives
in framing.py (byte-compatible with the tiny) and the arm-token safety logic in
arm_token.py; both are unit-tested off-hardware.

Fail-safe intent preserved on the Pi side:
  * CMD is forwarded ONLY while the MAVROS control input is fresh -- if it goes
    stale we stop sending, letting the tiny's USB_CMD_TIMEOUT_MS revert to SBUS.
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

# MAVROS message types are resolved lazily in main() so `colcon build` of this
# package doesn't hard-require mavros_msgs at build time (only at run time).
try:
    from mavros_msgs.msg import State, ActuatorControl
except Exception:  # pragma: no cover - present at runtime on a MAVROS system
    State = ActuatorControl = None

from afc_bridge_msgs.msg import ValveNodeCtrl, ValveNodeSensor

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
        self.cmd_topic = p("cmd_topic", "/mavros/target_actuator_control").value
        self.state_topic = p("state_topic", "/mavros/state").value
        self.cmd_group = int(p("cmd_group_mix", 0).value)     # PX4 group 0 = r/p/y/thrust
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
        self._ctrl = None                # latest [roll,pitch,yaw,thrust]
        self._ctrl_rx_mono = None        # monotonic time of last control msg
        self._armtx = ArmTokenTx(
            send=self._send_arm,
            heartbeat_s=1.0 / self.arm_hb_hz,
            stale_after_s=self.arm_stale_s,
        )
        self._tlm_offset_ms = None       # est. (node_ms - ros_ms), for logging

        # ---- publishers ----
        self._ctrl_pub = self.create_publisher(ValveNodeCtrl, self.ctrl_out, 10)
        self._sensor_pub = self.create_publisher(ValveNodeSensor, self.sensor_out, 10)

        # ---- subscribers (sensor QoS: best-effort, matches MAVROS) ----
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        if State is not None:
            self.create_subscription(State, self.state_topic, self._on_state, sensor_qos)
            self.create_subscription(ActuatorControl, self.cmd_topic,
                                     self._on_actuator, sensor_qos)
        else:
            self.get_logger().error(
                "mavros_msgs not importable; CMD/ARM inputs are disabled. "
                "Install mavros_msgs and restart.")

        # ---- timers ----
        self.create_timer(1.0 / self.cmd_rate_hz, self._cmd_timer)
        self.create_timer(0.05, self._arm_timer)          # 20 Hz drive; TX self-paces
        self.create_timer(0.002, self._rx_timer)          # ~500 Hz serial drain
        self.create_timer(1.0, self._reconnect_timer)     # port keepalive

        self._open_serial()
        self.get_logger().info(
            f"afc_bridge up: port={self.port_name} cmd<={self.cmd_topic} "
            f"(group {self.cmd_group}) cmd_rate={self.cmd_rate_hz}Hz "
            f"arm_hb={self.arm_hb_hz}Hz mdot={self.mdot_target}g/s")

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

    def _on_actuator(self, msg):
        if int(msg.group_mix) != self.cmd_group:
            return
        c = msg.controls
        if len(c) < 4:
            return
        self._ctrl = (float(c[0]), float(c[1]), float(c[2]), float(c[3]))
        self._ctrl_rx_mono = time.monotonic()

    # ----------------------------------------------------------------- timers
    def _cmd_timer(self):
        if self._ctrl is None or self._ctrl_rx_mono is None:
            return
        if (time.monotonic() - self._ctrl_rx_mono) >= self.cmd_stale_s:
            return  # stale primary -> stop forwarding, let the tiny revert
        roll, pitch, yaw, thrust = self._ctrl
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

        # running node<->ROS offset estimate (telemetry sanity / logging only).
        ros_ms = m.header.stamp.sec * 1000 + m.header.stamp.nanosec // 1_000_000
        self._tlm_offset_ms = int(d["node_stamp_ms"]) - ros_ms


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
