"""
dash_mux.py -- merge everything the browser dashboard shows into ONE message per tick.

Why: over the 3 MHz Doodle link, per-packet airtime overhead dominates. Six topics
at their own rates (~110 rosbridge messages/s) queue behind each other in the radio
and arrive bunched; that bunching is the dashboard's jitter. One compact frame at a
fixed rate (default 10 Hz) cuts the packet count ~10x and makes arrivals regular.

  /afc/dash  std_msgs/String, compact JSON, published at rate_hz:
    {"v":1, "seq":N, "t":<pi ms>,
     "src":{"<k>":{"n":updates, "g":worst gap ms, "age":ms}, ...},  per source, at the Pi
     "ctrl":{...}, "sensor":{...}, "health":{...}, "comp":{...},  latest message, only
                                                                    if it changed since the
                                                                    previous frame
     "ts":{"torque":[[t_ms,x,y,z],...], "thrust":[[t_ms,z],...],   samples since the previous
           "rpm":[[t_ms,rpm,target],...]}}                          frame, SOURCE timestamps
  Message dicts keep the ROS field names (minus header), so the dashboard's handlers
  take them unchanged. Torque/thrust times are PX4 time (ms), rpm times the Tiny's
  node_stamp_ms: the dashboard plots by source time behind a short playout delay.

Read-only and off the command path: this node only subscribes to topics that
already exist, so it cannot affect the bridge, and rosbags keep recording the
original topics. The flight-critical bridge does not depend on it.

The merging logic (DashMux) has no ROS dependency, so tools/dash_check.py tests it
without a ROS install.
"""
from __future__ import annotations

import array
import json
import math
import time
from typing import Any, Dict, List, Optional

SOURCES = ("torque", "thrust", "ctrl", "sensor", "health", "comp")
STATE_SOURCES = ("ctrl", "sensor", "health", "comp")       # latest message only
MAX_SAMPLES = 40                                            # per stream per frame (bounds a stall)


def _round(x: float, nd: int = 4) -> Any:
    if math.isnan(x) or math.isinf(x):
        return None                                         # JSON has no NaN
    r = round(x, nd)
    return int(r) if r == int(r) and abs(r) < 1e15 else r


def to_plain(msg: Any, nd: int = 4) -> Any:
    """ROS message (or any nested value) -> JSON-ready plain data, header dropped,
    floats rounded. Works on anything exposing get_fields_and_field_types()."""
    if hasattr(msg, "get_fields_and_field_types"):
        return {k: to_plain(getattr(msg, k), nd)
                for k in msg.get_fields_and_field_types() if k != "header"}
    if isinstance(msg, float):
        return _round(msg, nd)
    if isinstance(msg, bool) or isinstance(msg, (int, str)) or msg is None:
        return msg
    if isinstance(msg, (bytes, bytearray)):
        return list(msg)
    if isinstance(msg, array.array) or hasattr(msg, "tolist"):  # array.array / numpy
        return to_plain(msg.tolist(), nd)
    if isinstance(msg, (list, tuple)):
        return [to_plain(v, nd) for v in msg]
    if isinstance(msg, dict):
        return {k: to_plain(v, nd) for k, v in msg.items() if k != "header"}
    return msg


class DashMux:
    """Pure merging logic. Times are milliseconds on the caller's monotonic clock."""

    def __init__(self, px4_sample_hz: float = 25.0) -> None:
        # Decimate PX4 streams to px4_sample_hz on a grid in SOURCE time: the mean rate is
        # exact and the kept spacing varies by at most one source period (no beat with our timer).
        self._period = 1000.0 / px4_sample_hz if px4_sample_hz > 0 else 0.0
        self._n = {k: 0 for k in SOURCES}
        self._last_rx: Dict[str, Optional[float]] = {k: None for k in SOURCES}
        self._gap = {k: 0.0 for k in SOURCES}               # worst rx gap since last frame
        self._latest: Dict[str, Optional[dict]] = {k: None for k in STATE_SOURCES}
        self._dirty = {k: False for k in STATE_SOURCES}
        self._samples: Dict[str, List[list]] = {"torque": [], "thrust": [], "rpm": []}
        self._due: Dict[str, Optional[float]] = {"torque": None, "thrust": None}
        self._sbus_gap_max = 0                              # merged over every ctrl message
        self._rpm_target = 0
        self.seq = 0

    # ---- inputs ---------------------------------------------------------------
    def _rx(self, k: str, now: float) -> None:
        last = self._last_rx[k]
        if last is not None:
            self._gap[k] = max(self._gap[k], now - last)
        self._last_rx[k] = now
        self._n[k] += 1

    def _keep(self, stream: str, src_ms: float) -> bool:
        P = self._period
        due = self._due[stream]
        if P <= 0 or due is None or src_ms < due - 2 * P:   # first sample, or FC rebooted
            self._due[stream] = src_ms + P
            return True
        if src_ms < due - 0.1 * P:
            return False                                    # too soon: decimate
        due += P
        self._due[stream] = due if due > src_ms else src_ms + P   # resync after a gap
        return True

    def _push(self, stream: str, row: list) -> None:
        s = self._samples[stream]
        s.append(row)
        if len(s) > MAX_SAMPLES:
            del s[0]

    def on_torque(self, now: float, src_ms: float, xyz) -> None:
        self._rx("torque", now)
        if self._keep("torque", src_ms):
            self._push("torque", [int(src_ms)] + [_round(float(v)) for v in xyz])

    def on_thrust(self, now: float, src_ms: float, z: float) -> None:
        self._rx("thrust", now)
        if self._keep("thrust", src_ms):
            self._push("thrust", [int(src_ms), _round(float(z))])

    def on_state(self, k: str, now: float, d: dict) -> None:
        """k in STATE_SOURCES; d = to_plain(msg)."""
        self._rx(k, now)
        if k == "ctrl":
            self._sbus_gap_max = max(self._sbus_gap_max, int(d.get("sbus_gap_ms", 0) or 0))
            self._rpm_target = int(d.get("rpm_target", 0)) if d.get("armed") else 0
        if k == "comp":
            self._push("rpm", [int(d.get("node_stamp_ms", 0)), d.get("rpm", 0), self._rpm_target])
        self._latest[k] = d
        self._dirty[k] = True

    # ---- output ---------------------------------------------------------------
    def frame(self, now: float) -> dict:
        self.seq += 1
        src = {}
        for k in SOURCES:
            last = self._last_rx[k]
            age = None if last is None else int(now - last)
            g = self._gap[k] if last is None else max(self._gap[k], now - last)
            src[k] = {"n": self._n[k], "g": int(g), "age": age}
            self._gap[k] = 0.0
        f: Dict[str, Any] = {"v": 1, "seq": self.seq, "t": int(now), "src": src}
        for k in STATE_SOURCES:
            if self._dirty[k] and self._latest[k] is not None:
                d = self._latest[k]
                if k == "ctrl" and "sbus_gap_ms" in d:
                    d = dict(d, sbus_gap_ms=self._sbus_gap_max)   # worst over every ctrl since last frame
                f[k] = d
                self._dirty[k] = False
        self._sbus_gap_max = 0
        ts = {s: rows for s, rows in self._samples.items() if rows}
        if ts:
            f["ts"] = ts
        self._samples = {"torque": [], "thrust": [], "rpm": []}
        return f

    @staticmethod
    def encode(f: dict) -> str:
        return json.dumps(f, separators=(",", ":"), allow_nan=False)


# ---- ROS 2 node ---------------------------------------------------------------
def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSPresetProfiles
    from std_msgs.msg import String
    from afc_bridge_msgs.msg import ValveNodeCtrl, ValveNodeSensor, ValveNodeHealth, ValveNodeComp

    class DashMuxNode(Node):
        def __init__(self) -> None:
            super().__init__("afc_dash_mux")
            p = self.declare_parameter
            rate = float(p("rate_hz", 10.0).value)
            self.mux = DashMux(px4_sample_hz=float(p("px4_sample_hz", 25.0).value))
            self.pub = self.create_publisher(String, p("dash_topic", "/afc/dash").value, 10)
            q = 10
            self.create_subscription(ValveNodeCtrl, p("ctrl_topic", "/afc/ctrl_tlm").value,
                                     lambda m: self._state("ctrl", m), q)
            self.create_subscription(ValveNodeSensor, p("sensor_topic", "/afc/sensor_tlm").value,
                                     lambda m: self._state("sensor", m), q)
            self.create_subscription(ValveNodeHealth, p("health_topic", "/afc/health").value,
                                     lambda m: self._state("health", m), q)
            self.create_subscription(ValveNodeComp, p("comp_topic", "/afc/compressor").value,
                                     lambda m: self._state("comp", m), q)
            try:
                from px4_msgs.msg import VehicleTorqueSetpoint, VehicleThrustSetpoint
                sq = QoSPresetProfiles.SENSOR_DATA.value            # PX4 /fmu/out is best-effort
                self.create_subscription(
                    VehicleTorqueSetpoint, p("torque_topic", "/fmu/out/vehicle_torque_setpoint").value,
                    lambda m: self.mux.on_torque(self._now(), m.timestamp / 1000.0, m.xyz), sq)
                self.create_subscription(
                    VehicleThrustSetpoint, p("thrust_topic", "/fmu/out/vehicle_thrust_setpoint").value,
                    lambda m: self.mux.on_thrust(self._now(), m.timestamp / 1000.0, m.xyz[2]), sq)
            except ImportError:
                self.get_logger().warn("px4_msgs not importable: torque/thrust not in /afc/dash")
            self.create_timer(1.0 / rate, self._tick)
            self.get_logger().info(f"dash_mux: /afc/dash at {rate:g} Hz")

        @staticmethod
        def _now() -> float:
            return time.monotonic() * 1000.0

        def _state(self, k: str, m) -> None:
            self.mux.on_state(k, self._now(), to_plain(m))

        def _tick(self) -> None:
            self.pub.publish(String(data=DashMux.encode(self.mux.frame(self._now()))))

    rclpy.init(args=args)
    node = DashMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
