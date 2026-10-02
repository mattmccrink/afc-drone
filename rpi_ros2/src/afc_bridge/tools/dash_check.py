#!/usr/bin/env python3
"""dash_check.py -- offline tests for dash_mux.DashMux (no ROS needed).

  cd rpi_ros2/src/afc_bridge && PYTHONPATH=. python3 tools/dash_check.py
"""
import array
import json
import sys

from afc_bridge.dash_mux import DashMux, to_plain

_fails = 0


def check(name, cond):
    global _fails
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
    if not cond:
        _fails += 1


class FakeMsg:
    """Stands in for a generated ROS message: exposes get_fields_and_field_types()."""
    def __init__(self, **kw):
        self.__dict__.update(kw)
        self._fields = list(kw)

    def get_fields_and_field_types(self):
        return {k: "x" for k in self._fields}


def test_to_plain():
    print("to_plain: ROS message -> JSON-ready dict")
    m = FakeMsg(header=FakeMsg(stamp=1), armed=True, rpm_target=30000, desat_rp=0.123456,
                valve=array.array("h", [1, -2, 3]), surf=[0.5, float("nan")], mode_str="PRIMARY")
    d = to_plain(m)
    check("header dropped", "header" not in d)
    check("bool/int/str kept", d["armed"] is True and d["rpm_target"] == 30000 and d["mode_str"] == "PRIMARY")
    check("float rounded to 4 dp", d["desat_rp"] == 0.1235)
    check("array.array -> list", d["valve"] == [1, -2, 3])
    check("NaN -> null (valid JSON)", d["surf"] == [0.5, None])
    json.dumps(d, allow_nan=False)
    check("whole-number float -> int (shorter)", to_plain(2.0) == 2)


def test_state_only_when_changed():
    print("state sources: latest message, only when it changed")
    mx = DashMux()
    f = mx.frame(0)
    check("empty frame has no state keys", not any(k in f for k in ("ctrl", "sensor", "health", "comp")))
    mx.on_state("ctrl", 10, {"armed": False, "rpm_target": 0, "sbus_gap_ms": 18})
    mx.on_state("ctrl", 50, {"armed": False, "rpm_target": 0, "sbus_gap_ms": 21})
    mx.on_state("sensor", 30, {"mdot_total": 45.3})
    f = mx.frame(100)
    check("ctrl and sensor present", "ctrl" in f and "sensor" in f and "health" not in f)
    check("latest ctrl wins", f["ctrl"]["sbus_gap_ms"] == 21)
    f2 = mx.frame(200)
    check("unchanged sources omitted next frame", "ctrl" not in f2 and "sensor" not in f2)
    check("counters keep counting", f2["src"]["ctrl"]["n"] == 2 and f2["src"]["sensor"]["n"] == 1)
    check("seq increments", f2["seq"] == f["seq"] + 1)


def test_sbus_gap_merged():
    print("RC gap: worst over EVERY ctrl message since the last frame")
    mx = DashMux()
    for t, g in ((0, 18), (40, 95), (80, 20)):           # the 95 ms gap is in a message we don't forward
        mx.on_state("ctrl", t, {"armed": False, "rpm_target": 0, "sbus_gap_ms": g})
    f = mx.frame(100)
    check("frame carries 95, not the latest 20", f["ctrl"]["sbus_gap_ms"] == 95)
    mx.on_state("ctrl", 120, {"armed": False, "rpm_target": 0, "sbus_gap_ms": 19})
    check("reset after each frame", mx.frame(200)["ctrl"]["sbus_gap_ms"] == 19)


def test_source_gaps():
    print("per-source worst gap and age, measured at the Pi")
    mx = DashMux()
    for t in (0, 40, 80, 230, 270):                      # one 150 ms hole
        mx.on_state("sensor", t, {"x": t})
    f = mx.frame(300)
    check("worst gap 150 ms", f["src"]["sensor"]["g"] == 150)
    check("age 30 ms", f["src"]["sensor"]["age"] == 30)
    f = mx.frame(900)
    check("a gap still in progress counts", f["src"]["sensor"]["g"] == 630 and f["src"]["sensor"]["age"] == 630)
    check("never-seen source: age null", f["src"]["torque"]["age"] is None)


def test_px4_decimation():
    print("PX4 streams: decimated to ~25 Hz in SOURCE time, timestamps carried")
    mx = DashMux(px4_sample_hz=25)
    for i in range(100):                                  # 250 Hz for 0.4 s
        t = 1000.0 + i * 4.0
        mx.on_torque(t, t, [0.1, 0.2, 0.3])
        mx.on_thrust(t, t, -0.45)
    f = mx.frame(2000)
    tq = f["ts"]["torque"]
    dts = [b[0] - a[0] for a, b in zip(tq, tq[1:])]
    check(f"~10 torque samples in 0.4 s (got {len(tq)})", 9 <= len(tq) <= 11)
    check(f"kept spacing within one source period of 40 ms {sorted(set(dts))}", all(36 <= d <= 44 for d in dts))
    check("sample row = [t, x, y, z]", tq[0] == [1000, 0.1, 0.2, 0.3])
    check("thrust rows = [t, z]", f["ts"]["thrust"][0] == [1000, -0.45])
    check("samples cleared after the frame", "ts" not in mx.frame(2100))
    mx.on_torque(3000, 50.0, [0, 0, 0])                    # FC rebooted: source clock restarts
    check("source clock going backwards is kept (reboot)", len(mx.frame(3100)["ts"]["torque"]) == 1)


def test_rpm_stream():
    print("rpm samples ride the Tiny clock, with the armed target")
    mx = DashMux()
    mx.on_state("ctrl", 0, {"armed": True, "rpm_target": 30000, "sbus_gap_ms": 0})
    mx.on_state("comp", 5, {"rpm": 29950, "node_stamp_ms": 123456})
    mx.on_state("ctrl", 10, {"armed": False, "rpm_target": 30000, "sbus_gap_ms": 0})
    mx.on_state("comp", 45, {"rpm": 0, "node_stamp_ms": 123496})
    rows = mx.frame(100)["ts"]["rpm"]
    check("rows = [node ms, rpm, target]", rows[0] == [123456, 29950, 30000])
    check("target 0 once disarmed", rows[1] == [123496, 0, 0])


def test_size():
    print("frame size with realistic content")
    mx = DashMux()
    ctrl = {k: False for k in ("armed", "flow_fallback", "terminated", "elig_fc", "elig_sbus", "sbus_sw",
                               "surf_engaged", "surf_switch", "flags2_valid", "sbus_lost", "sbus_ok",
                               "bench_build", "cal_flash", "sim_sensors", "sim_sbus", "sim_primary",
                               "sbus_stats_valid")}
    ctrl.update(source=0, comp_mode=3, rpm_target=30000, n_valid=6, valve=[-900, -250, 12, -20, 420, 860],
                servo_us=[1500] * 12, mode=1, desat_rp=1, desat_yaw=1, surf=[0, 0, 0, 0], air_sev=0,
                air_causes=0, node_stamp_ms=123456789, sbus_n_frames=1234567, sbus_n_lost=12,
                sbus_n_fs=0, sbus_n_bad=0, sbus_gap_ms=18)
    sensor = dict(p_up=[1013.4] * 6, p_lo=[1003.1] * 6, t_die=[25.31] * 6, mdot=[9.1] * 6,
                  mdot_total=54.6, valid=[True] * 6, why=[0] * 6, node_stamp_ms=123456789)
    comp = dict(volt=24.1, amp=18.3, rpm=29950, temp_c=54, err=0, tlm_ok=True, thermal_suspect=False,
                node_stamp_ms=123456789)
    for i in range(4):                                    # 100 ms at 10 Hz: ~4 of each 25 Hz source
        t = i * 25.0
        mx.on_state("ctrl", t, ctrl); mx.on_state("sensor", t, sensor); mx.on_state("comp", t, comp)
        mx.on_torque(t, 5e6 + t, [0.1234, -0.0567, 0.0089]); mx.on_thrust(t, 5e6 + t, -0.4512)
    s = DashMux.encode(mx.frame(100))
    rosbridge = len(json.dumps({"op": "publish", "topic": "/afc/dash", "msg": {"data": s}}))
    print(f"    frame {len(s)} B, as rosbridge sends it {rosbridge} B "
          f"-> {rosbridge * 10 * 8 / 1000:.0f} kbit/s at 10 Hz")
    check("under 4 kB per frame on the wire", rosbridge < 4000)


def main():
    for fn in (test_to_plain, test_state_only_when_changed, test_sbus_gap_merged, test_source_gaps,
               test_px4_decimation, test_rpm_stream, test_size):
        fn()
    print()
    if _fails:
        print(f"{_fails} check(s) FAILED")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
