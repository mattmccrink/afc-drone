#!/usr/bin/env python3
"""afc_preflight -- one-command go/no-go check of the Phoenix stack, run on the Pi.

Listens to the live topics for a few seconds, reads a little system state, and
prints one PASS / WARN / FAIL line per check. Exit status: 0 = no FAIL, 1 = at
least one FAIL, 2 = could not run (ROS not sourced, etc.).

    afc_preflight            # flight check: build flags, venturis, RC must be flight-ready
    afc_preflight --bench    # bench: sim/bench builds, missing venturis, no RC -> WARN, not FAIL
    afc_preflight -t 8       # listen 8 s instead of 5

Run it with the vehicle DISARMED, the FC powered and the full stack up. It is
read-only: it subscribes and reads; it never publishes or changes anything.

The checks themselves (evaluate()) are plain Python so they can be tested off
the Pi; only collect() needs ROS.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

PASS, WARN, FAIL, INFO = "PASS", "WARN", "FAIL", "INFO"

BAG_DIR = os.environ.get("AFC_BAG_DIR", "/home/pi/bags")
SERVICES = ["microxrce-agent", "afc-bridge"]

# topic key -> (name, message type path)
TOPICS = {
    "torque": ("/fmu/out/vehicle_torque_setpoint", "px4_msgs.msg.VehicleTorqueSetpoint"),
    "status": ("/fmu/out/vehicle_status_v1", "px4_msgs.msg.VehicleStatus"),
    "gps":    ("/fmu/out/vehicle_gps_position", "px4_msgs.msg.SensorGps"),
    "ctrl":   ("/afc/ctrl_tlm", "afc_bridge_msgs.msg.ValveNodeCtrl"),
    "sensor": ("/afc/sensor_tlm", "afc_bridge_msgs.msg.ValveNodeSensor"),
    "comp":   ("/afc/compressor", "afc_bridge_msgs.msg.ValveNodeComp"),
    "health": ("/afc/health", "afc_bridge_msgs.msg.ValveNodeHealth"),
}


@dataclass
class TopicObs:
    times: List[float] = field(default_factory=list)   # monotonic arrival times
    first: Any = None                                   # first message seen
    last: Any = None                                    # latest message seen

    @property
    def n(self) -> int:
        return len(self.times)

    def rate(self, window: float) -> float:
        return self.n / window if window > 0 else 0.0

    def max_gap(self, t0: float, t1: float) -> float:
        """Longest silence inside [t0, t1], counting the edges of the window."""
        if not self.times:
            return t1 - t0
        pts = [t0] + self.times + [t1]
        return max(b - a for a, b in zip(pts, pts[1:]))


@dataclass
class Result:
    level: str
    area: str
    text: str


def _lvl(bench: bool) -> str:
    """Severity for flight-readiness items: FAIL in flight mode, WARN on the bench."""
    return WARN if bench else FAIL


def evaluate(obs: Dict[str, TopicObs], t0: float, t1: float, sysinfo: Dict[str, Any],
             bench: bool = False) -> List[Result]:
    R: List[Result] = []
    win = t1 - t0
    add = lambda lvl, area, text: R.append(Result(lvl, area, text))

    # ---- services -----------------------------------------------------------
    for svc, state in sysinfo.get("services", {}).items():
        add(PASS if state == "active" else FAIL, "service", f"{svc}: {state}")

    # ---- FC over DDS --------------------------------------------------------
    tq = obs["torque"]
    if tq.n == 0:
        add(FAIL, "FC link", "no torque setpoints from the FC (DDS agent / TELEM2 / FC off?)")
    else:
        rate, gap = tq.rate(win), tq.max_gap(t0, t1)
        lvl = PASS if rate >= 15 and gap < 0.3 else WARN
        add(lvl, "FC link", f"torque setpoints {rate:.1f} Hz, longest gap {gap:.2f} s")
    st = obs["status"]
    if st.n == 0:
        add(FAIL, "FC link", "no vehicle_status (arm token has no source)")
    else:
        armed = getattr(st.last, "arming_state", 0) == 2      # ARMING_STATE_ARMED
        add(WARN if armed else PASS, "FC state", "FC ARMED -- preflight is meant to run disarmed"
            if armed else "FC disarmed")

    # ---- Pi <-> Tiny link (bridge health) -----------------------------------
    h = obs["health"]
    if h.n == 0:
        add(FAIL, "Tiny link", "no /afc/health (bridge not running?)")
    else:
        m = h.last
        if not m.serial_connected:
            add(FAIL, "Tiny link", "Pi <-> Tiny serial DOWN")
        else:
            ok = m.ctrl_hz >= 20 and m.sensor_hz >= 20 and m.comp_hz >= 20
            add(PASS if ok else WARN, "Tiny link",
                f"ctrl {m.ctrl_hz:.1f} Hz, sensor {m.sensor_hz:.1f} Hz, comp {m.comp_hz:.1f} Hz")
        dcrc = int(h.last.crc_errors) - int(h.first.crc_errors)
        if dcrc < 0:
            add(WARN, "Tiny link", "CRC counter went backwards: the bridge restarted during the check")
        else:
            add(PASS if dcrc == 0 else WARN, "Tiny link",
                f"CRC errors +{dcrc} during the check (total {h.last.crc_errors})")
        add(PASS if m.cmd_fresh else FAIL, "command path",
            "bridge forwarding fresh FC commands" if m.cmd_fresh
            else "bridge NOT forwarding commands (setpoints stale at the Pi)")
        add(PASS if m.arm_fresh else FAIL, "command path",
            f"arm token advancing (counter {m.arm_counter})" if m.arm_fresh
            else "arm token NOT advancing (vehicle_status stale)")
        if m.terminated:
            add(FAIL, "mode", "Tiny TERMINATED (latched) -- re-arm from the FC or reboot the Tiny")
        else:
            add(INFO, "mode", f"Tiny mode {m.mode_str or m.mode}")

    # ---- Tiny build + configuration (CTRL_TLM flags2) ----------------------
    c = obs["ctrl"]
    if c.n == 0:
        add(FAIL, "Tiny build", "no /afc/ctrl_tlm")
    else:
        m = c.last
        if not m.flags2_valid:
            add(_lvl(bench), "Tiny build",
                "build flags NOT reported (old firmware?): sim/bench build, calibration and RC unverified")
        else:
            sims = [n for n, on in (("sensors/servos", m.sim_sensors), ("SBUS", m.sim_sbus),
                                    ("primary input", m.sim_primary)) if on]
            add(_lvl(bench) if sims else PASS, "Tiny build",
                "SIMULATED " + ", ".join(sims) if sims else "real sensors, SBUS and primary input")
            add(_lvl(bench) if m.bench_build else PASS, "Tiny build",
                "BENCH_HOOKS build (not a flight build)" if m.bench_build else "flight build (no bench hooks)")
            add(PASS if m.cal_flash else _lvl(bench), "calibration",
                "complete: /cal.bin loaded and servo fits compiled" if m.cal_flash else
                "INCOMPLETE: no /cal.bin (zero/map) or placeholder servo fits (servo_cal.h)")
            # RC reversion must be available before flight
            if m.sbus_ok and not m.sbus_lost:
                add(PASS, "RC", "clean SBUS frames: reversion available")
            else:
                add(_lvl(bench), "RC", "no clean SBUS: reversion UNAVAILABLE")
        stamp0, stamp1 = int(getattr(c.first, "node_stamp_ms", 0)), int(getattr(m, "node_stamp_ms", 0))
        if c.n < 2:
            add(INFO, "Tiny clock", "too few frames to check the node clock")
        elif stamp1 == 0:
            add(INFO, "Tiny clock", "CTRL_TLM carries no node timestamp (pre-61 B firmware)")
        elif stamp1 <= stamp0:
            add(WARN, "Tiny clock", f"node_stamp_ms not advancing ({stamp0} -> {stamp1}): Tiny rebooted?")
        else:
            add(PASS, "Tiny clock", f"up {stamp1 / 1000:.0f} s")
        add(PASS if m.n_valid == 6 else _lvl(bench), "venturis",
            f"{m.n_valid}/6 valid" + ("" if m.n_valid == 6 else " (all six required to fly)"))
        if m.air_sev:
            add(INFO, "air", f"air severity {m.air_sev} (causes 0x{m.air_causes:02X}) -- expected while disarmed if a venturi is out")

    # ---- compressor controller --------------------------------------------
    cp = obs["comp"]
    if cp.n == 0:
        add(FAIL, "compressor", "no /afc/compressor (Tiny <-> Teensy link?)")
    else:
        m = cp.last
        # Disarmed, the Teensy only parses ESC telemetry while the motor runs, so a
        # stale err / tlm_ok=false at rest is normal; only the build flag is decisive.
        if m.err == -12:
            add(_lvl(bench), "compressor", "Teensy is an ESC-EMULATION build (err -12): telemetry is fake")
        elif m.err in (-10, -11):
            add(WARN, "compressor", f"last ESC telemetry state err {m.err} (lost / open loop) -- check once spun up")
        else:
            add(PASS, "compressor", f"Teensy reporting (err {m.err}, ESC {m.temp_c} C)")
        if m.thermal_suspect:
            add(WARN, "compressor", "ESC thermal-derate flag set")

    # ---- Pi health ------------------------------------------------------------
    fs = sysinfo.get("bag_fstype")
    if fs is None:
        add(_lvl(bench), "Pi disk", f"bag directory {BAG_DIR} missing")
    elif fs in ("overlay", "tmpfs"):
        add(FAIL, "Pi disk", f"bags would go to RAM ({BAG_DIR} is on {fs}): is the bag drive mounted?")
    elif not sysinfo.get("bag_writable", True):
        add(FAIL, "Pi disk", f"{BAG_DIR} is not writable")
    free = sysinfo.get("bag_free_gb")
    if free is not None and fs is not None:
        lvl = FAIL if free < 1 else WARN if free < 5 else PASS
        add(lvl, "Pi disk", f"{free:.1f} GB free for bags ({BAG_DIR})")
    thr = sysinfo.get("throttled")
    if thr is not None:
        if thr & 0x1:
            add(FAIL, "Pi power", f"UNDER-VOLTAGE now (get_throttled 0x{thr:X})")
        elif thr & 0xC:
            add(WARN, "Pi power", f"CPU throttled / at temperature limit now (0x{thr:X})")
        elif thr & 0x10000:
            add(WARN, "Pi power", f"under-voltage has occurred since boot (0x{thr:X})")
        else:
            add(PASS, "Pi power", "no under-voltage since boot")
    ro = sysinfo.get("overlay")
    if ro is not None:
        add(PASS if ro else INFO, "Pi SD card", "root is read-only (overlay): safe to power-cut"
            if ro else "root is writable: a hard power cut can corrupt the SD card")
    if sysinfo.get("wlan0"):
        add(INFO, "Pi network", "Wi-Fi interface present (pi_setup.sh wifi-off to disable)")
    rec = sysinfo.get("recording")
    add(INFO, "recording", f"recording: {rec}" if rec else "not recording (afc_rec start <tag>)")

    # ---- Pi clock vs GPS -----------------------------------------------------
    g = obs["gps"]
    if g.n and getattr(g.last, "fix_type", 0) >= 3 and getattr(g.last, "time_utc_usec", 0) > 0:
        # time_utc_usec is the GPS solution time; transport lag is tens of ms.
        off = sysinfo.get("wall_at_gps", time.time()) - g.last.time_utc_usec / 1e6
        add(PASS if abs(off) < 5 else WARN, "Pi clock",
            f"Pi clock {off:+.1f} s vs GPS" + ("" if abs(off) < 5 else
                                               "  (fix: pi_setup.sh time-from-gps, with the bridge stopped)"))
    else:
        yr = time.gmtime(sysinfo.get("wall_at_gps", time.time())).tm_year
        add(INFO if yr >= 2025 else WARN, "Pi clock", "no GPS fix to compare; Pi says "
            + time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime(sysinfo.get("wall_at_gps", time.time()))))
    return R


# ---------------------------------------------------------------------------- #
#  collection (ROS + system)
# ---------------------------------------------------------------------------- #
def _sh(cmd: List[str]) -> Optional[str]:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return None


def sysinfo_collect() -> Dict[str, Any]:
    info: Dict[str, Any] = {"services": {s: (_sh(["systemctl", "is-active", s]) or "unknown")
                                         for s in SERVICES}}
    if os.path.isdir(BAG_DIR):
        info["bag_fstype"] = _sh(["findmnt", "-n", "-o", "FSTYPE", "--target", BAG_DIR]) or "?"
        info["bag_writable"] = os.access(BAG_DIR, os.W_OK)
        try:
            info["bag_free_gb"] = shutil.disk_usage(BAG_DIR).free / 1e9
        except OSError:
            pass
    out = _sh(["vcgencmd", "get_throttled"])            # "throttled=0x50000"
    if out and "=" in out:
        try:
            info["throttled"] = int(out.split("=")[1], 16)
        except ValueError:
            pass
    mounts = _sh(["findmnt", "-n", "-o", "FSTYPE", "/"])
    if mounts is not None:
        info["overlay"] = (mounts == "overlay")
    info["wlan0"] = os.path.exists("/sys/class/net/wlan0")
    rec = _sh(["systemctl", "list-units", "--state=active", "--no-legend", "--plain", "afc-record@*"])
    info["recording"] = ", ".join(l.split()[0] for l in rec.splitlines()) if rec else ""
    return info


def collect(window: float):
    import importlib
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSPresetProfiles

    rclpy.init()
    node = Node("afc_preflight")
    qos = QoSPresetProfiles.SENSOR_DATA.value     # best effort: matches PX4 and reads reliable pubs too
    obs = {k: TopicObs() for k in TOPICS}
    gps_wall = [None]

    def cb_for(key):
        def cb(msg):
            o = obs[key]
            o.times.append(time.monotonic())
            if o.first is None:
                o.first = msg
            o.last = msg
            if key == "gps":
                gps_wall[0] = time.time()
        return cb

    for key, (name, tpath) in TOPICS.items():
        mod, cls = tpath.rsplit(".", 1)
        node.create_subscription(getattr(importlib.import_module(mod), cls), name, cb_for(key), qos)

    # DDS discovery takes 1-2 s: wait for the core topics (max 3 s) so the window
    # measures the link, not discovery; then start the window clean.
    tw = time.monotonic()
    while time.monotonic() - tw < 3.0 and not all(obs[k].n for k in ("torque", "ctrl", "health")):
        rclpy.spin_once(node, timeout_sec=0.05)
    for o in obs.values():
        o.times.clear()
        o.first = o.last            # baseline for counters (CRC, node clock)
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        rclpy.spin_once(node, timeout_sec=0.05)
    t1 = time.monotonic()
    node.destroy_node()
    rclpy.shutdown()
    return obs, t0, t1, gps_wall[0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bench", action="store_true", help="bench mode: flight-readiness items WARN instead of FAIL")
    ap.add_argument("-t", "--time", type=float, default=5.0, help="listen window, s (default 5)")
    a = ap.parse_args()
    try:
        obs, t0, t1, gps_wall = collect(a.time)
    except ImportError as e:
        print(f"cannot run: {e} -- source ROS 2 and the afc workspace first (the afc_preflight wrapper does)")
        return 2
    except Exception as e:                      # rclpy init etc.: not a vehicle FAIL
        print(f"cannot run: {type(e).__name__}: {e}")
        return 2
    info = sysinfo_collect()
    if gps_wall is not None:
        info["wall_at_gps"] = gps_wall
    res = evaluate(obs, t0, t1, info, bench=a.bench)

    color = sys.stdout.isatty()
    col = {PASS: "\033[32m", WARN: "\033[33m", FAIL: "\033[31;1m", INFO: "\033[36m"}
    print(f"Phoenix preflight ({'BENCH' if a.bench else 'FLIGHT'} mode, {a.time:.0f} s listen)")
    for r in res:
        tag = f"{col[r.level]}{r.level}\033[0m" if color else r.level
        print(f"  {tag}  {r.area:<13} {r.text}")
    nf = sum(r.level == FAIL for r in res)
    nw = sum(r.level == WARN for r in res)
    verdict = "NO-GO" if nf else "GO"
    print(f"\n{verdict}: {nf} fail, {nw} warn")
    return 1 if nf else 0


if __name__ == "__main__":
    sys.exit(main())
