"""
Off-hardware wire test for the whole framing path: CMD/ARM out, CTRL/SENSOR in.
Mirrors both sides of framing.ino (parse_byte/on_frame and tlm_service) so the
Pi modules are proven byte-exact without a tiny attached.

Run from the package dir:  python3 test/test_wire.py
"""
import struct
import sys

from afc_bridge import framing as F
from afc_bridge.arm_token import ArmTokenTx

ARM_LOSS_TIMEOUT_MS = 10000

PASS, FAIL = "\033[32mPASS\033[0m", "\033[31mFAIL\033[0m"
_fails = 0


def check(name, cond):
    global _fails
    if not cond:
        _fails += 1
    print(f"  [{PASS if cond else FAIL}] {name}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---- mirror of the tiny's RX side (parse_byte + on_frame) -------------------
class TinyRx:
    def __init__(self):
        self.reader = F.FrameReader()
        self.roll = self.pitch = self.yaw = self.throttle = self.mdot = None
        self.arm_state = 0
        self.arm_counter = 0
        self._last_counter = None
        self._last_advance_ms = None

    def feed(self, raw, now_ms=0):
        for fr in self.reader.feed(raw):
            if fr.type == F.FT_CMD and len(fr.payload) >= 12:
                r, p, y, t, _u, m = struct.unpack("<6h", fr.payload[:12])
                self.roll, self.pitch, self.yaw = r / 1000, p / 1000, y / 1000
                self.throttle, self.mdot = t / 1000, m / 10
            elif fr.type == F.FT_ARM and len(fr.payload) >= 5:
                self.arm_state, self.arm_counter = struct.unpack("<BI", fr.payload[:5])
                if self.arm_counter != self._last_counter:
                    self._last_counter = self.arm_counter
                    self._last_advance_ms = now_ms

    def armed_live(self, now_ms):
        if self.arm_state != 1 or self._last_advance_ms is None:
            return False
        return (now_ms - self._last_advance_ms) <= ARM_LOSS_TIMEOUT_MS


# ---- mirror of the tiny's TX side (tlm_service) -----------------------------
def tiny_pack_ctrl(source, armed, comp_mode, flow_fb, rpm_target, n_valid,
                   valve, servo_us):
    pl = struct.pack("<BBBBHB", source, armed, comp_mode, flow_fb, rpm_target, n_valid)
    pl += struct.pack("<6h", *valve)
    pl += struct.pack("<12H", *servo_us)
    return F.build_frame(F.FT_CTRL_TLM, pl)


def tiny_pack_sensor(p_up, p_lo, t_die, mdot, mdot_total, valid, node_ms):
    pl = b""
    pl += struct.pack("<6h", *[round(x * 10) for x in p_up])
    pl += struct.pack("<6h", *[round(x * 10) for x in p_lo])
    pl += struct.pack("<6h", *[round(x * 100) for x in t_die])
    pl += struct.pack("<6h", *[round(x * 10) for x in mdot])
    pl += struct.pack("<h", round(mdot_total * 10))
    mask = sum((1 << v) for v in range(6) if valid[v])
    pl += struct.pack("<H", mask)
    pl += struct.pack("<I", node_ms)
    return F.build_frame(F.FT_SENSOR_TLM, pl)


# ---- 1. command/arm out (as before, condensed) -----------------------------
def test_cmd_and_resync():
    print("CMD out: round-trip, clamp, resync, CRC drop")
    tiny = TinyRx()
    tiny.feed(F.build_cmd(0.5, -0.25, 1.0, 0.8, 40.0))
    check("fields decode", approx(tiny.roll, 0.5) and approx(tiny.mdot, 40.0))
    tiny.feed(F.build_cmd(2.0, -9.0, 0.0, 5.0, 999.0))
    check("clamped", approx(tiny.roll, 1.0) and approx(tiny.mdot, 400.0))
    tiny.feed(b"\x00\xff\x33\xAA" + F.build_cmd(0.1, 0.2, 0.3, 0.4, 12.0))
    check("resyncs past garbage", approx(tiny.yaw, 0.3))
    bad = bytearray(F.build_cmd(0.9, 0.9, 0.9, 0.9, 99.0)); bad[7] ^= 0xFF
    e0 = tiny.reader.crc_errors
    tiny.feed(bytes(bad))
    check("bad CRC dropped", approx(tiny.yaw, 0.3) and tiny.reader.crc_errors == e0 + 1)


# ---- 2. telemetry in -------------------------------------------------------
def test_ctrl_tlm():
    print("CTRL_TLM in: decode round-trip")
    valve = [-1000, -500, 0, 250, 750, 1000]
    servo = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 1500]
    frame = tiny_pack_ctrl(F.FT_CTRL_TLM and 1, 1, 2, 1, 34567, 5, valve, servo)
    reader = F.FrameReader()
    got = list(reader.feed(frame))
    check("one frame parsed", len(got) == 1 and got[0].type == F.FT_CTRL_TLM)
    d = F.decode_ctrl_tlm(got[0].payload)
    check("source/armed", d["source"] == 1 and d["armed"] is True)
    check("comp_mode/fallback", d["comp_mode"] == 2 and d["flow_fallback"] is True)
    check("rpm_target", d["rpm_target"] == 34567)
    check("n_valid", d["n_valid"] == 5)
    check("valve[]", d["valve"] == valve)
    check("servo_us[]", d["servo_us"] == servo)


def test_sensor_tlm():
    print("SENSOR_TLM in: decode round-trip + node timestamp")
    p_up = [1013.2, 1012.5, 1015.0, 1011.1, 1014.4, 1013.9]
    p_lo = [1002.1, 1001.0, 1003.3, 1000.5, 1004.0, 1002.8]
    t_die = [25.10, 26.20, 24.90, 27.30, 25.55, 26.00]
    mdot = [3.2, 3.5, 0.0, 2.9, 4.1, 3.8]
    total = sum(mdot)
    valid = [True, True, False, True, True, True]
    node_ms = 1234567
    frame = tiny_pack_sensor(p_up, p_lo, t_die, mdot, total, valid, node_ms)
    reader = F.FrameReader()
    got = list(reader.feed(frame))
    check("one frame parsed", len(got) == 1 and got[0].type == F.FT_SENSOR_TLM)
    d = F.decode_sensor_tlm(got[0].payload)
    check("p_up (0.1 mbar quant)", all(approx(a, b, 0.05) for a, b in zip(d["p_up"], p_up)))
    check("t_die (0.01 C quant)", all(approx(a, b, 0.005) for a, b in zip(d["t_die"], t_die)))
    check("mdot (0.1 g/s quant)", all(approx(a, b, 0.05) for a, b in zip(d["mdot"], mdot)))
    check("mdot_total", approx(d["mdot_total"], total, 0.05))
    check("valid mask", d["valid"] == valid)
    check("node_stamp_ms carried", d["node_stamp_ms"] == node_ms)


# ---- 3. arm-token TX (fresh / on-change / stale window) --------------------
def test_arm_token():
    print("ARM token: heartbeat, on-change disarm, stale->window->disarm")
    tiny = TinyRx()
    now_ms = [0]
    tx = ArmTokenTx(lambda a, c: tiny.feed(F.build_arm(a, c), now_ms[0]),
                    heartbeat_s=1.0, stale_after_s=0.5)

    # 5 s fresh+armed
    t = 0.0
    while t < 5.0:
        now_ms[0] = int(t * 1000)
        if abs((t * 2) - round(t * 2)) < 1e-9:
            tx.note_state(True, t)
        tx.tick(t)
        t += 0.1
    check("armed_live while fresh", tiny.armed_live(now_ms[0]))
    check("counter advanced ~1 Hz", 5 <= tx.counter <= 7)

    # on-change disarm
    now_ms[0] = 5200
    tx.note_state(False, 5.2); tx.tick(5.2)
    check("disarms immediately on-change", not tiny.armed_live(now_ms[0]))

    # re-arm, then stale link -> ride window -> auto disarm
    for k in range(20):
        t = 6.0 + k * 0.1
        now_ms[0] = int(t * 1000)
        tx.note_state(True, t); tx.tick(t)
    last = tx.counter
    adv_ms = tiny._last_advance_ms
    now_ms[0] = int((6.0 + 10.0) * 1000); tx.tick(16.0)   # >0.5s since last state
    check("token withheld once stale", tx.frozen and tx.counter == last)
    now_ms[0] = adv_ms + ARM_LOSS_TIMEOUT_MS - 100; tx.tick(now_ms[0] / 1000)
    check("armed_live just inside window", tiny.armed_live(now_ms[0]))
    now_ms[0] = adv_ms + ARM_LOSS_TIMEOUT_MS + 100; tx.tick(now_ms[0] / 1000)
    check("auto-disarmed past window", not tiny.armed_live(now_ms[0]))


def main():
    for fn in [test_cmd_and_resync, test_ctrl_tlm, test_sensor_tlm, test_arm_token]:
        fn()
    print()
    if _fails:
        print(f"{_fails} check(s) FAILED"); sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
