"""
Off-hardware wire test for the whole framing path: CMD/ARM out, CTRL/SENSOR/COMP in.
Mirrors both sides of framing.ino (parse_byte/on_frame and tlm_service) so the
Pi modules are proven byte-exact without a tiny attached.

Run from the package dir:  PYTHONPATH=. python3 tools/wire_check.py
"""
import struct
import sys

from afc_bridge import framing as F
from afc_bridge.arm_token import ArmTokenTx
from afc_bridge.cmd_gate import setpoints_fresh
from afc_bridge.qgc_alert import AirAlerter, alert_text, encode_text

# Tiny arm-token semantics (Q5, 2026-09-22; config.h):
ARM_TOKEN_FRESH_MS = 1500     # the token is OBEYED only if it advanced this recently
ARM_LOSS_TIMEOUT_MS = 10000   # token stalled this long -> reversion (SBUS if usable, else
                              #   hold PRIMARY). A stall NEVER disarms by itself.

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

    def token_fresh(self, now_ms):
        return (self._last_advance_ms is not None
                and (now_ms - self._last_advance_ms) <= ARM_TOKEN_FRESH_MS)

    def obeyed(self, now_ms):
        """Arm state the tiny ACTS on: the token's value if fresh, else None
        (tiny holds its current state -- an old token is never obeyed)."""
        return self.arm_state if self.token_fresh(now_ms) else None

    def stalled(self, now_ms):
        """Past ARM_LOSS_TIMEOUT_MS: tiny reverts (SBUS / hold PRIMARY) -- not a disarm."""
        return (self._last_advance_ms is None
                or (now_ms - self._last_advance_ms) > ARM_LOSS_TIMEOUT_MS)


# ---- mirror of the tiny's TX side (tlm_service) -----------------------------
def tiny_pack_ctrl(source, armed, comp_mode, flow_fb, rpm_target, n_valid,
                   valve, servo_us):
    pl = struct.pack("<BBBBHB", source, armed, comp_mode, flow_fb, rpm_target, n_valid)
    pl += struct.pack("<6h", *valve)
    pl += struct.pack("<12H", *servo_us)
    return F.build_frame(F.FT_CTRL_TLM, pl)


def tiny_pack_ctrl_full(source, armed, comp_mode, flow_fb, rpm_target, n_valid,
                        valve, servo_us, mode, flags, drp, dyaw, surf, air_causes=None):
    """Firmware layout: 43 base + 4 fault-tree + 8 surfaces (= 55 B) + 1 air_causes (= 56 B)."""
    pl = struct.pack("<BBBBHB", source, armed, comp_mode, flow_fb, rpm_target, n_valid)
    pl += struct.pack("<6h", *valve)
    pl += struct.pack("<12H", *servo_us)
    pl += struct.pack("<BBBB", mode, flags, drp, dyaw)
    pl += struct.pack("<4h", *surf)
    assert len(pl) == F.CTRL_TLM_LEN_EXT2
    if air_causes is not None:
        pl += struct.pack("<B", air_causes)
        assert len(pl) == F.CTRL_TLM_LEN_EXT3
    return F.build_frame(F.FT_CTRL_TLM, pl)


def tiny_pack_comp(volt_cv, amp_ca, rpm10, temp_c, err, ok, node_ms, flags=None):
    """Mirror of the COMP_TLM block in tlm_service(): 13 B base (+1 B flags)."""
    pl = struct.pack("<HHhBbB", volt_cv, amp_ca, rpm10, temp_c, err, ok)
    pl += struct.pack("<I", node_ms)
    if flags is not None:
        pl += struct.pack("<B", flags)
    return F.build_frame(F.FT_COMP_TLM, pl)


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


def test_ctrl_tlm_ext():
    print("CTRL_TLM in: fault-tree + surfaces extension")
    valve = [100, -100, 50, -50, 25, -25]
    servo = [1500] * 12
    surf = [520, -520, 300, 300]
    flags = 0x01 | 0x02 | 0x10 | 0x20   # terminated, FC eligible, surfaces active, switch on
    frame = tiny_pack_ctrl_full(0, 1, 3, 1, 30000, 6, valve, servo, 4, flags, 77, 100, surf)
    got = list(F.FrameReader().feed(frame))
    check("one 55-byte frame", len(got) == 1 and len(got[0].payload) == 55)
    d = F.decode_ctrl_tlm(got[0].payload)
    check("mode/terminated", d["mode"] == 4 and d["terminated"] is True)
    check("elig_fc / not elig_sbus", d["elig_fc"] is True and d["elig_sbus"] is False)
    check("surf_engaged bit4 / surf_switch bit5", d["surf_engaged"] is True and d["surf_switch"] is True)
    check("desat scales", approx(d["desat_rp"], 0.77) and approx(d["desat_yaw"], 1.0))
    check("surf[] round-trip", d["surf"] == surf)
    # old 47-byte firmware still decodes, with surfaces defaulted
    d47 = F.decode_ctrl_tlm(got[0].payload[:47])
    check("47-byte frame: surf defaults to centered", d47["surf"] == [0, 0, 0, 0])


def test_ctrl_tlm_air():
    print("CTRL_TLM in: air-delivery severity (flags bits 6-7) + causes byte")
    valve = [0] * 6; servo = [1500] * 12; surf = [0, 0, 0, 0]
    flags = 0x02 | (2 << 6)                 # FC eligible + severity CAUTION
    frame = tiny_pack_ctrl_full(0, 1, 3, 1, 30000, 0, valve, servo, 1, flags, 100, 100, surf,
                                air_causes=0x08 | 0x20)
    got = list(F.FrameReader().feed(frame))
    check("one 56-byte frame", len(got) == 1 and len(got[0].payload) == 56)
    d = F.decode_ctrl_tlm(got[0].payload)
    check("air_sev = CAUTION", d["air_sev"] == F.AIR_CAUTION)
    check("air_causes = VENT_LOST|SENSOR_STALE", d["air_causes"] == 0x28)
    check("severity bits don't leak into other flags",
          d["elig_fc"] is True and d["terminated"] is False and d["surf_engaged"] is False
          and d["surf_switch"] is False)
    d55 = F.decode_ctrl_tlm(got[0].payload[:55])
    check("55-byte frame: causes default 0, severity still read", d55["air_causes"] == 0
          and d55["air_sev"] == F.AIR_CAUTION)
    d47 = F.decode_ctrl_tlm(got[0].payload[:43] + bytes([1, 0x02, 100, 100]))
    check("older firmware (bits 6-7 = 0) -> AIR_OK", d47["air_sev"] == F.AIR_OK)


def test_comp_tlm():
    print("COMP_TLM in: byte-exact round-trip (+ thermal flag ext)")
    frame = tiny_pack_comp(2520, 4550, 3012, 91, -8, 1, 7654321, flags=0x01)
    got = list(F.FrameReader().feed(frame))
    check("one 14-byte frame", len(got) == 1 and got[0].type == F.FT_COMP_TLM
          and len(got[0].payload) == 14)
    d = F.decode_comp_tlm(got[0].payload)
    check("volt/amp scaling", approx(d["volt"], 25.20) and approx(d["amp"], 45.50))
    check("rpm = rpm10*10", d["rpm"] == 30120)
    check("temp / signed err", d["temp_c"] == 91 and d["err"] == -8)
    check("tlm_ok / node_ms", d["tlm_ok"] is True and d["node_stamp_ms"] == 7654321)
    check("thermal_suspect set", d["thermal_suspect"] is True)
    got13 = list(F.FrameReader().feed(tiny_pack_comp(2520, 0, 0, 30, 0, 0, 1)))
    d13 = F.decode_comp_tlm(got13[0].payload)
    check("13-byte (old fw) frame: thermal defaults False", d13["thermal_suspect"] is False)


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
    print("ARM token (Q5): heartbeat, on-change, withheld when stale, obeyed only if fresh")
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
    check("fresh ARMED token is obeyed", tiny.obeyed(now_ms[0]) == 1)
    check("heartbeat keeps it inside the 1.5 s freshness window",
          tiny.token_fresh(now_ms[0]))
    check("counter advanced ~1 Hz", 5 <= tx.counter <= 7)

    # on-change disarm
    now_ms[0] = 5200
    tx.note_state(False, 5.2); tx.tick(5.2)
    check("disarm is sent on-change and obeyed immediately", tiny.obeyed(now_ms[0]) == 0)

    # re-arm, then vehicle_status goes stale at the Pi
    for k in range(20):
        t = 6.0 + k * 0.1
        now_ms[0] = int(t * 1000)
        tx.note_state(True, t); tx.tick(t)
    last = tx.counter
    adv_ms = tiny._last_advance_ms           # last token the tiny saw advance (<= 7.9 s)
    now_ms[0] = 9000; tx.tick(9.0)           # 1.1 s since the last vehicle_status (> 0.5 s)
    check("Pi withholds the token once vehicle_status is stale",
          tx.frozen and tx.counter == last and tiny._last_advance_ms == adv_ms)
    check("within 1.5 s: last token still obeyed", tiny.obeyed(adv_ms + ARM_TOKEN_FRESH_MS - 100) == 1)
    check("after 1.5 s: old token NOT obeyed (tiny holds its state; no disarm)",
          tiny.obeyed(adv_ms + ARM_TOKEN_FRESH_MS + 100) is None)
    check("before 10 s: not yet a stall", not tiny.stalled(adv_ms + ARM_LOSS_TIMEOUT_MS - 100))
    check("after 10 s: stall -> reversion (SBUS / hold PRIMARY), still not a disarm",
          tiny.stalled(adv_ms + ARM_LOSS_TIMEOUT_MS + 100)
          and tiny.obeyed(adv_ms + ARM_LOSS_TIMEOUT_MS + 100) is None)
    # link recovers: the next fresh token is obeyed again
    now_ms[0] = adv_ms + 12000
    tx.note_state(True, now_ms[0] / 1000); tx.tick(now_ms[0] / 1000)
    check("fresh token after recovery is obeyed again", tiny.obeyed(now_ms[0]) == 1)


def test_flags2_and_venturi_reason():
    print("CTRL_TLM flags2 (57 B) + SENSOR_TLM per-venturi reason (62 B)")
    valve = [0] * 6; servo = [1500] * 12; surf = [0, 0, 0, 0]
    pl = struct.pack("<BBBBHB", 0, 1, 3, 1, 30000, 5)
    pl += struct.pack("<6h", *valve) + struct.pack("<12H", *servo)
    pl += struct.pack("<BBBB", 1, 0x02 | (1 << 6), 100, 100) + struct.pack("<4h", *surf)
    pl += bytes([0x04])                                   # air_causes: VENT_PARTIAL
    pl += bytes([0x01 | 0x04 | 0x10])                     # flags2: sbus_lost, bench, sim sensors
    got = list(F.FrameReader().feed(F.build_frame(F.FT_CTRL_TLM, pl)))
    check("one 57-byte CTRL frame", len(got) == 1 and len(got[0].payload) == 57)
    d = F.decode_ctrl_tlm(got[0].payload)
    check("flags2 decoded", d["flags2_valid"] and d["sbus_lost"] and d["bench_build"]
          and d["sim_sensors"] and not d["sbus_ok"] and not d["cal_flash"]
          and not d["sim_sbus"] and not d["sim_primary"])
    check("air fields unaffected", d["air_sev"] == 1 and d["air_causes"] == 0x04)
    d56 = F.decode_ctrl_tlm(got[0].payload[:56])
    check("56-byte (older) frame: flags2_valid False, nothing flagged",
          not d56["flags2_valid"] and not d56["bench_build"] and not d56["sbus_lost"])

    why = [0, 0, 4, 0, 10, 9]                            # OK OK STALE OK INJECTED RECOVER
    valid = [w == 0 for w in why]
    frame = tiny_pack_sensor([1000.0] * 6, [990.0] * 6, [25.0] * 6, [7.0] * 6, 28.0, valid, 1234)
    pl = list(F.FrameReader().feed(frame))[0].payload + bytes(why)
    got = list(F.FrameReader().feed(F.build_frame(F.FT_SENSOR_TLM, pl)))
    check("one 62-byte SENSOR frame", len(got) == 1 and len(got[0].payload) == 62)
    ds = F.decode_sensor_tlm(got[0].payload)
    check("why[] round-trip", ds["why"] == why and ds["valid"] == valid)
    check("names", [F.VH_NAMES[w] for w in ds["why"]][2] == "STALE")
    ds56 = F.decode_sensor_tlm(got[0].payload[:56])
    check("56-byte (older) frame: why = 0 for valid, 255 (unknown) for invalid",
          ds56["why"] == [0 if v else 255 for v in valid])


def test_ctrl_node_stamp():
    print("CTRL_TLM node clock (61 B)")
    valve = [0] * 6; servo = [1500] * 12; surf = [0, 0, 0, 0]
    pl = struct.pack("<BBBBHB", 0, 1, 3, 1, 30000, 5)
    pl += struct.pack("<6h", *valve) + struct.pack("<12H", *servo)
    pl += struct.pack("<BBBB", 1, 0x02, 100, 100) + struct.pack("<4h", *surf)
    pl += bytes([0x00, 0x02])                             # air_causes, flags2 (sbus_ok)
    stamp = 0xFEDCBA98                                    # > 2^31: catches a signed unpack
    pl += struct.pack("<I", stamp)
    got = list(F.FrameReader().feed(F.build_frame(F.FT_CTRL_TLM, pl)))
    check("one 61-byte CTRL frame", len(got) == 1 and len(got[0].payload) == F.CTRL_TLM_LEN_EXT5)
    d = F.decode_ctrl_tlm(got[0].payload)
    check("node_stamp_ms round-trip (unsigned)", d["node_stamp_ms"] == stamp)
    check("earlier fields unaffected", d["flags2_valid"] and d["sbus_ok"] and d["mode"] == 1)
    d57 = F.decode_ctrl_tlm(got[0].payload[:57])
    check("57-byte (older) frame: node_stamp_ms = 0, flags2 still decoded",
          d57["node_stamp_ms"] == 0 and d57["flags2_valid"])


def test_setpoint_gate():
    print("CMD gate: both torque and thrust must be fresh")
    w = 0.1
    check("nothing received -> not fresh", not setpoints_fresh(1.0, None, None, w))
    check("torque only -> not fresh", not setpoints_fresh(1.0, 0.99, None, w))
    check("both recent -> fresh", setpoints_fresh(1.0, 0.95, 0.97, w))
    check("thrust stalled, torque live -> NOT fresh (old behaviour forwarded)",
          not setpoints_fresh(1.0, 0.999, 0.85, w))
    check("torque stalled, thrust live -> NOT fresh", not setpoints_fresh(1.0, 0.85, 0.999, w))
    check("older exactly at the window -> not fresh",       # binary-exact values
          not setpoints_fresh(2.0, 1.875, 1.95, 0.125))


def test_qgc_alerter():
    print("QGC alert policy (air severity -> mavlink_log -> STATUSTEXT)")
    a = AirAlerter(min_gap_s=2.0)
    check("OK / ADVISORY send nothing", a.update(0, 0, 0.0) == [] and a.update(1, 0x01, 0.5) == [])
    out = a.update(2, 0x08, 1.0)
    check("CAUTION -> one MAV_SEVERITY_WARNING (4)", len(out) == 1 and out[0][0] == 4
          and "CAUTION" in out[0][1] and "VENTURIS LOST" in out[0][1])
    check("same state -> no repeat", a.update(2, 0x08, 1.5) == [])
    check("rate limit: escalation within 2 s of the last send waits", a.update(3, 0x09, 2.5) == [])
    out = a.update(3, 0x09, 3.1)
    check("then WARNING -> MAV_SEVERITY_CRITICAL (2) with both causes", len(out) == 1
          and out[0][0] == 2 and "ESC TLM LOST" in out[0][1] and "VENTURIS LOST" in out[0][1])
    out = a.update(3, 0x19, 9.5)
    check("cause change while alerting -> re-sent", len(out) == 1 and "NO FLOW" in out[0][1])
    out = a.update(1, 0x01, 12.0)
    check("drop below CAUTION -> one INFO (6) recovered", len(out) == 1 and out[0][0] == 6)
    check("...and only once", a.update(0, 0, 20.0) == [])
    t = alert_text(3, 0x7F)
    check("text fits char[127]", len(t) <= 126 and len(encode_text(t)) == 127
          and encode_text(t)[len(t)] == 0)


def main():
    for fn in [test_cmd_and_resync, test_ctrl_tlm, test_ctrl_tlm_ext, test_ctrl_tlm_air,
               test_flags2_and_venturi_reason, test_ctrl_node_stamp, test_setpoint_gate,
               test_qgc_alerter, test_comp_tlm,
               test_sensor_tlm, test_arm_token]:
        fn()
    print()
    if _fails:
        print(f"{_fails} check(s) FAILED"); sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
