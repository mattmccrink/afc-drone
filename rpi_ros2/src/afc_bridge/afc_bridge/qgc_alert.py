"""
qgc_alert.py -- decide when an air-delivery alert should reach QGC.

The dashboard is not the pilot's primary display; QGC is. The Pi can't talk
MAVLink to QGC itself (C2 runs FC TELEM1 -> Doodle serial relay -> QGC), so it
publishes px4_msgs/MavlinkLog on /fmu/in/mavlink_log. PX4's mavlink module turns
every mavlink_log into a STATUSTEXT on all links, so the text shows (and is
spoken) in QGC like any vehicle message. Requires /fmu/in/mavlink_log in
PX4's dds_topics.yaml subscriptions (firmware rebuild).

Policy (pure logic, unit-tested in tools/wire_check.py):
  * WARNING (air_sev 3) -> MAV_SEVERITY_CRITICAL (2); CAUTION (2) -> WARNING (4).
  * Send on entering WARNING/CAUTION, on a level change, and when the causes
    change while still alerting; rate-limited to one message per min_gap_s.
  * One INFO (6) "recovered" message when dropping back below CAUTION.
  * ADVISORY is dashboard-only (no QGC traffic).
"""
from __future__ import annotations

MAV_SEV_CRITICAL, MAV_SEV_WARNING, MAV_SEV_INFO = 2, 4, 6
_LEVEL = {0: "OK", 1: "ADVISORY", 2: "CAUTION", 3: "WARNING"}
_CAUSES = {0x01: "ESC TLM LOST", 0x02: "ESC EMULATED", 0x04: "VENTURIS PARTIAL",
           0x08: "VENTURIS LOST", 0x10: "NO FLOW", 0x20: "SENSORS STALE", 0x40: "TEENSY LINK"}
TEXT_MAX = 126                     # MavlinkLog.text is char[127] incl. NUL


def alert_text(level: int, causes: int) -> str:
    why = ", ".join(n for b, n in _CAUSES.items() if causes & b)
    t = f"AFC AIR {_LEVEL.get(level, '?')}" + (f": {why}" if why else "")
    if level >= 3:
        t += " - engage surfaces / land"
    return t[:TEXT_MAX]


class AirAlerter:
    def __init__(self, min_gap_s: float = 2.0):
        self.min_gap_s = min_gap_s
        self._sent_level = 0
        self._sent_causes = 0
        self._last_send = -1e9

    def update(self, level: int, causes: int, now: float):
        """Return a list of (mav_severity, text) to publish now (usually empty)."""
        out = []
        if now - self._last_send < self.min_gap_s:
            return out                                   # re-evaluated next call
        if level >= 2:
            if level != self._sent_level or causes != self._sent_causes:
                sev = MAV_SEV_CRITICAL if level >= 3 else MAV_SEV_WARNING
                out.append((sev, alert_text(level, causes)))
                self._sent_level, self._sent_causes, self._last_send = level, causes, now
        elif self._sent_level >= 2:
            out.append((MAV_SEV_INFO, f"AFC air delivery recovered ({_LEVEL.get(level, '?')})"))
            self._sent_level, self._sent_causes, self._last_send = level, causes, now
        return out


def encode_text(text: str, n: int = 127) -> list:
    """char[127] as px4_msgs exposes it: NUL-padded uint8 list."""
    b = text.encode("ascii", "replace")[: n - 1]
    return list(b) + [0] * (n - len(b))
