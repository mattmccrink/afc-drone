"""Setpoint freshness gate for CMD forwarding (pure logic, no ROS imports).

CMD carries torque (roll/pitch/yaw) AND thrust, so it is forwarded only while
BOTH setpoints are fresh: the OLDER of the two arrivals must be inside the
window. Keying on the newer arrival (the pre-2026-09-29 behaviour) let a
stalled thrust topic ride on a live torque topic, freezing its last value.
"""
from __future__ import annotations

from typing import Optional


def setpoints_fresh(now: float, t_torque: Optional[float], t_thrust: Optional[float],
                    window_s: float) -> bool:
    """True iff both setpoints have arrived and the older is < window_s old."""
    if t_torque is None or t_thrust is None:
        return False
    return (now - min(t_torque, t_thrust)) < window_s
