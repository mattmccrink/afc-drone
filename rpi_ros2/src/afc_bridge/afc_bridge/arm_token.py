"""
arm_token.py -- the safety-critical arm-token transmitter.

The tiny disarms the compressor unless it keeps seeing a FT_ARM whose monotonic
counter ADVANCES. This class is the sole author of that token. It implements the
one rule that keeps a compressor from outliving its authority:

    advance the counter ONLY while /mavros/state is fresh; the instant it goes
    stale, STOP -- do not freeze-and-resend the last value.

A frozen armed=1 token would read as "link alive, still armed" straight through
an FCU dropout. Stopping instead lets the counter go static -> the tiny sees loss
-> it rides ARM_LOSS_TIMEOUT_MS (10 s powered-reversion window) and disarms.

Deliberate vs. accidental disarm are handled differently:
  * a FRESH disarm (MAVROS says .armed=false) is sent immediately, on-change,
    so a commanded kill propagates in one message-time;
  * a STALE link (MAVROS unknown) stops the token and rides the timeout -- we
    must not assert armed OR force an instant disarm, because we no longer know.

No rclpy in here: feed it MAVROS state via note_state() and drive it from a timer
via tick(). Both take an explicit monotonic 'now' so tests can fast-forward.
"""
from __future__ import annotations

from typing import Callable, Optional

# defaults mirror config.h intent (ARM_HEARTBEAT_MS = 1000)
DEFAULT_HEARTBEAT_S = 1.0
# how long since the last /mavros/state before we treat the link as dead.
# 0.5 s = a couple of missed 1 Hz+ state messages; well inside the 10 s window.
DEFAULT_STALE_AFTER_S = 0.5


class ArmTokenTx:
    def __init__(
        self,
        send: Callable[[bool, int], None],
        heartbeat_s: float = DEFAULT_HEARTBEAT_S,
        stale_after_s: float = DEFAULT_STALE_AFTER_S,
    ) -> None:
        self._send = send
        self._heartbeat_s = heartbeat_s
        self._stale_after_s = stale_after_s

        self._counter = 0
        self._last_send_t: Optional[float] = None
        self._last_sent_state: Optional[bool] = None

        self._mavros_armed: Optional[bool] = None
        self._last_state_rx_t: Optional[float] = None

        # observability
        self.frozen = False           # True while withholding the token (stale link)
        self.sends = 0

    # ---- inputs ----
    def note_state(self, armed: bool, now: float) -> None:
        """Call on every /mavros/state message. 'now' = receipt time (monotonic)."""
        self._mavros_armed = bool(armed)
        self._last_state_rx_t = now

    # ---- freshness ----
    def _fresh(self, now: float) -> bool:
        return (
            self._last_state_rx_t is not None
            and (now - self._last_state_rx_t) < self._stale_after_s
        )

    # ---- driver (call at >= a few Hz; 10-20 Hz is plenty) ----
    def tick(self, now: float) -> None:
        if not self._fresh(now):
            # link dead -> stop advancing. Counter goes static; tiny fails safe.
            self.frozen = True
            return
        self.frozen = False

        armed = self._mavros_armed
        assert armed is not None  # implied by _fresh()

        changed = (armed != self._last_sent_state)
        due = (self._last_send_t is None
               or (now - self._last_send_t) >= self._heartbeat_s)
        if changed or due:
            self._emit(armed, now)

    def _emit(self, armed: bool, now: float) -> None:
        self._counter = (self._counter + 1) & 0xFFFFFFFF
        self._send(armed, self._counter)
        self._last_send_t = now
        self._last_sent_state = armed
        self.sends += 1

    # ---- introspection (telemetry / logging) ----
    @property
    def counter(self) -> int:
        return self._counter
