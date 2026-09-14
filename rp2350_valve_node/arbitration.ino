// =============================================================================
//  arbitration.ino  --  Source select + arm enforcement  (core 0)
//
//  Source ladder:  PRIMARY (fresh USB) -> SBUS (fresh, flags clear) -> DEFINED-SAFE
//  with hysteresis so a single dropped frame cannot flap sources.
//
//  Arm: the node never *decides* arm -- it enforces a token owned upstream by the
//  Pixhawk. Liveness is judged on monotonic-counter advancement (a stuck/replayed
//  frame must not read as continued arm), with a configurable loss-before-disarm
//  window (the powered-reversion window). Boot disarmed; an ARM transition is only
//  honored after the token has been seen disarmed at least once (no power-up into
//  a hot switch). DISARMED overrides everything downstream.
// -----------------------------------------------------------------------------
#include "config.h"

// ---- source hysteresis timers (module-private) ----
static bool     arb_forced        = false;    // console 'src' override active?
static Source   arb_forced_src    = SRC_SAFE;
static uint32_t arb_primary_fresh_since = 0;
static uint32_t arb_primary_stale_since = 0;

// ---- arm enforcement state ----
static uint32_t arb_last_arm_counter = 0;
static uint32_t arb_last_advance_ms  = 0;
static bool     arb_seen_disarmed    = false;
static bool     arb_arm_init         = false;

// Console hook: force a source, or pass "auto" (SRC_SAFE sentinel => release).
void arbitration_force(Source s, bool force) {
  arb_forced = force;
  arb_forced_src = s;
}

static void arm_update(uint32_t now) {
  // Track FC arm-token liveness + the "seen disarmed once" latch ONLY. This no
  // longer decides g_status.armed: arm authority is resolved per ACTIVE SOURCE at
  // the end of arbitration_update(), so a stale FC token triggers reversion, never
  // a disarm.
  uint32_t counter = g_arm_counter;
  uint8_t  state   = g_arm_state;

  if (!arb_arm_init) { arb_last_arm_counter = counter; arb_last_advance_ms = now; arb_arm_init = true; }

  // Liveness = counter advanced since we last looked.
  if (counter != arb_last_arm_counter) {
    arb_last_arm_counter = counter;
    arb_last_advance_ms  = now;
    if (state == 0) arb_seen_disarmed = true;   // observed a clean disarmed token
  }
}

// Expose the FC-side "seen disarmed once" latch for console/telemetry.
bool arb_fc_seen_disarmed() { return arb_seen_disarmed; }

void arbitration_update(uint32_t now) {
  arm_update(now);

  // Freshness of each candidate source.
  bool primary_fresh = g_primary_in.valid &&
                       (now - g_primary_in.stamp_ms) < USB_CMD_TIMEOUT_MS;
  bool sbus_ok = g_sbus_in.valid && !g_sbus_failsafe && !g_sbus_framelost &&
                 (now - g_sbus_in.stamp_ms) < USB_CMD_TIMEOUT_MS;

  // Track how long PRIMARY has been continuously fresh / stale (for hysteresis).
  if (primary_fresh) {
    if (arb_primary_fresh_since == 0) arb_primary_fresh_since = now;
    arb_primary_stale_since = 0;
  } else {
    if (arb_primary_stale_since == 0) arb_primary_stale_since = now;
    arb_primary_fresh_since = 0;
  }

  // ---- pick the active source: console override, else the hysteretic ladder ---
  Source next;
  if (arb_forced) {
    next = arb_forced_src;
  } else {
    Source cur = g_status.source;
    next = cur;
    switch (cur) {
      case SRC_PRIMARY:
        // Leave PRIMARY only after it has been stale long enough (debounce).
        if (!primary_fresh &&
            arb_primary_stale_since && (now - arb_primary_stale_since) >= PRIMARY_TO_SBUS_HOLD_MS) {
          next = sbus_ok ? SRC_SBUS : SRC_SAFE;
        }
        break;

      case SRC_SBUS:
        // Return to PRIMARY only after it has been fresh long enough.
        if (primary_fresh &&
            arb_primary_fresh_since && (now - arb_primary_fresh_since) >= SBUS_TO_PRIMARY_HOLD_MS) {
          next = SRC_PRIMARY;
        } else if (!sbus_ok) {
          next = SRC_SAFE;
        }
        break;

      case SRC_SAFE:
      default:
        if (primary_fresh)      next = SRC_PRIMARY;   // primary recovers immediately from SAFE
        else if (sbus_ok)       next = SRC_SBUS;
        break;
    }
  }

  g_status.source = next;

  // ---- arm authority FOLLOWS the active source --------------------------------
  // Principle: FC-token staleness is a REVERSION trigger, never a disarm. armed
  // clears ONLY on a positive disarm from whoever currently holds control:
  //   PRIMARY -> a live FC token reporting disarmed
  //   SBUS    -> the pilot's SBUS arm switch going low
  //   SAFE    -> no live command source; the node cannot safely auto-disarm in the
  //              air, so HOLD the last armed state (compressor FALLBACK keeps air).
  //              Ground disarm is a positive act (FC returns & disarms, SBUS switch,
  //              or power-off) -- a stale link alone will not cut airflow.
  bool token_live = (now - arb_last_advance_ms) < ARM_LOSS_TIMEOUT_MS;
  g_status.arm_live = token_live;

  switch (g_status.source) {
    case SRC_PRIMARY:
      // FC token governs. Requires having seen a clean disarm first (no power-up
      // into a hot switch). A STALE token does not reach here as a disarm -- we
      // simply hold the last state until the ladder reverts us.
      if (token_live && arb_seen_disarmed) {
        g_status.armed = (g_arm_state == 1);
      }
      break;

    case SRC_SBUS:
      // Pilot's SBUS arm switch governs, once that switch has been seen disarmed
      // once on a clean frame. If we reverted here with the switch already up and
      // never-seen-disarmed, HOLD the last state so a reversion cannot disarm us
      // mid-flight; the pilot cycling the switch low then arms authority to SBUS.
      if (g_sbus_arm_seen_disarmed) {
        g_status.armed = g_sbus_arm;
      }
      break;

    case SRC_SAFE:
    default:
      // hold last armed
      break;
  }
}
