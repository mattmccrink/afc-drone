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
  // Snapshot the (volatile) token inbox.
  uint32_t counter = g_arm_counter;
  uint8_t  state   = g_arm_state;

  if (!arb_arm_init) { arb_last_arm_counter = counter; arb_last_advance_ms = now; arb_arm_init = true; }

  // Liveness = counter advanced since we last looked.
  if (counter != arb_last_arm_counter) {
    arb_last_arm_counter = counter;
    arb_last_advance_ms  = now;
    if (state == 0) arb_seen_disarmed = true;   // observed a clean disarmed token
  }
  bool live = (now - arb_last_advance_ms) < ARM_LOSS_TIMEOUT_MS;

  // Armed only if: token live AND reports armed AND we've seen disarmed once.
  bool armed = live && (state == 1) && arb_seen_disarmed;

  g_status.arm_live = live;
  g_status.armed    = armed;
}

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

  if (arb_forced) { g_status.source = arb_forced_src; return; }

  Source cur = g_status.source;
  Source next = cur;

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

  g_status.source = next;
}
