// =============================================================================
//  arbitration.ino  --  Source select + arm enforcement  (core 0)
//
//  Source ladder:  PRIMARY (fresh Pi CMD) -> SBUS (fresh, flags clear) -> SAFE
//  with hysteresis so a single dropped frame cannot flap sources.
//
//  Arm: the node never *decides* arm -- it follows the intent of whichever link
//  is in control (decision 2026-09-22, Q5):
//    PRIMARY -> the FC arm token. If the FC says ARMED, the node is armed,
//               immediately, whatever the reset history (cold boot, watchdog or
//               brownout reset in flight, or recovery from termination). There
//               is no case where the FC is armed and the compressor should not
//               run. Liveness = monotonic-counter advancement; a stale token
//               triggers REVERSION, never a disarm.
//    SBUS    -> the pilot's arm switch, honored only after a clean disarmed
//               frame has been seen (no power-up / revert into a hot switch;
//               after a termination the pilot must cycle the switch).
//    SAFE    -> total loss: hold the last armed state for SAFE_TERMINATE_MS,
//               then terminate (air off). PRIMARY returning armed un-terminates
//               at once ("keep trying to save itself"); SBUS needs a cycle.
//
//  Also resolves the traditional-surface engage switch into g_status.
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
static bool     arb_token_seen       = false; // >=1 FC token received since boot
static bool     arb_arm_init         = false;
static uint32_t arb_safe_since       = 0;     // when SRC_SAFE began (0 = not in SAFE)
static bool     s_ever_armed         = false; // armed at least once since boot -> gates terminate

// Console hook: force a source, or pass "auto" (SRC_SAFE sentinel => release).
void arbitration_force(Source s, bool force) {
  arb_forced = force;
  arb_forced_src = s;
}

static void arm_update(uint32_t now) {
  // Track FC arm-token liveness only. Arm authority is resolved per ACTIVE SOURCE
  // at the end of arbitration_update(), so a stale FC token triggers reversion,
  // never a disarm.
  uint32_t counter = g_arm_counter;

  if (!arb_arm_init) { arb_last_arm_counter = counter; arb_last_advance_ms = now; arb_arm_init = true; }

  // A token has actually arrived since boot (the power-on default g_arm_state=0
  // is not a token and must never be read as FC intent).
  if (g_arm_stamp != 0) arb_token_seen = true;

  // Liveness = counter advanced since we last looked.
  if (counter != arb_last_arm_counter) {
    arb_last_arm_counter = counter;
    arb_last_advance_ms  = now;
  }
}

// FC arm path eligible = a token has been received and advanced recently.
// (Replaces the old "seen disarmed once" latch; exported for console/telemetry.)
static bool s_fc_eligible = false;
bool arb_fc_eligible() { return s_fc_eligible; }

void arbitration_update(uint32_t now) {
  arm_update(now);
  bool token_live = (now - arb_last_advance_ms) < ARM_LOSS_TIMEOUT_MS;

  // Freshness of each candidate source. PRIMARY source freshness = fresh CMD.
  // Separately, once a token has been seen, a token that stops advancing (10 s)
  // hands control to the PILOT when SBUS is usable -- but never forces SAFE /
  // termination while the command link is alive ("follow the intent of the
  // available links"): with no SBUS, PRIMARY holds its state.
  bool primary_fresh = g_primary_in.valid &&
                       (now - g_primary_in.stamp_ms) < USB_CMD_TIMEOUT_MS;
  bool token_ok      = !arb_token_seen || token_live;
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
        } else if (!token_ok && sbus_ok) {
          next = SRC_SBUS;          // FC token dead, pilot available: pilot governs
        }
        break;

      case SRC_SBUS:
        // Return to PRIMARY only after it has been fresh long enough (and its
        // token, if ever seen, is alive).
        if (primary_fresh && token_ok &&
            arb_primary_fresh_since && (now - arb_primary_fresh_since) >= SBUS_TO_PRIMARY_HOLD_MS) {
          next = SRC_PRIMARY;
        } else if (!sbus_ok) {
          next = primary_fresh ? SRC_PRIMARY : SRC_SAFE;   // CMD still alive: hold on PRIMARY
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
  // PRIMARY acts only on a RECENT token (advanced within ARM_TOKEN_FRESH_MS), so
  // a pre-outage g_arm_state -- e.g. ARMED from before a termination or a bridge
  // restart -- can never re-arm the node; a token that stalls for longer is held
  // (no action) until the 10 s liveness window reverts the source.
  bool token_recent = (now - arb_last_advance_ms) <= ARM_TOKEN_FRESH_MS;
  g_status.arm_live = token_live;
  s_fc_eligible = arb_token_seen && token_recent;

  if (g_status.source != SRC_SAFE) arb_safe_since = 0;   // reset the terminate debounce

  switch (g_status.source) {
    case SRC_PRIMARY:
      // FC token governs, immediately, with no seen-disarmed precondition -- as
      // long as it is RECENT (see above). Otherwise hold the last state. An
      // ARMED token also clears a termination.
      if (s_fc_eligible) {
        bool a = (g_arm_state == 1);
        if (a) g_status.terminated = false;   // FC armed -> fly, even after termination
        g_status.armed = a;
      }
      break;

    case SRC_SBUS:
      // Pilot's SBUS arm switch governs, once seen disarmed on a clean frame.
      // A stale FC token is expected here and must not disarm.
      if (g_sbus_arm_seen_disarmed) {
        bool a = g_sbus_arm;
        if (a && !g_status.armed) g_status.terminated = false;  // positive re-arm clears termination
        g_status.armed = a;
      }
      break;

    case SRC_SAFE:
    default:
      // TOTAL command loss -- neither PRIMARY nor SBUS is live. Debounce against
      // a transient double-dropout, then TERMINATE: cut air and come down.
      // Recovery: PRIMARY returning with an ARMED token re-arms at once (above);
      // SBUS requires a fresh disarm->arm cycle (latch reset here).
      if (arb_safe_since == 0) arb_safe_since = now;
      // Only terminate if we were EVER armed: a cold boot waiting in SAFE for the
      // links to come up has not "lost everything" and must not self-terminate.
      if (s_ever_armed && !g_status.terminated && (now - arb_safe_since) >= SAFE_TERMINATE_MS) {
        g_status.terminated      = true;
        g_status.armed           = false;   // arm gates the compressor -> air OFF
        g_sbus_arm_seen_disarmed = false;   // SBUS recovery needs a positive cycle
      }
      // Before the timeout: hold last armed so the compressor keeps air through
      // the debounce -- only a SUSTAINED total loss terminates.
      break;
  }

  if (g_status.armed) s_ever_armed = true;   // latch: once armed, terminate is armed

  // ---- traditional surfaces: pilot switch (or bench force) --------------------
  // g_sbus_surf_sw is refreshed only on clean SBUS frames, so it HOLDS its last
  // state through an SBUS dropout. Allocation ignores it in SAFE / terminated.
  g_status.surf_engaged = (g_surf_force >= 0) ? (g_surf_force == 1) : g_sbus_surf_sw;
  // What the surfaces are actually doing (switch AND a live source AND not
  // terminated) -- this is what telemetry/dashboard report.
  g_status.surf_active  = g_status.surf_engaged && g_status.source != SRC_SAFE && !g_status.terminated;
}
