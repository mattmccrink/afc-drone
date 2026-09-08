// =============================================================================
//  sbus_sim.ino  --  Simulated command sources (alpha)
//
//  Stand-ins for the two absent upstreams so arbitration, arm enforcement, and
//  allocation are fully exercised on a bare board:
//    * primary_sim  -- the Pixhawk/Pi PRIMARY path (command + arm token)
//    * sbus_sim     -- the reversionary SBUS RX-B (command only, no arm)
//
//  When hardware arrives, primary command arrives as real FT_CMD/FT_ARM frames
//  over USB (framing.ino already feeds the same inboxes), and SBUS arrives via
//  the PIO program sketched at the bottom of this file.
// =============================================================================
#include "config.h"

// -----------------------------------------------------------------------------
//  PRIMARY (simulated Pi/Pixhawk)
// -----------------------------------------------------------------------------
//  While "present", emits gently moving sticks every loop (kept fresh) and an
//  arm token at 1 Hz + immediately on change. When not present (button/console
//  toggles g_status.primary_present), it goes silent -> the arm counter stalls
//  and, after ARM_LOSS_TIMEOUT_MS, the node disarms (the powered-reversion
//  window). Note real FT_CMD/FT_ARM frames also write these same inboxes, so a
//  real Pi transparently takes over from the sim.
void primary_sim_update(uint32_t now) {
  if (!g_status.primary_present) return;   // silent -> tokens stall (loss)

  // Gentle synthetic stick motion so allocation output visibly moves.
  float t = now * 0.001f;
  g_primary_in.roll        = 0.30f * sinf(t * 0.7f);
  g_primary_in.pitch       = 0.20f * sinf(t * 0.4f + 1.0f);
  g_primary_in.yaw         = 0.15f * sinf(t * 0.9f + 2.0f);
  g_primary_in.throttle    = 0.5f;
  g_primary_in.mdot_target = g_status.mdot_target;   // console-settable
  g_primary_in.valid       = true;
  g_primary_in.stamp_ms    = now;                     // keeps PRIMARY "fresh"

  // Arm token: 1 Hz heartbeat + on-change. Counter advancement == liveness.
  static uint32_t last_hb = 0;
  static uint8_t  last_intent = 0xFF;
  uint8_t intent = g_sim_arm_intent ? 1 : 0;
  bool changed = (intent != last_intent);
  if (changed || (now - last_hb) >= ARM_HEARTBEAT_MS) {
    last_hb     = now;
    last_intent = intent;
    g_arm_state   = intent;
    g_arm_counter = g_arm_counter + 1;   // monotonic; node checks advancement
    g_arm_stamp   = now;
  }
}

// -----------------------------------------------------------------------------
//  SBUS (simulated reversionary receiver)
// -----------------------------------------------------------------------------
//  Always "present" unless its failsafe/frame-lost flags are set (console
//  'sbusfs'/'sbuslost'). Sticks are whatever the console last set via 'stick'
//  (default centered). Command authority only -- never touches arm.
void sbus_sim_update(uint32_t now) {
  // g_sbus_in.roll/pitch/yaw/throttle are set by the console 'stick' command.
  bool ok = !g_sbus_failsafe && !g_sbus_framelost;
  g_sbus_in.valid    = ok;
  if (ok) g_sbus_in.stamp_ms = now;   // fresh while not in failsafe
}

// -----------------------------------------------------------------------------
//  Real SBUS via PIO  --  STUB (USE_REAL_SBUS == 0 in alpha)
// -----------------------------------------------------------------------------
//  Not shipped as "done" without a receiver to validate against. When RX-B is
//  wired to PIN_SBUS, implement here:
//
//   * PIO SM clocked for 100000 baud, 8E2, INVERTED idle (SBUS is inverted).
//     Sample the start bit, shift 8 data bits LSB-first, verify even parity,
//     require 2 stop bits; push assembled bytes to the RX FIFO.
//   * Core-1-free: the SM runs autonomously; core 0 drains the FIFO and
//     assembles 25-byte SBUS frames (0x0F start, 22 payload, flags, 0x00 end).
//   * Decode 16 x 11-bit channels; map the failsafe (bit 3) and frame-lost
//     (bit 2) flags of byte 23 into g_sbus_failsafe / g_sbus_framelost.
//   * Populate g_sbus_in.roll/pitch/yaw/throttle from the mapped channels
//     (open decision #5: throttle -> direct rpm), stamp on each good frame.
//
//  Everything downstream (arbitration, allocation, direct-rpm mapping) already
//  consumes g_sbus_in + the two flags, so dropping in the PIO decoder is local.
