// Host test for rp2350_valve_node/arbitration.ino (source ladder + arm follow).
// Compiles the REAL arbitration.ino against stand-in globals and drives it with
// a 1 ms loop, like core 0.
// Build + run from repo root:
//   g++ -std=c++17 -Wall -Wextra -I host_tools/stub -I rp2350_valve_node
//       host_tools/test_arbitration.cpp -o /tmp/tarb && /tmp/tarb
//
// Focus (2026-09-28 bench): armed on SBUS, the dashboard flashed SAFE HOLD /
// SBUS LOST for a frame at a time. Two causes, both covered here:
//   * stamps taken after loop()'s `now` (age wrap)        -> age_ms()
//   * receiver per-frame frame-lost bit, no SBUS debounce -> SBUS_LOSS_HOLD_MS
#include <cstdio>
#include "types.h"

// ---- the globals arbitration.ino reads/writes (defined in the main .ino) ----
StickInput g_primary_in = { 0,0,0,0, 0.0f, false, 0 };
StickInput g_sbus_in    = { 0,0,0,0, 0.0f, false, 0 };
volatile uint8_t  g_arm_state = 0;
volatile uint32_t g_arm_counter = 0;
volatile uint32_t g_arm_stamp = 0;
bool g_sbus_failsafe = false, g_sbus_framelost = false;
bool g_sbus_arm = false, g_sbus_arm_seen_disarmed = false, g_sbus_surf_sw = false;
int8_t g_surf_force = -1;
NodeStatus g_status;

#include "arbitration.ino"

static int fails = 0;
static void check(const char* n, bool ok) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", n); if (!ok) fails++; }

// One 1 ms loop pass. The SBUS decoder stamps with its own (possibly later)
// millis() read -- `stamp_ahead` models the tick landing between the two reads.
struct Sim {
  uint32_t now = 1000;
  uint32_t next_frame = 0;
  int frame_period = 9;              // ~9 ms SBUS frames
  bool sbus_alive = true;            // receiver producing frames at all
  bool frame_lost_bit = false;       // receiver flags THIS frame as lost
  bool failsafe = false;
  int  stamp_ahead = 0;
  int  safe_passes = 0, flips = 0;
  Source last = SRC_SAFE;

  void pass() {
    if (sbus_alive && now >= next_frame) {                 // a frame arrives
      next_frame = now + frame_period;
      g_sbus_failsafe  = failsafe;
      g_sbus_framelost = frame_lost_bit;
      if (!failsafe && !frame_lost_bit) {                  // clean: refresh + stamp
        g_sbus_in.valid = true; g_sbus_in.stamp_ms = now + stamp_ahead;
        g_sbus_arm = true;
      }
    }
    arbitration_update(now);
    if (g_status.source == SRC_SAFE) safe_passes++;
    if (g_status.source != last) { flips++; last = g_status.source; }
    now++;
  }
  void run(int ms) { for (int i = 0; i < ms; ++i) pass(); }
};

static Sim armed_on_sbus() {
  // fresh boot, no FC/Pi at all; pilot arms on SBUS (disarmed frame first)
  g_status = NodeStatus{}; g_primary_in.valid = false; g_arm_stamp = 0;
  g_sbus_in = { 0,0,0,0, 0.0f, false, 0 }; g_sbus_failsafe = g_sbus_framelost = false;
  g_sbus_arm = false; g_sbus_arm_seen_disarmed = true;
  arb_sbus_seen_ok = false; arb_sbus_ok_ms = 0; arb_safe_since = 0; s_ever_armed = false;
  Sim s; s.run(50);
  s.safe_passes = 0; s.flips = 0; s.last = g_status.source;
  return s;
}

int main() {
  printf("baseline\n");
  Sim s = armed_on_sbus();
  check("pilot arms on SBUS: source SBUS, armed", g_status.source == SRC_SBUS && g_status.armed);
  s.run(2000);
  check("clean SBUS for 2 s: no flips, never SAFE", s.flips == 0 && s.safe_passes == 0);

  printf("stamp 1 ms ahead of loop now on every frame (age wrap)\n");
  s = armed_on_sbus(); s.stamp_ahead = 1; s.run(2000);
  check("fresh-but-'future' stamps never read as stale: no flips", s.flips == 0 && s.safe_passes == 0);

  printf("receiver frame-lost bit on single frames (missed RF packets)\n");
  s = armed_on_sbus();
  for (int k = 0; k < 20; ++k) {                           // one lost frame every ~100 ms
    s.frame_lost_bit = true;  s.run(s.frame_period);
    s.frame_lost_bit = false; s.run(100 - s.frame_period);
  }
  check("isolated lost frames: stays on SBUS, never SAFE", s.flips == 0 && s.safe_passes == 0);
  check("...and still armed", g_status.armed && !g_status.terminated);

  printf("burst of lost frames shorter than the hold\n");
  s = armed_on_sbus(); s.frame_lost_bit = true; s.run(SBUS_LOSS_HOLD_MS - 20);
  s.frame_lost_bit = false; s.run(200);
  check("~80 ms of frame-lost: rides through on last clean sticks", s.flips == 0 && s.safe_passes == 0);

  printf("real loss: receiver goes silent\n");
  s = armed_on_sbus(); s.sbus_alive = false;
  uint32_t t0 = s.now; uint32_t t_safe = 0;
  for (int i = 0; i < 1000 && !t_safe; ++i) { s.pass(); if (g_status.source == SRC_SAFE) t_safe = s.now - t0; }
  printf("    left SBUS after %u ms\n", (unsigned)t_safe);
  check("still reverts to SAFE (no live source)", t_safe > 0);
  check("within USB_CMD_TIMEOUT_MS + SBUS_LOSS_HOLD_MS + a frame",
        t_safe <= USB_CMD_TIMEOUT_MS + SBUS_LOSS_HOLD_MS + 10);
  check("holds armed through the SAFE debounce (no immediate terminate)", g_status.armed && !g_status.terminated);
  s.run(SAFE_TERMINATE_MS + 10);
  check("sustained total loss still terminates after SAFE_TERMINATE_MS", g_status.terminated && !g_status.armed);

  printf("receiver FAILSAFE bypasses the hold\n");
  s = armed_on_sbus(); s.failsafe = true;
  t0 = s.now; t_safe = 0;
  for (int i = 0; i < 1000 && !t_safe; ++i) { s.pass(); if (g_status.source == SRC_SAFE) t_safe = s.now - t0; }
  printf("    left SBUS after %u ms\n", (unsigned)t_safe);
  check("failsafe frame -> leaves SBUS on the next frame (no 100 ms hold)", t_safe > 0 && t_safe <= (uint32_t)s.frame_period + 1);

  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
