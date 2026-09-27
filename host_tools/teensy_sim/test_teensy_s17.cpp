// Host simulation of ESCPID.ino's start / telemetry-loss / open-loop sequencing.
// Real ESCPID.ino + real AWPID; ESCCMD, Serial and the ESC are mocks.
// Run: host_tools/teensy_sim/run_test.sh   (builds with ESCPID_OL_THROTTLE=1935 = 30k in the sim plant)
#include "Arduino.h"
#include "ESCCMD.h"
#include <algorithm>
#include <functional>
uint32_t g_ms = 0;
MockUSB  Serial;
MockUart Serial3;
MockEsc  g_esc;
#include "ESCPID.ino"

static int fails = 0;
static void check(const char* n, bool ok) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", n); if (!ok) fails++; }

static void send_host(int16_t rpm10) {
  Hostcomm_struct_t h = {};
  h.magic = ESCPID_COMM_MAGIC; h.RPM_r[0] = rpm10;
  h.PID_P[0] = ESCPID_PID_P; h.PID_I[0] = ESCPID_PID_I; h.PID_D[0] = ESCPID_PID_D; h.PID_f[0] = ESCPID_PID_F;
  const uint8_t* p = (const uint8_t*)&h;
  for (size_t k = 0; k < sizeof h; ++k) Serial3.rx.push_back(p[k]);
}

// Plant: rpm follows throttle (first order, tau 0.3 s); gain = rpm at full throttle.
static float plant_rpm = 0, plant_gain = 31000;
static int   throttle() { return g_esc.cmd == DSHOT_CMD_MOTOR_STOP ? 0 : (int)g_esc.cmd - (DSHOT_CMD_MAX + 1); }

// Packet source: decides per tick what the ESC sends. Default: a good packet every tick.
enum Pkt { NONE, GOOD, CRCBAD, GARBAGE };
static std::function<Pkt(uint32_t)> pkt = [](uint32_t) { return GOOD; };
static int16_t garbage_rpm10 = -20000;
static int16_t tgt10 = 3000;
static float   min_rpm_seen = 1e9;

static void step_ms(uint32_t ms, bool link = true) {
  for (uint32_t t = 0; t < ms; t += 2) {
    if (link && (g_ms % 20) == 0) send_host(tgt10);
    float ss = plant_gain * throttle() / 1999.0f;
    plant_rpm += (ss - plant_rpm) * 0.002f / 0.3f;
    if (g_esc.tlm) {
      switch (pkt(g_ms)) {
        case GOOD:    g_esc.rx++; g_esc.valid = true; g_esc.rpm10 = (int16_t)(plant_rpm / 10.0f); break;
        case CRCBAD:  g_esc.valid = false; g_esc.err = ESCCMD_ERROR_TLM_CRC; break;   // rx NOT counted
        case GARBAGE: g_esc.rx++; g_esc.valid = true; g_esc.rpm10 = garbage_rpm10; break;
        case NONE:    break;
      }
    }
    loop();
    min_rpm_seen = std::min(min_rpm_seen, plant_rpm);
    g_ms += 2;
  }
}
static int8_t last_err() { return ESCPID_comm.err[0]; }
static void   stop_and_reset() { step_ms(300, false); plant_rpm = 0; plant_gain = 31000; tgt10 = 3000;
                                 pkt = [](uint32_t) { return GOOD; }; }

int main() {
  setup();
  const int OL = ESCPID_OL_THROTTLE;

  printf("normal start (telemetry every tick)\n");
  step_ms(8000);
  check("closed loop reaches ~30k", fabsf(plant_rpm - 30000) < 1500 && !ESCPID_OpenLoop[0]);
  check("no NO_TLM code", last_err() != ESCPID_ERROR_NO_TLM);

  printf("restart with NO telemetry -> open loop at the 30k point\n");
  stop_and_reset(); pkt = [](uint32_t) { return NONE; };
  step_ms(400);
  check("before ACQ timeout: PID_MIN, not open loop", !ESCPID_OpenLoop[0] && throttle() == ESCPID_PID_MIN);
  step_ms(200);
  check("after ACQ timeout: OPEN LOOP, -11 reported", ESCPID_OpenLoop[0] && last_err() == ESCPID_ERROR_NO_TLM);
  step_ms(2500);
  check("ramping (mid-ramp)", throttle() > ESCPID_PID_MIN + 200 && throttle() < OL);
  step_ms(3000);
  check("holds the OL throttle", throttle() == OL);

  printf("hand-back needs a run of good packets\n");
  int before = throttle();
  pkt = [](uint32_t) { return GOOD; };
  step_ms(2 * (ESCPID_HANDBACK_N - 2));
  check("N-2 good packets: still open loop", ESCPID_OpenLoop[0]);
  step_ms(6);
  check("N good packets: back to closed loop", !ESCPID_OpenLoop[0]);
  check("no throttle bump at hand-back (<= 5 counts)", abs(throttle() - before) <= 5);
  step_ms(8000);
  check("closed loop settles back at ~30k", fabsf(plant_rpm - 30000) < 1500);

  printf("garbage packet passing CRC (review blocker #1)\n");
  stop_and_reset(); pkt = [](uint32_t) { return NONE; };
  step_ms(6000);                                             // in open loop at OL
  pkt = [](uint32_t ms) { return ms % 1000 == 0 ? GARBAGE : NONE; };  // 1 garbage/s, else nothing
  step_ms(10000);
  check("garbage never ends open loop or drives the PID", ESCPID_OpenLoop[0] && throttle() == OL);
  garbage_rpm10 = 6400;                                      // plausible-looking garbage, isolated
  step_ms(5000);
  check("isolated plausible garbage: still open loop at OL", ESCPID_OpenLoop[0] && throttle() == OL);
  garbage_rpm10 = -20000;

  printf("mid-run loss; wound-up PID can't latch full throttle\n");
  stop_and_reset(); step_ms(8000);
  pkt = [](uint32_t ms) { return ms % 4 == 0 ? GARBAGE : NONE; };   // noisy wire: garbage only
  step_ms(3000);
  check("noisy wire -> open loop near the 30k point, not 1999", ESCPID_OpenLoop[0] && abs(throttle() - OL) <= 30);
  check("-11 reported", last_err() == ESCPID_ERROR_NO_TLM);

  printf("fail toward airflow: heavy load keeps the higher throttle (review v2 (c))\n");
  stop_and_reset(); plant_gain = 28000; step_ms(8000);      // 30k needs > PID_MAX here
  float filt = ESCPID_CtrlFilt[0];
  pkt = [](uint32_t) { return NONE; };
  step_ms(3000);
  check("open loop holds the averaged closed-loop throttle, not the lower OL point",
        ESCPID_OpenLoop[0] && throttle() >= (int)filt - 2 && throttle() > OL);

  printf("one in-range garbage packet right before the wire dies (review v2 (b))\n");
  stop_and_reset(); step_ms(8000);
  float avg = ESCPID_CtrlFilt[0];
  garbage_rpm10 = 0;                                          // plausible but false 0 rpm
  { bool once = true; pkt = [&once](uint32_t) { if (once) { once = false; return GARBAGE; } return NONE; }; }
  step_ms(3000);
  char m2[120]; snprintf(m2, sizeof m2, "open loop floor = average (%.0f), not the spike (now %d)", avg, throttle());
  check(m2, ESCPID_OpenLoop[0] && abs(throttle() - (int)std::max(avg, (float)OL)) <= 30);
  garbage_rpm10 = -20000;

  printf("sparse telemetry (1 packet / 300 ms)\n");
  stop_and_reset(); step_ms(8000);
  pkt = [](uint32_t ms) { return ms % 300 == 0 ? GOOD : NONE; };
  step_ms(3000);
  check("sparse link -> open loop and stays there", ESCPID_OpenLoop[0]);
  check("throttle near the OL point (no ringing)", abs(throttle() - OL) <= 30);

  printf("CRC-bad packets count as missing\n");
  stop_and_reset(); step_ms(8000);
  pkt = [](uint32_t) { return CRCBAD; };
  step_ms(300);
  check("all-CRC-bad -> open loop", ESCPID_OpenLoop[0]);

  printf("open loop follows the host target (review #2)\n");
  stop_and_reset(); tgt10 = 500; pkt = [](uint32_t) { return NONE; };   // 5k bench target
  step_ms(7000);
  int exp5k = ESCPID_PID_MIN + (int)(5000.0f / ESCPID_OL_RPM * (OL - ESCPID_PID_MIN));
  check("5k target: OL throttle scaled down, not the 30k point", ESCPID_OpenLoop[0] && abs(throttle() - exp5k) <= 2);
  stop_and_reset(); tgt10 = 0; pkt = [](uint32_t) { return NONE; };
  step_ms(7000);
  check("target 0: open loop holds PID_MIN", throttle() == ESCPID_PID_MIN);

  printf("hand-back with measured rpm ABOVE target (review #3)\n");
  stop_and_reset(); tgt10 = 1500; plant_gain = 50000; pkt = [](uint32_t) { return NONE; };
  step_ms(7000);                                              // OL scaled for 15k, light load -> ~20k
  float rpm_at_ol = plant_rpm;
  int thr_before = throttle();
  pkt = [](uint32_t) { return GOOD; };
  min_rpm_seen = 1e9;
  step_ms(40);
  check("rpm above target when handed back (test precondition)", rpm_at_ol > 15000 * 1.1f);
  char m[96]; snprintf(m, sizeof m, "no throttle drop at hand-back (%d counts, <= 5)", throttle() - thr_before);
  check(m, abs(throttle() - thr_before) <= 5);
  step_ms(8000);
  check("walks down to target without undershooting >10%", min_rpm_seen > 15000 * 0.9f && fabsf(plant_rpm - 15000) < 1000);

  printf("error priority\n");
  stop_and_reset(); pkt = [](uint32_t) { return NONE; };
  step_ms(1000);
  g_esc.inject_err = ESCCMD_ERROR_DSHOT;
  step_ms(20);
  bool saw_dshot = false;
  for (int k = 0; k < 5 && !saw_dshot; ++k) { g_esc.inject_err = ESCCMD_ERROR_DSHOT; step_ms(20); saw_dshot = (last_err() == ESCCMD_ERROR_DSHOT); }
  check("DShot error (-1) not masked by -11", saw_dshot);

  printf("link lapse / stop\n");
  stop_and_reset(); pkt = [](uint32_t) { return NONE; }; step_ms(1000);
  check("open loop", ESCPID_OpenLoop[0]);
  step_ms(60, false);
  check("60 ms link lapse: still driving (hold-through)", g_esc.cmd != DSHOT_CMD_MOTOR_STOP);
  step_ms(200, false);
  check("long lapse -> MOTOR_STOP, open loop cleared", g_esc.cmd == DSHOT_CMD_MOTOR_STOP && !ESCPID_OpenLoop[0]);
  step_ms(40);
  check("restart begins at PID_MIN", throttle() == ESCPID_PID_MIN);

  printf("emulation marker\n");
  stop_and_reset(); g_esc.emulated = true;
  pkt = [](uint32_t ms) { return ms % 600 == 0 ? CRCBAD : GOOD; };   // occasional emulated loss
  step_ms(3000);
  bool steady = true;
  for (int k = 0; k < 50; ++k) { step_ms(20); if (last_err() != ESCPID_ERROR_EMULATION) steady = false; }
  check("emulation build reports -12 steadily (no flicker on emulated loss)", steady);

  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
