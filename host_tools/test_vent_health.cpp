// Host test for rp2350_valve_node/vent_health.h (venturi / sensor health logic).
// Build + run from repo root:
//   g++ -std=c++17 -Wall -Wextra -I host_tools/stub -I rp2350_valve_node
//       host_tools/test_vent_health.cpp -o /tmp/tvh && /tmp/tvh
//
// Covers the pure state machines only. The sensors_tick() integration (EMA
// reset, n_valid / mdot_total exclusion, slot->venturi mapping) is exercised on
// the bench: console 'fault valve N on', unplugging one sensor, 'vhealth'.
#include <cstdio>
#include <random>
#include "vent_health.h"

static int fails = 0;
static void check(const char* name, bool ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) fails++;
}

// One sensor on the real core-1 cadence: a 10 ms tick; a conversion is read
// back every 3 ticks; every 16th conversion is D2. sh_eval() runs EVERY tick,
// as in sensors_tick(). 'feed(type, ok, raw)' decides each read's result.
struct Sim {
  SlotHealth h;
  uint32_t now = 1000, tick = 0, conv = 0;
  uint32_t stale_ev_seen = 0;
  uint8_t  last = VH_NODATA;
  template <class F> void run_ms(uint32_t ms, F feed) {
    for (uint32_t t = 0; t < ms; t += 10) {
      if (tick % 3 == 0) {
        int type = (conv % 16 == 0) ? 1 : 0;
        bool ok; uint32_t r;
        feed(type, ok, r);
        sh_on_read(h, type, ok, r, now);
        conv++;
      }
      last = sh_eval(h, true, now);
      tick++; now += 10;
    }
  }
  // count D1 reads only, for boundary tests
  template <class F> void run_d1_reads(int n, F feed) {
    int got = 0;
    while (got < n) {
      if (tick % 3 == 0) {
        int type = (conv % 16 == 0) ? 1 : 0;
        bool ok; uint32_t r;
        feed(type, ok, r);
        sh_on_read(h, type, ok, r, now);
        conv++;
        if (type == 0) got++;
      }
      last = sh_eval(h, true, now);
      tick++; now += 10;
    }
  }
};

static std::mt19937 rng(12345);
static uint32_t noisy(double sigma, uint32_t base = 5000000) {
  std::normal_distribution<double> n(0.0, sigma);
  return (uint32_t)((int64_t)base + llround(n(rng)));
}
static auto GOOD = [](int, bool& ok, uint32_t& r) { ok = true; r = noisy(36); };
static uint32_t absdiff32(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }
static auto DEAD = [](int, bool& ok, uint32_t& r) { ok = false; r = 0; };

int main() {
  printf("slot: healthy / dropout / stale\n");
  {
    Sim s;
    s.run_ms(1000, GOOD);
    check("healthy sensor -> OK", s.last == VH_OK);

    // D1 every 30 ms (60 ms across a D2): 5 consecutive missed D1 = ~150-180 ms
    s.run_d1_reads(5, DEAD);
    check("5 missed D1 reads ride through (evaluated every tick)", s.last == VH_OK);
    s.run_ms(300, GOOD);
    s.run_ms(300, DEAD);
    check("stops answering -> STALE", s.last == VH_STALE);
    check("stale event counted exactly once (per-tick eval)", s.h.n_stale_ev == 1);
    s.run_ms(600, GOOD);
    check("answers again -> OK", s.last == VH_OK);
  }

  printf("slot: bad raw values\n");
  {
    Sim s; s.run_ms(1000, GOOD);
    uint32_t f0 = s.h.n_fail;
    s.run_ms(300, [](int, bool& ok, uint32_t& r) { ok = true; r = 0; });
    check("raw 0 counted as failed read", s.h.n_fail > f0);
    check("raw 0 does not refresh -> STALE", s.last == VH_STALE);
    Sim t; t.run_ms(1000, GOOD);
    t.run_ms(300, [](int, bool& ok, uint32_t& r) { ok = true; r = 0xFFFFFF; });
    check("raw 0xFFFFFF does not refresh -> STALE", t.last == VH_STALE);
    Sim u; u.run_ms(1000, GOOD);
    u.run_ms(300, [](int, bool& ok, uint32_t& r) { ok = false; r = 5000123; });
    check("failed transfer with a non-zero raw is still a failure", u.last == VH_STALE);
  }

  printf("slot: frozen boundary\n");
  {
    Sim s; s.run_ms(1000, GOOD);
    const uint32_t stuck = 5000017;   // near the live level (a far value is a SPIKE first)
    // after this constant read, SENS_FROZEN_N-1 further identical D1s -> same_d1 = N-1
    s.run_d1_reads(SENS_FROZEN_N, [&](int, bool& ok, uint32_t& r) { ok = true; r = stuck; });
    check("N-1 repeats -> still OK", s.last == VH_OK && s.h.same_d1 == SENS_FROZEN_N - 1);
    s.run_d1_reads(1, [&](int, bool& ok, uint32_t& r) { ok = true; r = stuck; });
    check("N repeats -> FROZEN", s.last == VH_FROZEN);
    s.run_d1_reads(1, [&](int, bool& ok, uint32_t& r) { ok = true; r = stuck + 1; });
    check("one different count -> OK again (slot level)", s.last == VH_OK);
  }

  printf("slot: spike rejection\n");
  {
    Sim s; s.run_ms(1000, GOOD);
    uint32_t prev = s.h.prev_d1, sp0 = s.h.n_spike;
    bool used = sh_on_read(s.h, 0, true, prev + SENS_D1_JUMP_MAX + 1, s.now);
    check("jump > SENS_D1_JUMP_MAX rejected", !used && s.h.n_spike == sp0 + 1);
    check("rejected spike does not move the reference", s.h.prev_d1 == prev);
    used = sh_on_read(s.h, 0, true, prev + SENS_D1_JUMP_MAX, s.now);
    check("jump == SENS_D1_JUMP_MAX accepted", used);

    // One corrupt read, then normal: dropped, reference kept, candidate cleared.
    Sim g; g.run_ms(1000, GOOD);
    uint32_t ref = g.h.prev_d1;
    bool a = sh_on_read(g.h, 0, true, ref ^ (1u << 20), g.now);        // bit-20 flip
    bool b = sh_on_read(g.h, 0, true, ref + 20, g.now + 30);
    check("single corrupt read dropped, next normal read accepted", !a && b && !g.h.has_cand[0]);

    // A REAL step (~+3x the limit): accepted on the confirming read, never STALE,
    // whatever the D2 phase (a D2 can fall between the two D1 reads).
    bool all_ok = true; int worst_reads = 0;
    for (int phase = 0; phase < 16; ++phase) {
      Sim t; t.run_ms(1000, GOOD);
      t.run_ms(10 * 3 * phase, GOOD);                    // shift the D2 phase
      uint32_t base = t.h.prev_d1 + 3 * SENS_D1_JUMP_MAX;
      uint32_t st0 = t.h.n_stale_ev;
      int reads = 0; bool accepted = false;
      t.run_ms(300, [&](int ty, bool& ok, uint32_t& r) {
        ok = true; r = (ty == 0) ? noisy(36, base) : noisy(36);   // pressure steps, temperature doesn't
        if (ty == 0 && !accepted) { reads++; }
      });
      accepted = absdiff32(t.h.prev_d1, base) < 1000;
      if (!accepted || t.h.n_stale_ev != st0 || t.last != VH_OK) all_ok = false;
      if (t.h.n_spike > (uint32_t)worst_reads) worst_reads = (int)t.h.n_spike;
    }
    check("real step accepted on the confirming read, no STALE, all 16 D2 phases", all_ok);
    char msg[96]; snprintf(msg, sizeof msg, "real step costs exactly one rejected read (worst %d)", worst_reads);
    check(msg, worst_reads == 1);

    // 1 bar/s ramp (~30 mbar per 30 ms read, below the per-read limit): no spikes.
    Sim r; r.run_ms(1000, GOOD);
    uint32_t lvl = r.h.prev_d1;
    r.run_ms(1000, [&](int ty, bool& ok, uint32_t& v) {
      ok = true; if (ty == 0) lvl += 68000; v = (ty == 0) ? lvl : noisy(36); });
    check("1 bar/s ramp tracked with no spikes", r.h.n_spike == 0 && r.last == VH_OK);

    // D2 (temperature) corrupt read is rejected too
    Sim d; d.run_ms(2000, GOOD);
    uint32_t sp = d.h.n_spike;
    bool tu = sh_on_read(d.h, 1, true, d.h.prev_d2 + SENS_D2_JUMP_MAX + 1, d.now);
    check("D2 spike rejected", !tu && d.h.n_spike == sp + 1);
  }

  printf("slot: loss rate (LOSSY)\n");
  {
    // 2 of every 3 reads fail: never 200 ms without a good D1, so not STALE
    Sim s; s.run_ms(1000, GOOD);
    uint32_t i = 0;
    s.run_ms(10000, [&](int, bool& ok, uint32_t& r) { ok = (i++ % 3 == 0); r = noisy(36); });
    check("2-of-3 reads failing -> LOSSY (not hidden as OK)", s.last == VH_LOSSY);
    check("lossy event counted", s.h.n_lossy_ev >= 1);

    Sim q; q.run_ms(1000, GOOD);
    uint32_t j = 0;          // 1 in 5 fails (20 %): below the ~1/3 trip
    q.run_ms(60000, [&](int, bool& ok, uint32_t& r) { ok = (j++ % 5 != 0); r = noisy(36); });
    check("20 % read loss -> stays OK", q.last == VH_OK && q.h.n_lossy_ev == 0);

    s.run_ms(5000, GOOD);
    check("LOSSY clears once reads are clean", s.last == VH_OK);
  }

  printf("slot: temperature (D2) path + boot\n");
  {
    Sim s; s.run_ms(1000, GOOD);
    s.run_ms(2600, [](int t, bool& ok, uint32_t& r) { ok = (t == 0); r = noisy(36); });
    check("D1 fine but no D2 for >2 s -> STALE", s.last == VH_STALE);
    SlotHealth h;
    check("no reads yet -> NODATA", sh_eval(h, true, 5000) == VH_NODATA);
    check("PROM failed -> NOPROM", sh_eval(h, false, 5000) == VH_NOPROM);
  }

  printf("venturi: reasons, precedence, recover gate\n");
  {
    VentInputs in{true, VH_OK, VH_OK, false, true, 0.0f};
    check("all good -> OK", vent_reason(in) == VH_OK);
    in.dp_corr = -VENTURI_DP_NEG_MAX - 0.1f;
    check("throat above upstream -> DPNEG", vent_reason(in) == VH_DPNEG);
    in.dp_corr = -VENTURI_DP_NEG_MAX + 0.1f;
    check("small negative dp (noise) -> OK", vent_reason(in) == VH_OK);
    in.range_ok = false;
    check("out of range -> RANGE", vent_reason(in) == VH_RANGE);
    in.sh_lo = VH_LOSSY;
    check("sensor fault wins over range", vent_reason(in) == VH_LOSSY);
    in.injected = true;
    check("injection shows as INJECTED", vent_reason(in) == VH_INJECTED);
    in.mapped = false;
    check("unmapped -> UNMAPPED", vent_reason(in) == VH_UNMAPPED);
  }
  {
    VentGate g;
    check("never faulted -> OK immediately", vent_gate(g, VH_OK, 100) == VH_OK);
    vent_gate(g, VH_STALE, 1000);
    check("just recovered -> RECOVER", vent_gate(g, VH_OK, 1100) == VH_RECOVER);
    check("still RECOVER at +499 ms", vent_gate(g, VH_OK, 1499) == VH_RECOVER);
    check("OK at +500 ms", vent_gate(g, VH_OK, 1500) == VH_OK);
    check("no spurious RECOVER ~25 days later", vent_gate(g, VH_OK, 1500u + 0x80000000u + 1000u) == VH_OK);

    VentGate f; int valid_ticks = 0;          // fault flapping 300 ms on / 300 ms off
    for (uint32_t t = 0; t < 10000; t += 10) {
      uint8_t w = ((t / 300) % 2) ? VH_OK : VH_FROZEN;
      if (vent_gate(f, w, 2000 + t) == VH_OK) valid_ticks++;
    }
    check("flapping fault (300 ms cycle) never counts as valid", valid_ticks == 0);

    VentGate w;                                // millis() wrap
    vent_gate(w, VH_STALE, 0xFFFFFF00u);
    check("wrap: RECOVER across rollover", vent_gate(w, VH_OK, 0x00000010u) == VH_RECOVER);
    check("wrap: OK after 500 ms", vent_gate(w, VH_OK, 0xFFFFFF00u + 500u) == VH_OK);
  }

  printf("frozen / spike false-trip check (1 h at ~31 Hz, assumed noise)\n");
  for (double sigma : {36.0, 3.0, 1.0}) {
    Sim s;
    s.run_ms(3600000, [&](int, bool& ok, uint32_t& r) { ok = true; r = noisy(sigma); });
    char name[128];
    snprintf(name, sizeof name, "noise sigma %.0f LSB: frozen=%u spikes=%u lossy=%u",
             sigma, (unsigned)s.h.n_frozen_ev, (unsigned)s.h.n_spike, (unsigned)s.h.n_lossy_ev);
    check(name, s.h.n_frozen_ev == 0 && s.h.n_spike == 0 && s.h.n_lossy_ev == 0);
  }

  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
