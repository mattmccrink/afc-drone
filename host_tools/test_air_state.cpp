// Host test for rp2350_valve_node/air_state.h (air-delivery severity).
// Build + run from repo root:
//   g++ -std=c++17 -Wall -Wextra -I host_tools/stub -I rp2350_valve_node
//       host_tools/test_air_state.cpp -o /tmp/tas && /tmp/tas
#include <cstdio>
#include "air_state.h"

static int fails = 0;
static void check(const char* n, bool ok) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", n); if (!ok) fails++; }

static AirInputs base() {   // armed, past both graces, everything healthy, flow nominal
  AirInputs a{};
  a.armed = true; a.terminated = false; a.tlm_grace_done = true; a.flow_grace_done = true;
  a.comp_link_ok = true; a.comp_err = 0; a.sensor_stale = false; a.n_valid = VALVE_COUNT; a.mdot_total = 40.0f;
  return a;
}
static uint8_t sev(const AirInputs& a, uint8_t* c = nullptr) { uint8_t x; return air_eval(a, c ? c : &x); }

int main() {
  printf("the ranking (decision 2026-09-27)\n");
  AirInputs a = base(); uint8_t c;
  check("all healthy -> OK", sev(a) == AIR_OK);

  a = base(); a.comp_err = ESCPID_ERR_NO_TLM;
  check("telemetry lost, venturis all good, flow ok -> ADVISORY", sev(a, &c) == AIR_ADVISORY && (c & AIR_C_TLM_LOST));

  a = base(); a.n_valid = 4;
  check("some venturis invalid, telemetry ok -> ADVISORY", sev(a, &c) == AIR_ADVISORY && (c & AIR_C_VENT_PARTIAL));

  a = base(); a.n_valid = 0;
  check("venturis lost, telemetry ok -> CAUTION (outranks lost telemetry)", sev(a, &c) == AIR_CAUTION && (c & AIR_C_VENT_LOST));

  a = base(); a.sensor_stale = true;
  check("sensor frame stale counts as venturis lost -> CAUTION", sev(a, &c) == AIR_CAUTION && (c & AIR_C_SENSOR_STALE));

  a = base(); a.comp_err = ESCPID_ERR_NO_TLM; a.n_valid = 3;
  check("telemetry lost AND venturis partial -> CAUTION", sev(a) == AIR_CAUTION);

  a = base(); a.comp_err = ESCPID_ERR_NO_TLM; a.n_valid = 0;
  check("telemetry lost AND venturis lost -> WARNING", sev(a) == AIR_WARNING);

  a = base(); a.comp_err = ESCPID_ERR_NO_TLM; a.mdot_total = 2.0f;
  check("telemetry lost, venturis good but NO FLOW (rotor not delivering) -> WARNING",
        sev(a, &c) == AIR_WARNING && (c & AIR_C_FLOW_LOW));

  a = base(); a.mdot_total = 2.0f;
  check("rpm fine but venturis show no flow (blockage/leak) -> WARNING", sev(a) == AIR_WARNING);

  a = base(); a.n_valid = 3; a.mdot_total = 20.0f;   // 3 valid carrying 20 -> ~40 extrapolated
  check("flow check extrapolates from valid venturis (3 x ~6.7 g/s is fine)", sev(a) == AIR_ADVISORY);

  printf("link / emulation\n");
  a = base(); a.comp_link_ok = false;
  check("no Teensy replies while armed -> treated as telemetry lost", sev(a, &c) == AIR_ADVISORY && (c & AIR_C_TEENSY_LINK));
  a = base(); a.comp_err = ESCPID_ERR_EMULATION;
  check("emulation build: rpm unverified -> ADVISORY", sev(a, &c) == AIR_ADVISORY && (c & AIR_C_TLM_EMU));
  a = base(); a.comp_err = -10;
  check("ordinary telemetry error (-10) alone is not 'lost'", sev(a) == AIR_OK);

  printf("grace periods / states\n");
  a = base(); a.tlm_grace_done = false; a.flow_grace_done = false; a.comp_link_ok = false; a.mdot_total = 0.0f;
  check("just armed: link and flow not judged yet -> OK", sev(a) == AIR_OK);
  a = base(); a.flow_grace_done = false; a.mdot_total = 0.0f;
  check("during spin-up grace: low flow not flagged", sev(a) == AIR_OK);
  a = base(); a.armed = false; a.comp_link_ok = false; a.mdot_total = 0.0f;
  check("disarmed, healthy: Teensy silence and zero flow are normal -> OK", sev(a) == AIR_OK);
  a = base(); a.armed = false; a.n_valid = 5;
  check("disarmed with a bad venturi -> ADVISORY (pre-flight notice)", sev(a) == AIR_ADVISORY);
  a = base(); a.terminated = true; a.n_valid = 0; a.comp_link_ok = false;
  check("terminated -> OK (air off by design; mode shows TERMINATED)", sev(a) == AIR_OK);

  printf("debounce\n");
  const uint8_t W = AIR_C_TLM_LOST | AIR_C_VENT_LOST;
  AirDebounce d;
  check("starts OK", air_debounce(d, AIR_OK, 0, 0) == AIR_OK);
  air_debounce(d, AIR_WARNING, W, 1000);
  check("raise not before AIR_RAISE_MS", air_debounce(d, AIR_WARNING, W, 1000 + AIR_RAISE_MS - 1) == AIR_OK);
  check("raise at AIR_RAISE_MS", air_debounce(d, AIR_WARNING, W, 1000 + AIR_RAISE_MS) == AIR_WARNING);
  check("held = causes of the raising evaluation", d.held == W);
  air_debounce(d, AIR_OK, 0, 5000);
  check("fall not before AIR_FALL_MS", air_debounce(d, AIR_OK, 0, 5000 + AIR_FALL_MS - 1) == AIR_WARNING);
  check("still lit while falling: held keeps the reason", d.held == W);
  check("fall at AIR_FALL_MS", air_debounce(d, AIR_OK, 0, 5000 + AIR_FALL_MS) == AIR_OK);
  check("held cleared at OK", d.held == 0);
  AirDebounce f; uint8_t shown_max = 0;
  for (uint32_t t = 0; t < 5000; t += 10) {               // 100 ms blips of CAUTION
    uint8_t cand = ((t / 100) % 3 == 0) ? AIR_CAUTION : AIR_OK;
    uint8_t o = air_debounce(f, cand, cand ? AIR_C_VENT_LOST : 0, t); if (o > shown_max) shown_max = o;
  }
  check("100 ms blips never reach the display", shown_max == AIR_OK && f.held == 0);

  printf("held causes: intermittent fault holding a level (bench 2026-09-28)\n");
  // ADVISORY raised by lost telemetry; telemetry then recovers except a one-tick
  // blip every 500 ms. The fall timer never completes -> level stays lit, and the
  // instantaneous causes are 0 almost all the time. held must still name it.
  AirDebounce b; uint32_t t = 0;
  for (; t <= 1000; t += 10) air_debounce(b, AIR_ADVISORY, AIR_C_TLM_LOST, t);
  bool lit = true, named = true, inst_zero_seen = false;
  for (; t <= 6000; t += 10) {
    bool blip = (t % 500 == 0);
    uint8_t o = air_debounce(b, blip ? AIR_ADVISORY : AIR_OK, blip ? AIR_C_TLM_LOST : 0, t);
    lit &= (o == AIR_ADVISORY); named &= (b.held == AIR_C_TLM_LOST);
    if (!blip) inst_zero_seen = true;
  }
  check("blips every 500 ms keep ADVISORY lit (anti-flap as designed)", lit);
  check("...and held names TLM_LOST although 'now' is 0 between blips", named && inst_zero_seen);
  uint32_t last_blip = 6000;
  for (t += 10; t <= 6000 + AIR_FALL_MS + 20; t += 10) air_debounce(b, AIR_OK, 0, t);
  check("blips stop -> falls to OK one AIR_FALL_MS after the last blip, held cleared",
        b.out == AIR_OK && b.held == 0 && b.held_ms == last_blip);
  // a higher-level blip holding a lower display names the higher cause
  AirDebounce h;
  for (t = 0; t <= 1000; t += 10) air_debounce(h, AIR_ADVISORY, AIR_C_VENT_PARTIAL, t);
  air_debounce(h, AIR_CAUTION, AIR_C_VENT_LOST, t);                 // one-tick CAUTION blip
  air_debounce(h, AIR_OK, 0, t + 10);
  check("CAUTION blip under ADVISORY: display stays ADVISORY, held = VENT_LOST",
        h.out == AIR_ADVISORY && h.held == AIR_C_VENT_LOST);

  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
