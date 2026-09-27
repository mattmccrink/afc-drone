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
  AirDebounce d;
  check("starts OK", air_debounce(d, AIR_OK, 0) == AIR_OK);
  air_debounce(d, AIR_WARNING, 1000);
  check("raise not before AIR_RAISE_MS", air_debounce(d, AIR_WARNING, 1000 + AIR_RAISE_MS - 1) == AIR_OK);
  check("raise at AIR_RAISE_MS", air_debounce(d, AIR_WARNING, 1000 + AIR_RAISE_MS) == AIR_WARNING);
  air_debounce(d, AIR_OK, 5000);
  check("fall not before AIR_FALL_MS", air_debounce(d, AIR_OK, 5000 + AIR_FALL_MS - 1) == AIR_WARNING);
  check("fall at AIR_FALL_MS", air_debounce(d, AIR_OK, 5000 + AIR_FALL_MS) == AIR_OK);
  AirDebounce f; uint8_t shown_max = 0;
  for (uint32_t t = 0; t < 5000; t += 10) {               // 100 ms blips of CAUTION
    uint8_t cand = ((t / 100) % 3 == 0) ? AIR_CAUTION : AIR_OK;
    uint8_t o = air_debounce(f, cand, t); if (o > shown_max) shown_max = o;
  }
  check("100 ms blips never reach the display", shown_max == AIR_OK);

  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
