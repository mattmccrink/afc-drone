// =============================================================================
//  air_state.h  --  "is air being delivered?" severity (core 0)
//
//  Pure logic (no Arduino calls) so it runs unchanged in the host test
//  (host_tools/test_air_state.cpp). compressor_update() feeds it once per loop.
//
//  Two independent views of air delivery:
//    * ESC telemetry (Teensy): rpm tracked, or open loop (err -11), or the
//      Teensy link itself silent while armed;
//    * venturis (Tiny): measured mass flow, per-venturi validity.
//  Severity ranks how well air delivery is CONFIRMED, not which sensor failed
//  (decision 2026-09-27). Lost telemetry with healthy venturis is only an
//  advisory -- flow is still measured, and the (future) mdot loop can still
//  drive the compressor through the Teensy's target-scaled open loop. Lost
//  venturis mean we don't know whether air is flowing: caution. Neither, or
//  venturis reporting no flow while running: warning (engage surfaces).
//
//    armed, not terminated:
//      WARNING  (tlm lost AND venturis lost)  or  flow low (any telemetry state)
//      CAUTION  venturis lost (tlm ok)        or  tlm lost AND venturis partial
//      ADVISORY tlm lost AND venturis all ok  or  venturis partial (tlm ok)
//    disarmed: venturis partial/lost -> ADVISORY (pre-flight notice); else OK
//    terminated: OK (air is off by design; the MODE pill says TERMINATED)
//
//  Reporting only: nothing here changes control.
// =============================================================================
#pragma once
#include <stdint.h>
#include "config.h"

enum AirSev : uint8_t { AIR_OK = 0, AIR_ADVISORY = 1, AIR_CAUTION = 2, AIR_WARNING = 3 };

// Cause bits (CTRL_TLM byte 55, /afc/ctrl_tlm.air_causes -- the HELD causes, see air_debounce)
#define AIR_C_TLM_LOST      0x01   // Teensy reports no ESC telemetry (err -11, open loop)
#define AIR_C_TLM_EMU       0x02   // Teensy is an ESC-emulation build (err -12): rpm is fake
#define AIR_C_VENT_PARTIAL  0x04   // some venturis invalid
#define AIR_C_VENT_LOST     0x08   // no valid venturis (or sensor frame stale)
#define AIR_C_FLOW_LOW      0x10   // venturis healthy but flow implausibly low while running
#define AIR_C_SENSOR_STALE  0x20   // core-1 sensor frame not updating
#define AIR_C_TEENSY_LINK   0x40   // no reply from the Teensy while armed

#define ESCPID_ERR_NO_TLM     (-11)   // mirror of teensy_escpid ESCPID_ERROR_NO_TLM
#define ESCPID_ERR_EMULATION  (-12)   // mirror of teensy_escpid ESCPID_ERROR_EMULATION

struct AirInputs {
  bool    armed;
  bool    terminated;
  bool    tlm_grace_done;    // running >= AIR_TLM_GRACE_MS (Teensy acquisition ~0.5 s)
  bool    flow_grace_done;   // running >= COMP_SPINUP_GRACE_MS (flow has had time to build)
  bool    comp_link_ok;      // g_comp.ok: Teensy replied within its stale window
  int8_t  comp_err;          // g_comp.err
  bool    sensor_stale;      // core-1 SensorFrame not advancing
  uint8_t n_valid;           // valid venturis (0..VALVE_COUNT)
  float   mdot_total;        // g/s, sum over valid venturis
};

inline uint8_t air_eval(const AirInputs& in, uint8_t* causes_out) {
  uint8_t c = 0;
  const bool vent_lost    = in.sensor_stale || in.n_valid == 0;
  const bool vent_partial = !vent_lost && in.n_valid < VALVE_COUNT;
  if (in.sensor_stale) c |= AIR_C_SENSOR_STALE;
  if (vent_lost)       c |= AIR_C_VENT_LOST;
  if (vent_partial)    c |= AIR_C_VENT_PARTIAL;

  if (in.terminated) { *causes_out = c; return AIR_OK; }
  if (!in.armed) {
    *causes_out = c;
    return (vent_lost || vent_partial) ? AIR_ADVISORY : AIR_OK;
  }

  bool tlm_lost = false;
  if (in.tlm_grace_done) {
    if (!in.comp_link_ok)                          { tlm_lost = true; c |= AIR_C_TEENSY_LINK; }
    else if (in.comp_err == ESCPID_ERR_NO_TLM)     { tlm_lost = true; c |= AIR_C_TLM_LOST; }
    else if (in.comp_err == ESCPID_ERR_EMULATION)  { tlm_lost = true; c |= AIR_C_TLM_EMU; }
  }

  // Flow plausibility, extrapolated from the valid venturis to all of them.
  bool flow_low = false;
  if (in.flow_grace_done && !vent_lost) {
    float est = in.mdot_total * (float)VALVE_COUNT / (float)in.n_valid;
    if (est < AIR_MDOT_MIN_GPS) { flow_low = true; c |= AIR_C_FLOW_LOW; }
  }

  *causes_out = c;
  if (flow_low || (tlm_lost && vent_lost))  return AIR_WARNING;
  if (vent_lost || (tlm_lost && vent_partial)) return AIR_CAUTION;
  if (tlm_lost || vent_partial)             return AIR_ADVISORY;
  return AIR_OK;
}

// Debounce: a new level must persist AIR_RAISE_MS to raise, AIR_FALL_MS to lower.
//
// `held` = the causes of the most recent evaluation that supported the level on
// display (cand >= out); cleared when the display returns to OK. The fall delay
// means a level stays lit while its fault recurs at least once per AIR_FALL_MS,
// even if the fault is absent at any given instant (an intermittent blip re-arms
// the timer). Reporting `held` rather than the instantaneous causes means a lit
// annunciator / QGC alert always carries the reason it is lit. held_ms = when
// that supporting evaluation happened (for 'last seen N ms ago').
struct AirDebounce {
  uint8_t  out = AIR_OK, cand = AIR_OK; uint32_t since = 0;
  uint8_t  held = 0;                    uint32_t held_ms = 0;
};

inline uint8_t air_debounce(AirDebounce& d, uint8_t cand, uint8_t causes, uint32_t now) {
  if (cand == d.out)       { d.cand = cand; d.since = now; }
  else if (cand != d.cand) { d.cand = cand; d.since = now; }
  else {
    uint32_t need = (cand > d.out) ? AIR_RAISE_MS : AIR_FALL_MS;
    if ((uint32_t)(now - d.since) >= need) d.out = cand;
  }
  if (d.out == AIR_OK)     { d.held = 0; }
  else if (cand >= d.out)  { d.held = causes; d.held_ms = now; }
  return d.out;
}

inline const char* air_name(uint8_t s) {
  switch (s) { case AIR_OK: return "OK"; case AIR_ADVISORY: return "ADVISORY";
               case AIR_CAUTION: return "CAUTION"; case AIR_WARNING: return "WARNING"; }
  return "?";
}
