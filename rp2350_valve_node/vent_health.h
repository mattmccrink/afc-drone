// =============================================================================
//  vent_health.h  --  per-sensor and per-venturi health state machines
//
//  Pure logic (no I2C, no Arduino calls) so it runs unchanged in the host test
//  (host_tools/test_vent_health.cpp). sensors.ino owns one SlotHealth per MS5837
//  and one VentGate per venturi, all on core 1.
//
//  Slot (one sensor):
//    * every read-back attempt is reported with sh_on_read(); a failed transfer,
//      a 0 / 0xFFFFFF result (MS5837 returns 0 if read with no conversion
//      running), or a SPIKE (D1 / D2 jump > SENS_D1/D2_JUMP_MAX from the
//      previous good read) counts as a FAIL and refreshes nothing. A jump is
//      accepted as a REAL STEP when the next read confirms it (lands within the
//      jump limit of the rejected value): a real step costs one read (~30 ms),
//      a one-off corrupt read is dropped. sh_on_read() returns whether the value
//      may be used;
//    * STALE  = no good D1 within SENS_STALE_MS, or no good D2 within
//               SENS_T_STALE_MS (the node would otherwise keep publishing the
//               last value forever -- the pre-2026-09-26 behaviour);
//    * FROZEN = SENS_FROZEN_N identical consecutive good D1 counts;
//    * LOSSY  = sustained read-failure rate above ~1/(1+SENS_FAIL_WEIGHT)
//               (leaky score), for a sensor that answers often enough to
//               dodge STALE but loses a large share of its reads.
//  Venturi: first failing reason wins; any fault arms a SENS_RECOVER_MS
//  clean-time gate before the venturi counts as valid again (anti-flap).
// =============================================================================
#pragma once
#include <stdint.h>
#include "config.h"
#include "types.h"

struct SlotHealth {
  bool     seen_d1 = false, seen_d2 = false;
  uint32_t last_d1_ms = 0, last_d2_ms = 0;
  uint32_t prev_d1 = 0;
  uint32_t prev_d2 = 0;
  uint32_t cand[2] = {0, 0};     // last REJECTED jump per type (D1, D2): step confirmation
  bool     has_cand[2] = {false, false};
  uint16_t same_d1 = 0;          // identical consecutive good D1 counts
  uint16_t fail_score = 0;       // leaky failure score (LOSSY)
  uint8_t  state = VH_NODATA;    // last evaluated slot state
  // counters (monotonic; read by the console on core 0 -- diagnostic only)
  uint32_t n_ok = 0, n_fail = 0, n_spike = 0;
  uint32_t n_stale_ev = 0, n_frozen_ev = 0, n_lossy_ev = 0;
};

struct VentGate {
  uint32_t bad_until_ms = 0;     // valid only once now >= this
  bool     armed = false;        // a fault has been seen at least once
};

inline bool sh_raw_ok(bool xfer_ok, uint32_t raw) {
  return xfer_ok && raw != 0u && raw != 0xFFFFFFu;
}

static inline void sh_fail(SlotHealth& h) {
  h.n_fail++;
  uint32_t s = (uint32_t)h.fail_score + SENS_FAIL_WEIGHT;
  h.fail_score = (uint16_t)(s > 2u * SENS_FAIL_TRIP ? 2u * SENS_FAIL_TRIP : s);   // capped
}

// Report one read-back attempt. type: 0 = D1 (pressure), 1 = D2 (temperature).
// Returns true if the raw value may be used (stored + compensated).
static inline uint32_t absdiff(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }

inline bool sh_on_read(SlotHealth& h, int type, bool xfer_ok, uint32_t raw, uint32_t now) {
  if (!sh_raw_ok(xfer_ok, raw)) { sh_fail(h); return false; }
  const int t = type ? 1 : 0;
  const bool     seen = t ? h.seen_d2 : h.seen_d1;
  const uint32_t prev = t ? h.prev_d2 : h.prev_d1;
  const uint32_t age  = now - (t ? h.last_d2_ms : h.last_d1_ms);
  const uint32_t jmax = t ? SENS_D2_JUMP_MAX : SENS_D1_JUMP_MAX;
  const uint32_t amax = t ? SENS_T_STALE_MS : SENS_STALE_MS;
  if (seen && age <= amax && absdiff(raw, prev) > jmax) {
    bool confirmed = h.has_cand[t] && absdiff(raw, h.cand[t]) <= jmax;
    if (!confirmed) {                      // first read at a new level: hold it
      h.cand[t] = raw; h.has_cand[t] = true;
      h.n_spike++; sh_fail(h); return false;
    }
    // else: two consecutive reads agree on the new level -> real step, accept
  }
  h.has_cand[t] = false;
  h.n_ok++;
  if (h.fail_score) h.fail_score--;
  if (type == 0) {
    if (h.seen_d1 && raw == h.prev_d1) { if (h.same_d1 < 0xFFFF) h.same_d1++; }
    else                                 h.same_d1 = 0;
    h.prev_d1    = raw;
    h.last_d1_ms = now;
    h.seen_d1    = true;
  } else {
    h.prev_d2    = raw;
    h.last_d2_ms = now;
    h.seen_d2    = true;
  }
  return true;
}

// Evaluate one sensor. Counts STALE / FROZEN entry events.
inline uint8_t sh_eval(SlotHealth& h, bool prom_ok, uint32_t now) {
  uint8_t s;
  if (!prom_ok)                                        s = VH_NOPROM;
  else if (!h.seen_d1 || !h.seen_d2)                   s = VH_NODATA;
  else if ((uint32_t)(now - h.last_d1_ms) > SENS_STALE_MS ||
           (uint32_t)(now - h.last_d2_ms) > SENS_T_STALE_MS) s = VH_STALE;
  else if (h.same_d1 >= SENS_FROZEN_N)                 s = VH_FROZEN;
  else if (h.fail_score >= SENS_FAIL_TRIP)             s = VH_LOSSY;
  else                                                 s = VH_OK;
  if (s != h.state) {
    if (s == VH_STALE)  h.n_stale_ev++;
    if (s == VH_FROZEN) h.n_frozen_ev++;
    if (s == VH_LOSSY)  h.n_lossy_ev++;
    h.state = s;
  }
  return s;
}

// Venturi-level inputs, already reduced to numbers by the caller.
struct VentInputs {
  bool    mapped;      // both up and lo slots exist in SENSOR_MAP
  uint8_t sh_up, sh_lo;// slot states from sh_eval
  bool    injected;    // console fault injection
  bool    range_ok;    // p/T within MS_P_RANGE_* / MS_T_RANGE_*
  float   dp_corr;     // (p_up - p_lo) - g_dp_zero, mbar
};

inline uint8_t vent_reason(const VentInputs& in) {
  if (!in.mapped)                       return VH_UNMAPPED;
  if (in.injected)                      return VH_INJECTED;
  if (in.sh_up != VH_OK)                return in.sh_up;
  if (in.sh_lo != VH_OK)                return in.sh_lo;
  if (!in.range_ok)                     return VH_RANGE;
  if (in.dp_corr < -VENTURI_DP_NEG_MAX) return VH_DPNEG;
  return VH_OK;
}

// Anti-flap gate: a fault (re)arms SENS_RECOVER_MS of required clean time.
inline uint8_t vent_gate(VentGate& g, uint8_t why, uint32_t now) {
  if (why != VH_OK) { g.bad_until_ms = now + SENS_RECOVER_MS; g.armed = true; return why; }
  if (g.armed) {
    if ((int32_t)(g.bad_until_ms - now) > 0) return VH_RECOVER;
    g.armed = false;   // disarm once served, so the signed compare can't re-trip ~24.8 days later
  }
  return VH_OK;
}
