// =============================================================================
//  sensors_sim.ino  --  Simulated MS5837 pipeline + venturi mass-flow  (core 1)
//
//  Reproduces the STRUCTURE of the real 100 Hz pipeline (read-back-then-kick,
//  D1 pressure every tick, D2 temperature every ~16 ticks) so the real driver
//  drops in behind the same tick without restructuring. Synthetic pressures
//  respond to the current compressor rpm (g_current_rpm_cmd) so the mass-flow
//  outer loop actually closes on the bench. Per-valve validity applies the same
//  guards the real sensors will (range + differential-pressure plausibility);
//  faults are injectable live from the console.
//
//  Replace the "synthetic conversion" blocks with real PCA9545 channel selects
//  + MS5837 D1/D2 reads (guarded by USE_REAL_I2C) when hardware arrives.
// -----------------------------------------------------------------------------
#include "config.h"
#include "types.h"

#if !USE_REAL_I2C    // ===== simulated sensor path (real path: sensors.ino) =====

// physical-ish constants for the synthetic model
static const float SIM_P_AMBIENT = 1013.25f;  // mbar
static const float SIM_K_PLENUM  = 120.0f;    // mbar upstream rise at full rpm
static const float SIM_K_DP      = 8.0f;      // mbar throat drop at full rpm
static const float SIM_KVENT     = 0.45f;     // g/s per sqrt(Pa)  (lumped Cd*A*sqrt(2 rho))
static const float DP_MIN_VALID  = 0.05f;     // mbar; below this dp is implausible
static const float P_RANGE_MIN   = 300.0f;    // mbar range check
static const float P_RANGE_MAX   = 1300.0f;
static const float T_RANGE_MIN   = -40.0f;    // °C range check
static const float T_RANGE_MAX   = 85.0f;

// schedule types
enum { KICK_D1 = 0, KICK_D2 = 1 };

// "current" (last read-back) values
static float s_pup[VALVE_COUNT], s_plo[VALVE_COUNT], s_tdie[VALVE_COUNT];
// "pending" values latched at kick, read back next tick
static float s_pend_pup[VALVE_COUNT], s_pend_plo[VALVE_COUNT], s_pend_tdie[VALVE_COUNT];
static bool  s_pend_have = false;
static int   s_pend_type = KICK_D1;

// cheap deterministic "noise" so we don't pull in RNG state
static inline float wobble(uint32_t tick, int v, float amp) {
  return amp * sinf(0.11f * tick + 1.7f * v);
}

void sensors_setup() {
  for (int v = 0; v < VALVE_COUNT; ++v) {
    s_pup[v]  = SIM_P_AMBIENT;
    s_plo[v]  = SIM_P_AMBIENT;
    s_tdie[v] = 25.0f;
  }
}

// Reprintable boot/health summary (matches the real path's signature).
void sensors_health_print() {
  Serial.println(F("[health] sensor path: SIMULATED (synthetic MS5837)"));
}

// Synthesize what the sensors WOULD read for the current operating point.
static void sim_convert(uint32_t tick, int type) {
  float rpm_frac = (float)g_current_rpm_cmd / (float)RPM_CMD_MAX;
  if (rpm_frac < 0) rpm_frac = 0; if (rpm_frac > 1) rpm_frac = 1;

  for (int v = 0; v < VALVE_COUNT; ++v) {
    if (type == KICK_D1) {
      float up = SIM_P_AMBIENT + SIM_K_PLENUM * rpm_frac + wobble(tick, v, 0.4f);
      // per-valve throat drop; a little per-valve spread so mdot[] differ
      float dp = SIM_K_DP * rpm_frac * (0.9f + 0.05f * v) + wobble(tick, v, 0.15f);
      if (dp < 0) dp = 0;
      s_pend_pup[v] = up;
      s_pend_plo[v] = up - dp;
    } else { // KICK_D2
      s_pend_tdie[v] = 25.0f + 8.0f * rpm_frac + wobble(tick, v, 0.2f);
    }
  }
}

void sensors_tick(uint32_t tick) {
  // ---- 1) read back the conversion kicked on the PREVIOUS tick ----
  if (s_pend_have) {
    if (s_pend_type == KICK_D1) {
      for (int v = 0; v < VALVE_COUNT; ++v) { s_pup[v] = s_pend_pup[v]; s_plo[v] = s_pend_plo[v]; }
    } else {
      for (int v = 0; v < VALVE_COUNT; ++v) { s_tdie[v] = s_pend_tdie[v]; }
    }
  }

  // ---- 2) compute mass flow + validity, build the frame ----
  SensorFrame& out = g_sensor_pub.begin_write();
  float total = 0.0f;
  uint8_t nvalid = 0;

  for (int v = 0; v < VALVE_COUNT; ++v) {
    float pu = s_pup[v], pl = s_plo[v], td = s_tdie[v];

    // inject faults: aggregate forces all invalid; per-valve forces one out of range
    if (g_fault_aggregate)      pu = 9999.0f;             // out of range -> invalid
    else if (g_fault_valve[v])  pu = 9999.0f;

    // guards (mirror the real sensor guards, spec section 11)
    bool range_ok = (pu >= P_RANGE_MIN && pu <= P_RANGE_MAX) &&
                    (pl >= P_RANGE_MIN && pl <= P_RANGE_MAX) &&
                    (td >= T_RANGE_MIN && td <= T_RANGE_MAX);
    float dp = pu - pl;                                   // mbar (throat below upstream)
    bool dp_ok = (dp >= DP_MIN_VALID);                    // near-zero / inverted -> implausible

    bool valid = range_ok && dp_ok;

    float mdot = 0.0f;
    if (valid) {
      float dp_pa = dp * 100.0f;                          // mbar -> Pa
      mdot = SIM_KVENT * sqrtf(dp_pa);                    // g/s  (guarded: dp_pa > 0)
      total += mdot;
      nvalid++;
    }

    out.p_up[v]  = pu;
    out.p_lo[v]  = pl;
    out.t_die[v] = td;
    out.t_lo[v]  = td;                                   // sim: one die temp for the pair
    out.mdot[v]  = mdot;
    out.valid[v] = valid ? 1 : 0;
  }

  out.mdot_total = total;      // summed over VALID valves only
  out.n_valid    = nvalid;
  for (int s = 0; s < SERVO_COUNT; ++s) out.servo_us[s] = g_servo_us_echo[s]; // telemetry echo
  g_sensor_pub.end_write();

  // ---- 3) kick the NEXT conversion (D1 every tick, D2 every ~16 ticks) ----
  int type = ((tick % D2_CADENCE_TICKS) == 0) ? KICK_D2 : KICK_D1;
  sim_convert(tick, type);
  s_pend_have = true;
  s_pend_type = type;
}

#endif // !USE_REAL_I2C
