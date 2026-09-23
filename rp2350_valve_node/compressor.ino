// =============================================================================
//  compressor.ino  --  Mass-flow -> rpm outer loop + Teensy stream  (core 0)
//
//  Slow (sub-1-Hz dynamics) rate-limited PI with anti-windup drives compressor
//  rpm to hit the aggregate mass-flow target. When the aggregate estimate is
//  untrustworthy, rpm pins to the COMPILED-IN immutable RPM_FALLBACK (30k) --
//  failing toward MORE airflow to preserve coanda authority. The command is
//  streamed to the Motor Teensy as a teensyshot Host_comm frame at 50 Hz.
//
//  Start / stop (decisions 2026-09-22, Q4):
//    * Stop = SILENCE. Disarmed or terminated -> no frames. The Teensy's link
//      watchdog (~40 ms) sends DShot MOTOR_STOP and the rotor COASTS down (no
//      active brake). There is no bench stream while disarmed (S1).
//    * Start = the Tiny sends the target immediately; the TEENSY rate-limits its
//      reference (~5 s to 30k), seeded from the MEASURED rpm, so a warm re-arm
//      after a Tiny reset picks a still-coasting rotor up where it is. A single
//      late/dropped frame (< ~80 ms) is held through on the Teensy.
//
//  Also computes the advisory ESC thermal-derate flag (Q2) for COMP_TLM.
// -----------------------------------------------------------------------------
#include "config.h"
#include "types.h"

// ---- outer-loop + fallback state ----
static float    comp_integ      = 0.0f;   // PI integral (rpm)
static float    comp_rpm        = 0.0f;   // slewed rpm actually commanded
static bool     comp_running    = false;
static uint32_t comp_last_us    = 0;

// flow-trust debounce
static bool     comp_flow_ok        = false;
static uint32_t comp_trust_since    = 0;
static uint32_t comp_untrust_since  = 0;

// sensor staleness (seq non-advance)
static uint32_t comp_last_seq       = 0;
static uint32_t comp_last_seq_ms    = 0;

// Teensy stream cadence
static uint32_t comp_last_stream_ms = 0;

// -------- little-endian packers for the 64-byte Host_comm --------
static inline void put_u16(uint8_t* b, int& i, uint16_t v){ b[i++]=v; b[i++]=v>>8; }
static inline void put_i16(uint8_t* b, int& i, int16_t v){ put_u16(b,i,(uint16_t)v); }
static inline void put_u32(uint8_t* b, int& i, uint32_t v){ b[i++]=v; b[i++]=v>>8; b[i++]=v>>16; b[i++]=v>>24; }

// Build and emit one teensyshot Host_comm frame (slot 0 driven, rest zero).
static void teensy_stream(int16_t rpm10) {
  uint8_t f[64]; int i = 0;
  put_u32(f, i, TEENSY_MAGIC);
  for (int s = 0; s < TEENSY_NB_SLOTS; ++s) put_i16(f, i, (s==0)? rpm10 : 0);       // RPM_r
  for (int s = 0; s < TEENSY_NB_SLOTS; ++s) put_u16(f, i, (s==0)? TEENSY_DEF_P:0);  // PID_P
  for (int s = 0; s < TEENSY_NB_SLOTS; ++s) put_u16(f, i, (s==0)? TEENSY_DEF_I:0);  // PID_I
  for (int s = 0; s < TEENSY_NB_SLOTS; ++s) put_u16(f, i, (s==0)? TEENSY_DEF_D:0);  // PID_D
  for (int s = 0; s < TEENSY_NB_SLOTS; ++s) put_u16(f, i, (s==0)? TEENSY_DEF_F:0);  // PID_f
  // i == 64
#if USE_REAL_TEENSY
  Serial1.write(f, sizeof(f));
#else
  (void)f;
#endif
}

// ---- ESC thermal-derate suspicion (advisory; Q2) ---------------------------
// Hot (>= COMP_THERM_ON_C) AND measured rpm sagging below target for
// COMP_THERM_SUSTAIN_MS, evaluated only while running past the spin-up grace.
// Clears when the ESC cools below COMP_THERM_OFF_C or rpm recovers.
static bool thermal_update(uint32_t now, bool running, uint32_t run_since, float target) {
  static bool     flag      = false;
  static uint32_t sag_since = 0;

  bool eval = running && g_comp.ok && target > 0.0f &&
              (now - run_since) >= COMP_SPINUP_GRACE_MS;
  if (!eval) { sag_since = 0; flag = false; return false; }

  float rpm = (float)g_comp.rpm10 * 10.0f;
  int   t   = (int)g_comp.temp_c;

  if (flag) {
    if (t < COMP_THERM_OFF_C || rpm >= (1.0f - COMP_THERM_OK_FRAC) * target) {
      flag = false; sag_since = 0;
    }
  } else {
    bool sag = (t >= COMP_THERM_ON_C) && (rpm < (1.0f - COMP_THERM_LAG_FRAC) * target);
    if (!sag)                 sag_since = 0;
    else if (sag_since == 0)  sag_since = now;
    else if ((now - sag_since) >= COMP_THERM_SUSTAIN_MS) flag = true;
  }
  return flag;
}

void compressor_update(uint32_t now) {
  // ---- dt ----
  uint32_t us = micros();
  if (comp_last_us == 0) comp_last_us = us;
  float dt = (us - comp_last_us) * 1e-6f;
  comp_last_us = us;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;   // guard first call / stalls

  // ---- read the latest sensor frame ----
  SensorFrame fr;
  bool got = g_sensor_pub.snapshot(fr);
  uint32_t seq = g_sensor_pub.sequence();
  if (seq != comp_last_seq) { comp_last_seq = seq; comp_last_seq_ms = now; }
  bool sensor_stale = (now - comp_last_seq_ms) > 50;   // >5 missed ticks
  if (!got) sensor_stale = true;

  float mdot_total = got ? fr.mdot_total : 0.0f;
  uint8_t n_valid  = got ? fr.n_valid   : 0;

  // ---- flow trust (debounced) ----
  bool raw_trust = (n_valid >= MIN_VALID_VALVES) &&
                   (mdot_total >= MDOT_PLAUSIBLE_MIN) &&
                   (mdot_total <= MDOT_PLAUSIBLE_MAX) &&
                   !sensor_stale;
  if (raw_trust) {
    comp_untrust_since = 0;
    if (comp_trust_since == 0) comp_trust_since = now;
    if (!comp_flow_ok && (now - comp_trust_since) >= FLOW_FALLBACK_HOLD_MS) comp_flow_ok = true;
  } else {
    comp_trust_since = 0;
    if (comp_untrust_since == 0) comp_untrust_since = now;
    if (comp_flow_ok && (now - comp_untrust_since) >= FLOW_FALLBACK_HOLD_MS) comp_flow_ok = false;
  }

  // ---- decide run + target rpm ----
  bool     want_run = g_status.armed;      // arm gates the compressor only
  float    target;
  CompMode mode;

  if (!want_run) {
    mode   = COMP_STOPPED;
    target = 0.0f;
  }
#if COMP_MDOT_LOOP
  // <<STUBBED OFF via COMP_MDOT_LOOP=0>> Mass-flow tracking PI. Do NOT re-enable
  // until dead-venturi handling exists: gate on sensor health and hold RPM_FALLBACK
  // when n_valid is too low -- never track a mdot estimate from too few valid
  // venturis. See tracker (compressor mdot loop / venturi-loss).
  else if (g_status.source == SRC_PRIMARY && comp_flow_ok) {
    // Track the mass-flow target with a gentle PI (anti-windup below).
    float err = g_status.mdot_target - mdot_total;
    float integ_try = comp_integ + FLOW_PI_KI * err * dt;
    float unsat = FLOW_PI_KP * err + integ_try;
    float clamped = unsat;
    if (clamped > RPM_CMD_MAX) clamped = RPM_CMD_MAX;
    if (clamped < RPM_CMD_MIN) clamped = RPM_CMD_MIN;
    if (clamped == unsat) comp_integ = integ_try;                 // not saturated: accept
    else                  comp_integ = clamped - FLOW_PI_KP*err;  // back-calc anti-windup
    target = clamped;
    mode   = COMP_TRACK;
  }
#endif
  else {
    // STUB: throttle controls nothing and the mdot loop is off, so every armed
    // source holds a fixed RPM. PRIMARY and SBUS both land here -> RPM_FALLBACK
    // (30k). Reports FALLBACK = "fixed RPM, not tracking flow" (honest telemetry).
    target = RPM_FALLBACK;
    mode   = COMP_FALLBACK;
  }

  // ---- run-edge handling: jump straight to the target (the Teensy ramps) ----
  static uint32_t run_since = 0;
  if (want_run && !comp_running) { comp_rpm = target; comp_integ = 0.0f; run_since = now; }
  comp_running = want_run;

  // ---- slew toward target, clamp ----
  float max_step = RPM_SLEW_PER_S * dt;
  float d = target - comp_rpm;
  if (d >  max_step) d =  max_step;
  if (d < -max_step) d = -max_step;
  comp_rpm += d;
  if (comp_rpm > RPM_CMD_MAX) comp_rpm = RPM_CMD_MAX;
  if (comp_rpm < RPM_CMD_MIN) comp_rpm = RPM_CMD_MIN;
  if (!want_run) comp_rpm = 0.0f;

  // ---- publish status + close the sim flow loop ----
  g_status.comp_mode     = mode;
  g_status.rpm_target    = (uint16_t)lroundf(comp_rpm);
  g_status.mdot_total    = mdot_total;
  g_status.n_valid       = n_valid;
  g_status.flow_fallback = (mode == COMP_FALLBACK);
  g_status.sensor_stale  = sensor_stale;
  g_current_rpm_cmd      = want_run ? (uint32_t)comp_rpm : 0;   // core 1 sim reads this
  g_status.comp_thermal  = thermal_update(now, want_run, run_since, comp_rpm);

  // ---- stream to Teensy at 50 Hz while running; SILENCE when stopped (= stop) ----
  if (want_run) {
    if ((now - comp_last_stream_ms) >= (1000 / TEENSY_STREAM_HZ)) {
      comp_last_stream_ms = now;
      teensy_stream((int16_t)lroundf(comp_rpm / 10.0f));   // firmware units: 10 rpm
    }
  }
}
