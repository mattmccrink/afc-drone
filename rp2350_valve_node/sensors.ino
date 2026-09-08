// =============================================================================
//  sensors.ino  --  REAL MS5837-02BA pipeline over PCA9548A  (core 1)
//
//  Compiled only when USE_REAL_I2C == 1 (otherwise sensors_sim.ino provides the
//  same seam). Two absolute -02BA per valve (upstream + throat); dp by
//  subtraction with a per-valve ground zero (g_dp_zero, from 'zero'); density
//  from the UPSTREAM absolute + its die temp. Between-sensor differential with
//  local static.
//
//  Pipeline: conversion-in-flight state machine. A conversion is kicked to all
//  sensors at once (broadcast: open every populated channel, one convert to
//  0x76), then read back sequentially CONV_LATENCY_TICKS later (one channel at a
//  time). Latency lets OSR 8192 (~18 ms) live under a 10 ms tick. The frame is
//  (re)published every tick so downstream staleness detection is unaffected.
//
//  Noise handling for the low-full-scale venturi: max OSR + a per-sensor EMA on
//  compensated pressure (SENS_EMA_ALPHA).
//
//  Mapping (open decision #1) is the editable table below. Convention: adjacent
//  channels per valve, lower channel = upstream (higher static) = ROLE_UP.
// =============================================================================
#include "config.h"
#include "types.h"

#if USE_REAL_I2C
#include <Wire.h>

// ---------------------------------------------------------------------------
//  SENSOR MAP  --  EDIT as the tree populates (open decision #1).
//  Bench default: two sensors on mux 0x70, ch0 = valve0 upstream, ch1 = throat.
// ---------------------------------------------------------------------------
static const SensorSlot SENSOR_MAP[] = {
  //Left wing
  { 0x70, 0, 0, ROLE_UP },
  { 0x70, 1, 0, ROLE_LO },
  { 0x70, 2, 1, ROLE_UP },
  { 0x70, 3, 1, ROLE_LO },
  //Right wing
  { 0x71, 0, 2, ROLE_UP },
  { 0x71, 1, 2, ROLE_LO },
  { 0x71, 2, 3, ROLE_UP },
  { 0x71, 3, 3, ROLE_LO },
  //Canard
  { 0x72, 0, 4, ROLE_UP },
  { 0x72, 1, 4, ROLE_LO },
  { 0x72, 2, 5, ROLE_UP },
  { 0x72, 3, 5, ROLE_LO },
  // ---- example once fully populated (adjacent pairs, up = lower channel) ----
  // { 0x70, 2, 1, ROLE_UP }, { 0x70, 3, 1, ROLE_LO },
  // { 0x70, 4, 2, ROLE_UP }, { 0x70, 5, 2, ROLE_LO },
  // { 0x70, 6, 3, ROLE_UP }, { 0x70, 7, 3, ROLE_LO },
  // { 0x71, 0, 4, ROLE_UP }, { 0x71, 1, 4, ROLE_LO },
  // { 0x71, 2, 5, ROLE_UP }, { 0x71, 3, 5, ROLE_LO },
};
static const int N_SLOTS = sizeof(SENSOR_MAP) / sizeof(SENSOR_MAP[0]);

// ---- per-slot caches ----
static uint16_t s_prom[SENSOR_COUNT][8];
static bool     s_prom_ok[SENSOR_COUNT];
static uint32_t s_rawD1[SENSOR_COUNT], s_rawD2[SENSOR_COUNT];
static bool     s_have1[SENSOR_COUNT], s_have2[SENSOR_COUNT];
static float    s_P[SENSOR_COUNT], s_T[SENSOR_COUNT];   // s_P is EMA-filtered
static bool     s_ema_init[SENSOR_COUNT];

// ---- per-valve slot lookup ----
static int8_t   s_up[VALVE_COUNT], s_lo[VALVE_COUNT];

// ---- Valve constants ----
static float K_VENTURI[6] =   {0.8963f,0.8963f,0.8963f,0.8963f,0.8963f,0.8963f};  // PROVISIONAL from bench venturi (~8 g/s @ ~850 Pa,

// ---- unique mux list ----
static uint8_t  s_mux[8]; static int s_nmux = 0;

// ---- pipeline state (conversion-in-flight; supports multi-tick latency) ----
static bool     s_conv_pending    = false;
static uint32_t s_conv_ready_tick = 0;
static int      s_conv_type       = 0;    // 0 = D1 pressure, 1 = D2 temperature
static uint32_t s_conv_count      = 0;    // conversions issued (drives D2 cadence)

// ---------------------------------------------------------------------------
//  MS5837-02BA compensation (validated at bring-up). 64-bit intermediates.
// ---------------------------------------------------------------------------
static void ms_compensate(const uint16_t prom[8], uint32_t D1, uint32_t D2,
                          float& p_mbar, float& t_C) {
  int64_t C1=prom[1], C2=prom[2], C3=prom[3], C4=prom[4], C5=prom[5], C6=prom[6];
  int64_t dT   = (int64_t)D2 - (C5 << 8);
  int64_t TEMP = 2000 + (dT * C6) / 8388608LL;
  int64_t OFF  = (C2 << 17) + (C4 * dT) / 64;
  int64_t SENS = (C1 << 16) + (C3 * dT) / 128;
  int64_t Ti=0, OFFi=0, SENSi=0;
  if (TEMP < 2000) {
    int64_t d = TEMP - 2000;
    Ti    = (11 * dT * dT) >> 35;
    OFFi  = (31 * d * d) >> 3;
    SENSi = (63 * d * d) >> 5;
  }
  OFF -= OFFi; SENS -= SENSi;
  int64_t P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;
  p_mbar = P / 100.0f;
  t_C    = (TEMP - Ti) / 100.0f;
}

// ---------------------------------------------------------------------------
//  Mux + MS5837 low level
// ---------------------------------------------------------------------------
static void mux_select_only(uint8_t target, uint8_t ch) {
  for (int i = 0; i < s_nmux; ++i) {
    Wire.beginTransmission(s_mux[i]);
    Wire.write(s_mux[i] == target ? (uint8_t)(1u << ch) : (uint8_t)0x00);
    Wire.endTransmission();
  }
}

static void mux_open_all_populated() {
  for (int i = 0; i < s_nmux; ++i) {
    uint8_t mask = 0;
    for (int k = 0; k < N_SLOTS; ++k) if (SENSOR_MAP[k].mux == s_mux[i]) mask |= (1u << SENSOR_MAP[k].ch);
    Wire.beginTransmission(s_mux[i]);
    Wire.write(mask);
    Wire.endTransmission();
  }
}

static bool ms_read_adc(uint32_t& out) {
  Wire.beginTransmission(ADDR_MS5837);
  Wire.write(MS_ADC_READ_CMD);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)ADDR_MS5837, 3) != 3) return false;
  out = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  return true;
}

static uint8_t ms_crc4(uint16_t prom[8]) {
  uint16_t rem = 0, saved0 = prom[0];
  prom[0] &= 0x0FFF; prom[7] = 0;
  for (int cnt = 0; cnt < 16; cnt++) {
    if (cnt % 2 == 1) rem ^= (uint16_t)(prom[cnt >> 1] & 0x00FF);
    else              rem ^= (uint16_t)(prom[cnt >> 1] >> 8);
    for (uint8_t b = 8; b > 0; b--)
      rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
  }
  prom[0] = saved0;
  return (uint8_t)((rem >> 12) & 0x0F);
}

static bool ms_read_prom(int slot) {
  Wire.beginTransmission(ADDR_MS5837); Wire.write(MS_RESET_CMD); Wire.endTransmission();
  delay(12);
  for (uint8_t w = 0; w < 7; ++w) {
    Wire.beginTransmission(ADDR_MS5837); Wire.write((uint8_t)(0xA0 + w * 2));
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((int)ADDR_MS5837, 2) != 2) return false;
    s_prom[slot][w] = ((uint16_t)Wire.read() << 8) | Wire.read();
  }
  uint16_t tmp[8]; memcpy(tmp, s_prom[slot], sizeof(tmp));
  return ((s_prom[slot][0] >> 12) & 0x0F) == ms_crc4(tmp);
}

// Reprintable boot/health summary for the sensor path (also called at setup).
void sensors_health_print() {
  Serial.println(F("[health] sensor path: REAL (MS5837-02BA)"));
  for (int k = 0; k < N_SLOTS; ++k)
    Serial.printf("[health] slot%d mux0x%02X ch%u valve%u %s : PROM %s\n",
                  k, SENSOR_MAP[k].mux, SENSOR_MAP[k].ch, SENSOR_MAP[k].valve,
                  SENSOR_MAP[k].role == ROLE_UP ? "UP" : "LO",
                  s_prom_ok[k] ? "OK" : "FAIL");
}

// ---------------------------------------------------------------------------
//  Setup: build mux list + valve lookup, cache PROM per sensor
// ---------------------------------------------------------------------------
void sensors_setup() {
  for (int v = 0; v < VALVE_COUNT; ++v) { s_up[v] = -1; s_lo[v] = -1; }
  s_nmux = 0;

  for (int k = 0; k < N_SLOTS; ++k) {
    const SensorSlot& s = SENSOR_MAP[k];
    if (s.role == ROLE_UP) s_up[s.valve] = k; else s_lo[s.valve] = k;
    bool seen = false;
    for (int i = 0; i < s_nmux; ++i) if (s_mux[i] == s.mux) seen = true;
    if (!seen && s_nmux < (int)sizeof(s_mux)) s_mux[s_nmux++] = s.mux;
    s_have1[k] = s_have2[k] = false;
    s_ema_init[k] = false;
  }

  for (int k = 0; k < N_SLOTS; ++k) {
    mux_select_only(SENSOR_MAP[k].mux, SENSOR_MAP[k].ch);
    s_prom_ok[k] = ms_read_prom(k);
  }
  sensors_health_print();   // best-effort at boot; also available via 'health'
}

// ---------------------------------------------------------------------------
//  100 Hz tick
// ---------------------------------------------------------------------------
void sensors_tick(uint32_t tick) {
  // ---- 1) read back a conversion that has become ready ----
  if (s_conv_pending && (int32_t)(tick - s_conv_ready_tick) >= 0) {
    for (int k = 0; k < N_SLOTS; ++k) {
      if (!s_prom_ok[k]) continue;
      mux_select_only(SENSOR_MAP[k].mux, SENSOR_MAP[k].ch);
      uint32_t raw;
      if (ms_read_adc(raw)) {
        if (s_conv_type == 0) { s_rawD1[k] = raw; s_have1[k] = true; }
        else                  { s_rawD2[k] = raw; s_have2[k] = true; }
      }
      if (s_have1[k] && s_have2[k]) {
        float P, T;
        ms_compensate(s_prom[k], s_rawD1[k], s_rawD2[k], P, T);
        if (!s_ema_init[k]) { s_P[k] = P; s_ema_init[k] = true; }
        else                  s_P[k] += SENS_EMA_ALPHA * (P - s_P[k]);   // EMA on pressure
        s_T[k] = T;                                                      // temp slow: unfiltered
      }
    }
    s_conv_pending = false;
  }

  // ---- 2) assemble + publish per-valve frame (every tick; holds between updates) ----
  SensorFrame& out = g_sensor_pub.begin_write();
  float total = 0.0f; uint8_t nvalid = 0;

  for (int v = 0; v < VALVE_COUNT; ++v) {
    int up = s_up[v], lo = s_lo[v];
    bool paired = (up >= 0 && lo >= 0 && s_prom_ok[up] && s_prom_ok[lo] && s_have1[up] && s_have1[lo]);

    float pu = paired ? s_P[up] : 0.0f;
    float pl = paired ? s_P[lo] : 0.0f;
    float td = paired ? s_T[up] : 0.0f;   // density temp from the upstream sensor

    bool range_ok = paired &&
                    pu >= MS_P_RANGE_MIN && pu <= MS_P_RANGE_MAX &&
                    pl >= MS_P_RANGE_MIN && pl <= MS_P_RANGE_MAX &&
                    td >= MS_T_RANGE_MIN && td <= MS_T_RANGE_MAX;

    // VALIDITY = sensor health only. Flow presence is a separate deadband test,
    // so a healthy sensor at low/zero flow still counts toward n_valid.
    bool valid = range_ok && !g_fault_valve[v] && !g_fault_aggregate;


 //float pu_adjusted =  pu - g_dp_zero[v];

    float dp = -((pu - pl) - g_dp_zero[v]);          // zero-corrected differential (mbar)
//float dp = pl - pu_adjusted; // - g_dp_zero[v];          // zero-corrected differential (mbar)

    float mdot = 0.0f;
    if (valid) {
      if (dp >= VENTURI_DP_MIN) {                 // above the noise deadband -> real flow
        float rho = (pu * 100.0f) / (R_AIR * (td + 273.15f));   // ideal gas, upstream static
        mdot = K_VENTURI[v] * AREA_INLET* rho * sqrtf(2.0f * dp * 100.0f/ (rho * (AREA_RATIO * AREA_RATIO - 1) )) *1000;     // dp mbar -> Pa; guarded dp>0, mdot kg-->g is *1000
      }
      total += mdot;                              // 0 when below deadband (no phantom flow)
      nvalid++;                                   // healthy sensor counts, flowing or not
    }

    out.p_up[v]  = pu;
    out.p_lo[v]  = pl;
    out.t_die[v] = td;                                   // upstream (density temp)
    out.t_lo[v]  = paired ? s_T[lo] : 0.0f;              // throat (cal/tempco)
    out.mdot[v]  = mdot;
    out.valid[v] = valid ? 1 : 0;
  }

  out.mdot_total = total;
  out.n_valid    = nvalid;
  for (int s = 0; s < SERVO_COUNT; ++s) out.servo_us[s] = g_servo_us_echo[s];
  g_sensor_pub.end_write();

  // ---- 3) if nothing is in flight, kick the next conversion (broadcast) ----
  if (!s_conv_pending) {
    int type = ((s_conv_count % D2_CADENCE_TICKS) == 0) ? 1 : 0;   // D2 temp vs D1 pressure
    mux_open_all_populated();
    Wire.beginTransmission(ADDR_MS5837);
    Wire.write(type ? MS_D2_CMD : MS_D1_CMD);
    Wire.endTransmission();
    s_conv_type       = type;
    s_conv_ready_tick = tick + CONV_LATENCY_TICKS;
    s_conv_pending    = true;
    s_conv_count++;
  }
}

#endif // USE_REAL_I2C
