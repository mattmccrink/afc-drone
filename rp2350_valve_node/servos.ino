// =============================================================================
//  servos.ino  --  Curve-fit expansion + PCA9685 servo output  (core 1)
//
//  6 valve positions -> 12 servo microseconds via a per-servo cubic with a
//  per-valve gain/bias schedule (open decision #2), plus 4 traditional-surface
//  servos (linear center/throw/dir from config.h). Output goes to THREE PCA9685
//  drivers (A/B/C) over the shared I2C bus via auto-increment burst writes.
//  Layout (decision 2026-09-22): each PCA carries 4 valve servos on ch0-3;
//  surfaces on A4 (L wing), B4 (R wing), C4/C5 (L/R canard).
//
//  Manual bring-up: the console posts {g_servo_manual, dev, ch, us}; this tick
//  writes that one channel (single-channel write) so ALL I2C stays on core 1.
//  Manual mode bypasses the valve-command staleness failsafe -- bench only.
//
//  Failsafe (decision 2026-09-22, Q7): the PCA outputs are ALWAYS enabled (OE
//  held low from boot). On a stale/absent core-0 command -- including boot,
//  before the first allocation -- core 1 drives the DEFINED-SAFE valve pose
//  through the curve fit and centers the surfaces. Servos never lose their
//  pulse. If core 1 itself stalls, the PCA9685s keep emitting the last pulses
//  (hold-last) until the watchdog reset re-establishes the pose.
// =============================================================================
#include "config.h"
#include "types.h"
#include <Wire.h>

// ---- PCA9685 registers / bits ----
#define PCA_MODE1        0x00
#define PCA_MODE2        0x01
#define PCA_LED0_ON_L    0x06
#define PCA_ALL_OFF_H    0xFD
#define PCA_PRESCALE     0xFE
#define M1_RESTART       0x80
#define M1_AI            0x20   // register auto-increment (enables burst writes)
#define M1_SLEEP         0x10
#define M2_OUTDRV        0x04   // totem-pole outputs
#define PCA_PRESCALE_50HZ 121   // round(25MHz/(4096*50)) - 1

// Device index -> address / name (name "A"/"B"/"C" is what the console accepts).
static const uint8_t PCA_ADDRS[PCA9685_COUNT] = { ADDR_PCA9685_0, ADDR_PCA9685_1, ADDR_PCA9685_2 };

// ---------------------------------------------------------------------------
//  SERVO OUTPUT MAP  --  valve servo s -> { device s/4, channel s%4 }.
//  (servo -> valve assignment is g_servo_valve_map, from calibration.)
// ---------------------------------------------------------------------------
struct ServoOut { uint8_t dev; uint8_t ch; };
static const ServoOut SERVO_OUT_MAP[SERVO_COUNT] = {
  {0,0},{0,1},{0,2},{0,3},
  {1,0},{1,1},{1,2},{1,3},
  {2,0},{2,1},{2,2},{2,3},
};
static const ServoOut SURF_MAP[SURF_COUNT]       = SURF_OUT_MAP;
static const int16_t  SURF_CENTER[SURF_COUNT]    = SURF_US_CENTER;
static const int16_t  SURF_THROW[SURF_COUNT]     = SURF_US_THROW;
static const int8_t   SURF_SIGN[SURF_COUNT]      = SURF_DIR;
static const int16_t  SRV_SAFE_POSE[VALVE_COUNT] = DEFINED_SAFE_VALVE_POSE;

uint16_t g_surf_us_echo[SURF_COUNT] = { 0 };     // resolved surface us (core 1 only)

// microseconds -> 12-bit count at 50 Hz (period 20000 us)
static inline uint16_t us_to_count(uint16_t us) {
  uint32_t c = (uint32_t)us * 4096u / 20000u;
  return (c > 4095u) ? 4095u : (uint16_t)c;
}

#if USE_REAL_I2C
static bool s_pca_ok[PCA9685_COUNT] = { false };

static bool pca_write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
  
}

// Init one PCA9685: 50 Hz, auto-increment, totem-pole. Prescale must be set
// while asleep; wake needs ~500 us for the oscillator before RESTART.
static bool pca9685_init(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(PCA_MODE1);
  Wire.write(M1_SLEEP);
  uint8_t rc = Wire.endTransmission();
  if (rc != 0) { Serial.printf("[servo] 0x%02X MODE1 write rc=%u -> absent\n", addr, rc); return false; }
  pca_write8(addr, PCA_PRESCALE, PCA_PRESCALE_50HZ);
  pca_write8(addr, PCA_MODE2, M2_OUTDRV);
  pca_write8(addr, PCA_MODE1, M1_AI);                          // wake (clear SLEEP) + auto-inc
  delayMicroseconds(600);
  pca_write8(addr, PCA_MODE1, M1_AI | M1_RESTART);
  return true;
}

// Burst-write channels 0..nch-1: ON=0, OFF=count each (auto-increment).
static void pca9685_write_board(uint8_t addr, const uint16_t* counts, int nch) {
  Wire.beginTransmission(addr);
  Wire.write(PCA_LED0_ON_L);
  for (int c = 0; c < nch; ++c) {
    Wire.write(0x00);                          // ON_L
    Wire.write(0x00);                          // ON_H
    Wire.write(counts[c] & 0xFF);              // OFF_L
    Wire.write((counts[c] >> 8) & 0x0F);       // OFF_H
  }
  uint8_t rc = Wire.endTransmission();
  if (rc < 8) g_i2c_rc_hist[rc]++;
  if (rc)     g_i2c_last_fail_tk = g_core1_heartbeat;
}

// Single-channel write (manual bring-up).
static void pca9685_write_one(uint8_t addr, uint8_t ch, uint16_t us) {
  uint16_t count = us_to_count(us);
  Wire.beginTransmission(addr);
  Wire.write(PCA_LED0_ON_L + 4 * ch);
  Wire.write(0x00); Wire.write(0x00);
  Wire.write(count & 0xFF); Wire.write((count >> 8) & 0x0F);
  uint8_t rc = Wire.endTransmission();
  if (rc < 8) g_i2c_rc_hist[rc]++;
  if (rc)     g_i2c_last_fail_tk = g_core1_heartbeat;
}

#endif // USE_REAL_I2C

void servos_setup() {
  pinMode(PIN_OE_PCA, OUTPUT);
  digitalWrite(PIN_OE_PCA, LOW);       // outputs ENABLED, permanently (Q7)

#if USE_REAL_I2C
  for (int d = 0; d < PCA9685_COUNT; ++d) {           // scanner-equivalent addr probe
    Wire.beginTransmission(PCA_ADDRS[d]);
    uint8_t rc = Wire.endTransmission();
    Serial.printf("[servo] 0x%02X addr-probe rc=%u (%s)\n",
                  PCA_ADDRS[d], rc, rc == 0 ? "ACK" : "no-ACK");
  }
  for (int d = 0; d < PCA9685_COUNT; ++d) s_pca_ok[d] = pca9685_init(PCA_ADDRS[d]);
  servos_health_print();   // boot probe result (same format as 'health')
#endif
}

// Reprintable health summary for the servo drivers (boot-time probe, cached).
void servos_health_print() {
#if USE_REAL_I2C
  for (int d = 0; d < PCA9685_COUNT; ++d)
    Serial.printf("[health] PCA9685 %c 0x%02X @ i2c0(GP%d/GP%d): %s\n",
                  'A' + d, PCA_ADDRS[d], PIN_I2C_SDA, PIN_I2C_SCL,
                  s_pca_ok[d] ? "OK" : "absent");
#else
  Serial.println(F("[health] PCA9685: simulated (USE_REAL_I2C=0, not probed)"));
#endif
}

// Evaluate one servo's cubic for a normalized, gain/bias-shaped valve position.
static uint16_t curve_us(int servo, float x_norm) {
  int vv = g_servo_valve_map[servo];
  float xg = g_valve_gain[vv] * x_norm + g_valve_bias[vv];
  const float* c = g_servo_cubic[servo];
  float us = c[0] + c[1]*xg + c[2]*xg*xg + c[3]*xg*xg*xg;
  if (us < SERVO_US_MIN) us = SERVO_US_MIN;
  if (us > SERVO_US_MAX) us = SERVO_US_MAX;
  return (uint16_t)lroundf(us);
}

void servos_service(uint32_t tick) {
  // ---- manual bring-up override (bench): drive one posted channel ----
  if (g_servo_manual) {
#if USE_REAL_I2C
    for (int d = 0; d < PCA9685_COUNT; ++d) {
      if (!s_pca_ok[d]) continue;
      // find the highest populated channel on this device
      int nch = 0;
      for (int c = 0; c < PCA9685_MAX_CH; ++c)
        if (g_servo_man_tbl[d][c] != 0) nch = c + 1;
      if (nch == 0) continue;                       // nothing held on this board
      uint16_t counts[PCA9685_MAX_CH];
      for (int c = 0; c < nch; ++c) {
        uint16_t us = g_servo_man_tbl[d][c];
        counts[c] = us ? us_to_count(us) : us_to_count(SERVO_US_NEUTRAL);
      }
      pca9685_write_board(PCA_ADDRS[d], counts, nch);   // burst, actively re-asserted
    }
#endif
    return;   // NB: still bypasses the staleness failsafe -- bench only
  }

  // ---- normal path: curve-fit 12 valve servos + 4 surfaces, burst per device --
  // Hold the last good valve command across transient cross-core read misses. A
  // single dropped snapshot must NOT slam servos/valves to neutral -- that's a
  // control disturbance, not a safe state. The defined pose is used only on
  // SUSTAINED staleness (core 0 not updating), or before the first command.
  static ValveCmd s_last = {};                 // 0-init: stamp 0
  static bool     s_have = false;

  ValveCmd cmd;
  if (g_valve_pub.snapshot(cmd)) { s_last = cmd; s_have = true; }  // refresh only on a clean read
  // else: transient miss -> keep s_last (its stamp still measures core-0 liveness)

  bool stale = !s_have || (millis() - s_last.stamp_ms) > CMD_TIMEOUT_MS;

  if (stale) {
    // Boot or core-0 stall: defined-safe valve pose (through the curve fit),
    // surfaces centered. Outputs stay enabled -- the servos keep a pulse.
    for (int v = 0; v < VALVE_COUNT; ++v) cmd.valve[v] = SRV_SAFE_POSE[v];
    for (int k = 0; k < SURF_COUNT;  ++k) cmd.surf[k]  = 0;
  } else {
    cmd = s_last;                              // drive from the last good command
  }

  uint16_t us[SERVO_COUNT];
  for (int s = 0; s < SERVO_COUNT; ++s) {
    int   vv = g_servo_valve_map[s];
    float x  = (float)cmd.valve[vv] / (float)VALVE_POS_MAX;   // normalize to [-1,1]
    us[s] = curve_us(s, x);
    g_servo_us_echo[s] = us[s];                                // telemetry echo
  }
  uint16_t sus[SURF_COUNT];
  for (int k = 0; k < SURF_COUNT; ++k) {
    float x = (float)cmd.surf[k] / (float)VALVE_POS_MAX;      // [-1,1]
    float u = (float)SURF_CENTER[k] + (float)SURF_SIGN[k] * (float)SURF_THROW[k] * x;
    if (u < SERVO_US_MIN) u = SERVO_US_MIN;
    if (u > SERVO_US_MAX) u = SERVO_US_MAX;
    sus[k] = (uint16_t)lroundf(u);
    g_surf_us_echo[k] = sus[k];
  }

#if USE_REAL_I2C
  // Bin outputs into per-device channel arrays, then burst each board.
  uint16_t counts[PCA9685_COUNT][PCA9685_MAX_CH];
  for (int d = 0; d < PCA9685_COUNT; ++d)
    for (int c = 0; c < PCA9685_MAX_CH; ++c)
      counts[d][c] = us_to_count(SERVO_US_NEUTRAL);            // unmapped -> neutral
  for (int s = 0; s < SERVO_COUNT; ++s) {
    const ServoOut& o = SERVO_OUT_MAP[s];
    if (o.dev < PCA9685_COUNT && o.ch < PCA9685_MAX_CH) counts[o.dev][o.ch] = us_to_count(us[s]);
  }
  for (int k = 0; k < SURF_COUNT; ++k) {
    const ServoOut& o = SURF_MAP[k];
    if (o.dev < PCA9685_COUNT && o.ch < PCA9685_MAX_CH) counts[o.dev][o.ch] = us_to_count(sus[k]);
  }
  for (int d = 0; d < PCA9685_COUNT; ++d) {
    if (s_pca_ok[d]) pca9685_write_board(PCA_ADDRS[d], counts[d], PCA9685_MAX_CH);
  }
#endif
}
