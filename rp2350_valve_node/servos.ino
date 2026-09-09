// =============================================================================
//  servos.ino  --  Curve-fit expansion + PCA9685 servo output  (core 1)
//
//  6 valve positions -> 12 servo microseconds via a per-servo cubic with a
//  per-valve gain/bias schedule (open decision #2). Output goes to THREE PCA9685
//  drivers (A/B/C, <=6 servos each) over the shared I2C bus, via auto-increment
//  burst writes. Real I2C compiled only when USE_REAL_I2C == 1; absent boards
//  simply NACK.
//
//  Physical placement is the editable SERVO_OUT_MAP (servo -> device,channel) --
//  open decision, edit to your wiring.
//
//  Manual bring-up: the console posts {g_servo_manual, dev, ch, us}; this tick
//  writes that one channel (single-channel write) so ALL I2C stays on core 1.
//  Manual mode bypasses the valve-command staleness failsafe -- bench only.
//
//  Failsafe: OE pin (GPIO, active-low) is the primary hardware output-disable on
//  a stale command -- coanda domain only. Optional PCA9685 all-call ALL_LED_OFF
//  software backup is OFF by default (0x70 collides with a mux at 0x70).
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
//  SERVO OUTPUT MAP  --  EDIT to your wiring (open decision).
//  servo index -> { device 0..2, channel 0..PCA9685_MAX_CH-1 }
//  Default: servos 0-5 on device A, servos 6-11 on device B, C spare.
// ---------------------------------------------------------------------------
struct ServoOut { uint8_t dev; uint8_t ch; };
static const ServoOut SERVO_OUT_MAP[SERVO_COUNT] = {
  {0,0},{0,1},{0,2},{0,3},{0,4},{0,5},
  {1,0},{1,1},{1,2},{1,3},{1,4},{1,5},
};

static bool srv_oe_enabled = false;

// ---- OE domain control (active LOW: LOW = outputs enabled) ----
static void coanda_oe(bool enable) {
  if (enable == srv_oe_enabled) return;
  srv_oe_enabled = enable;
  digitalWrite(PIN_OE_COANDA, enable ? LOW : HIGH);
  // Emergency-surface OE (PIN_OE_EMERG) is a SEPARATE domain, never touched here.
}

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
  //if (rc)     g_i2c_last_fail_tk = tick;      // pass tick in, or read g_core1_heartbeat
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
  //if (rc)     g_i2c_last_fail_tk = tick;      // pass tick in, or read g_core1_heartbeat
}

#if USE_SERVO_ALLCALL_FAILSAFE
static void pca9685_all_off() {
  Wire.beginTransmission(ADDR_PCA9685_ALL);    // 0x70 -- collides with a mux at 0x70!
  Wire.write(PCA_ALL_OFF_H);
  Wire.write(0x10);                             // bit4 = full OFF on all channels
  Wire.endTransmission();
}
#endif
#endif // USE_REAL_I2C

void servos_setup() {
  pinMode(PIN_OE_COANDA, OUTPUT);
  pinMode(PIN_OE_EMERG,  OUTPUT);
  digitalWrite(PIN_OE_COANDA, HIGH);   // start DISABLED until a valid command
  digitalWrite(PIN_OE_EMERG,  HIGH);
  srv_oe_enabled = false;

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
    uint8_t d = g_servo_man_dev; if (d >= PCA9685_COUNT) d = 0;
#if USE_REAL_I2C
    if (s_pca_ok[d]) pca9685_write_one(PCA_ADDRS[d], g_servo_man_ch, g_servo_man_us);
#endif
    coanda_oe(true);                           // enable so the servo actually moves
    return;                                    // NB: bypasses the staleness failsafe
  }

  // ---- normal path: curve-fit all 12, burst per device ----
  ValveCmd cmd;
  bool got = g_valve_pub.snapshot(cmd);
  bool stale = !got || (millis() - cmd.stamp_ms) > CMD_TIMEOUT_MS;

  if (stale) {
    coanda_oe(false);                          // primary kill: hardware output-disable
#if USE_REAL_I2C && USE_SERVO_ALLCALL_FAILSAFE
    pca9685_all_off();
#endif
    for (int s = 0; s < SERVO_COUNT; ++s) g_servo_us_echo[s] = SERVO_US_NEUTRAL;
    return;
  }

  uint16_t us[SERVO_COUNT];
  for (int s = 0; s < SERVO_COUNT; ++s) {
    int   vv = g_servo_valve_map[s];
    float x  = (float)cmd.valve[vv] / (float)VALVE_POS_MAX;   // normalize to [-1,1]
    us[s] = curve_us(s, x);
    g_servo_us_echo[s] = us[s];                                // telemetry echo
  }

#if USE_REAL_I2C
  // Bin curve-fit outputs into per-device channel arrays, then burst each board.
  uint16_t counts[PCA9685_COUNT][PCA9685_MAX_CH];
  for (int d = 0; d < PCA9685_COUNT; ++d)
    for (int c = 0; c < PCA9685_MAX_CH; ++c)
      counts[d][c] = us_to_count(SERVO_US_NEUTRAL);            // unmapped -> neutral
  for (int s = 0; s < SERVO_COUNT; ++s) {
    const ServoOut& o = SERVO_OUT_MAP[s];
    if (o.dev < PCA9685_COUNT && o.ch < PCA9685_MAX_CH) counts[o.dev][o.ch] = us_to_count(us[s]);
  }
  for (int d = 0; d < PCA9685_COUNT; ++d) {
    if (s_pca_ok[d]) pca9685_write_board(PCA_ADDRS[d], counts[d], PCA9685_MAX_CH);
  }
#endif

  coanda_oe(true);
}
