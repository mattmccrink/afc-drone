// =============================================================================
//  bench_i2c.ino  --  'i2cstick' bench hook: emulate a device holding the I2C bus
//
//  Forces SDA or SCL low AT THE PAD with the RP2350's per-pin output overrides,
//  underneath whatever function owns the pin (the I2C peripheral, or SIO during
//  bus_clear). Electrically this is a slave holding the line: the lines are
//  open-drain with pull-ups, so low is a normal state and nothing contends.
//  No wiring and no need to open the box.
//
//    i2cstick sda|scl <ms>       hold for <ms> (1..60000), then release (transient)
//    i2cstick sda|scl hold       hold until 'i2cstick off' or a reset
//    i2cstick sda|scl persist N  hold now AND re-assert at each of the next N boots
//                                (1..20): a device that stays stuck through the
//                                watchdog reset. Watchdog scratch[0] survives a
//                                watchdog reset; a power cycle clears it.
//    i2cstick off                release now and cancel any persist
//    i2cstick probe on|off       core 1 also writes to an (absent is fine) address
//                                PROBE_PER_TICK times per 10 ms tick, so the test
//                                has traffic with no devices on the bus; the release
//                                line then reports the slowest single transaction
//
//  Compiled only with BENCH_HOOKS && USE_REAL_I2C (never in a flight build).
// =============================================================================
#include "config.h"

#if BENCH_HOOKS && USE_REAL_I2C
#include "hardware/gpio.h"
#include "hardware/structs/watchdog.h"

static const uint32_t STICK_MAGIC = 0x5C1C0000u;   // scratch[0]: magic | boots_left<<8 | pin
static int      s_stick_pin      = -1;
static uint32_t s_stick_until    = 0;              // millis() release time; 0 = no timed release
static int      s_stick_boot_left = -1;            // >=0: re-asserted at this boot, N boots left

// Evidence that the hold actually hit I2C traffic, and how close core 1 came to the
// watchdog: servo-write result counts (g_i2c_rc_hist) and the longest core-1
// heartbeat gap seen from core 0 while the line was held.
extern volatile uint32_t g_i2c_rc_hist[8];
static uint32_t s_rc0[8];
static uint32_t s_hold_t0 = 0, s_hb_last = 0, s_hb_t = 0, s_hb_gap_max = 0;

// ---- probe traffic (core 1 writes; core 0 only reads these) ----
#define PROBE_ADDR      0x40        // first PCA9685 address; NACK when absent
#define PROBE_PER_TICK  4
volatile bool     g_i2c_probe = false;
volatile uint32_t g_probe_n = 0, g_probe_ok = 0, g_probe_timeout = 0, g_probe_err = 0;
volatile uint32_t g_probe_max_us = 0;
static uint32_t s_pn0, s_pok0, s_pto0, s_perr0;

// Called from loop1() each tick (core 1 owns Wire).
void bench_i2c_probe_tick() {
  if (!g_i2c_probe) return;
  for (int k = 0; k < PROBE_PER_TICK; ++k) {
    uint32_t t0 = micros();
    Wire.beginTransmission(PROBE_ADDR);
    Wire.write((uint8_t)0x00);                     // MODE1 register pointer: harmless
    uint8_t rc = Wire.endTransmission();
    uint32_t dt = micros() - t0;
    g_probe_n++;
    if (rc == 0) g_probe_ok++; else if (rc == 5) g_probe_timeout++; else g_probe_err++;
    if (dt > g_probe_max_us) g_probe_max_us = dt;
  }
}

static void stick_stats_begin(uint32_t now) {
  for (int i = 0; i < 8; ++i) s_rc0[i] = g_i2c_rc_hist[i];
  s_hold_t0 = now; s_hb_last = g_core1_heartbeat; s_hb_t = now; s_hb_gap_max = 0;
  s_pn0 = g_probe_n; s_pok0 = g_probe_ok; s_pto0 = g_probe_timeout; s_perr0 = g_probe_err;
  g_probe_max_us = 0;
}
static void stick_stats_track(uint32_t now) {
  uint32_t hb = g_core1_heartbeat;
  if (hb != s_hb_last) { s_hb_last = hb; s_hb_t = now; }
  else {
    // signed: the hold starts in the console handler, after this loop pass read 'now'
    int32_t gap = (int32_t)(now - s_hb_t);
    if (gap > (int32_t)s_hb_gap_max) s_hb_gap_max = (uint32_t)gap;
  }
}
static void stick_stats_print(uint32_t now) {
  uint32_t d[8]; for (int i = 0; i < 8; ++i) d[i] = g_i2c_rc_hist[i] - s_rc0[i];
  Serial.printf("[i2cstick] held %lu ms | core-1 max heartbeat gap %lu ms (trip %d) | "
                "servo writes: ok %lu, nack %lu, timeout %lu, other %lu\n",
                (unsigned long)(now - s_hold_t0), (unsigned long)s_hb_gap_max, CORE1_STALL_TRIP_MS,
                (unsigned long)d[0], (unsigned long)(d[2] + d[3]), (unsigned long)d[5],
                (unsigned long)(d[1] + d[4] + d[6] + d[7]));
  uint32_t pn = g_probe_n - s_pn0;
  if (pn)
    Serial.printf("[i2cstick] probe: %lu writes (ok %lu, timeout %lu, error %lu), slowest %lu us\n",
                  (unsigned long)pn, (unsigned long)(g_probe_ok - s_pok0),
                  (unsigned long)(g_probe_timeout - s_pto0), (unsigned long)(g_probe_err - s_perr0),
                  (unsigned long)g_probe_max_us);
  if (d[0] + d[1] + d[2] + d[3] + d[4] + d[5] + d[6] + d[7] == 0 && pn == 0)
    Serial.println(F("[i2cstick] WARNING: no I2C traffic during the hold -- no PCA9685s present? use 'i2cstick probe on'"));
}

static void stick_assert(int pin) {
  gpio_set_oeover(pin, GPIO_OVERRIDE_HIGH);        // drive...
  gpio_set_outover(pin, GPIO_OVERRIDE_LOW);        // ...low
}
static void stick_release(int pin) {
  gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
  gpio_set_oeover(pin, GPIO_OVERRIDE_NORMAL);
}

// First thing in setup(), before core 1's bus_clear(): re-assert a persisted stick.
void bench_i2c_boot() {
  uint32_t s = watchdog_hw->scratch[0];
  if ((s & 0xFFFF0000u) != STICK_MAGIC) return;
  int pin  = (int)(s & 0xFF);
  int left = (int)((s >> 8) & 0xFF);
  if (left <= 0 || (pin != PIN_I2C_SDA && pin != PIN_I2C_SCL)) { watchdog_hw->scratch[0] = 0; return; }
  watchdog_hw->scratch[0] = STICK_MAGIC | ((uint32_t)(left - 1) << 8) | (uint32_t)pin;
  stick_assert(pin);
  s_stick_pin = pin; s_stick_until = 0; s_stick_boot_left = left - 1;
  stick_stats_begin(millis());
}

// Boot banner line (Serial is up by then).
void bench_i2c_report_boot() {
  if (s_stick_boot_left >= 0)
    Serial.printf("[i2cstick] PERSIST: %s held low from boot (%d more boot(s))\n",
                  s_stick_pin == PIN_I2C_SDA ? "SDA" : "SCL", s_stick_boot_left);
}

// Core 0 loop: timed release.
void bench_i2c_service(uint32_t now) {
  if (s_stick_pin < 0) return;
  stick_stats_track(now);
  if (s_stick_until && (int32_t)(now - s_stick_until) >= 0) {
    stick_release(s_stick_pin);
    Serial.printf("[i2cstick] released at %lu ms\n", (unsigned long)now);
    stick_stats_print(now);
    s_stick_pin = -1; s_stick_until = 0;
  }
}

// Console: tok[0] == "i2cstick".
void bench_i2c_command(char** tok, int n) {
  if (n >= 3 && !strcmp(tok[1], "probe")) {
    g_i2c_probe = !strcmp(tok[2], "on");
    g_probe_max_us = 0;
    Serial.printf("[i2cstick] probe %s: %d writes/tick to 0x%02X\n", g_i2c_probe ? "ON" : "off",
                  PROBE_PER_TICK, PROBE_ADDR);
    return;
  }
  if (n >= 2 && !strcmp(tok[1], "off")) {
    watchdog_hw->scratch[0] = 0;
    if (s_stick_pin >= 0) { stick_release(s_stick_pin); stick_stats_print(millis()); }
    s_stick_pin = -1; s_stick_until = 0; s_stick_boot_left = -1;
    Serial.println(F("[i2cstick] released; persist cleared"));
    return;
  }
  int pin = -1;
  if (n >= 2 && !strcasecmp(tok[1], "sda")) pin = PIN_I2C_SDA;
  if (n >= 2 && !strcasecmp(tok[1], "scl")) pin = PIN_I2C_SCL;
  if (pin < 0 || n < 3) {
    Serial.println(F("usage: i2cstick sda|scl <ms> | sda|scl hold | sda|scl persist N | off | probe on|off"));
    return;
  }
  if (s_stick_pin >= 0 && s_stick_pin != pin) stick_release(s_stick_pin);   // one line at a time
  uint32_t now = millis();
  if (!strcmp(tok[2], "hold")) {
    s_stick_until = 0;
    Serial.printf("[i2cstick] %s held low at %lu ms until 'i2cstick off' or reset\n", tok[1], (unsigned long)now);
  } else if (!strcmp(tok[2], "persist") && n >= 4) {
    int k = atoi(tok[3]);
    if (k < 1 || k > 20) { Serial.println(F("[i2cstick] persist N must be 1..20")); return; }
    watchdog_hw->scratch[0] = STICK_MAGIC | ((uint32_t)k << 8) | (uint32_t)pin;
    s_stick_until = 0;
    Serial.printf("[i2cstick] %s held low at %lu ms; re-asserted for the next %d boot(s). Power-cycle to abort.\n",
                  tok[1], (unsigned long)now, k);
  } else {
    long ms = atol(tok[2]);
    if (ms < 1 || ms > 60000) { Serial.println(F("[i2cstick] ms must be 1..60000")); return; }
    s_stick_until = now + (uint32_t)ms;
    if (s_stick_until == 0) s_stick_until = 1;
    Serial.printf("[i2cstick] %s held low at %lu ms for %ld ms\n", tok[1], (unsigned long)now, ms);
  }
  Serial.flush();
  stick_stats_begin(now);
  stick_assert(pin);
  s_stick_pin = pin;
}

#else   // hooks compiled out
void bench_i2c_probe_tick() {}
void bench_i2c_boot() {}
void bench_i2c_report_boot() {}
void bench_i2c_service(uint32_t) {}
void bench_i2c_command(char**, int) {
  Serial.println(F("[i2cstick] needs BENCH_HOOKS=1 and USE_REAL_I2C=1"));
}
#endif
