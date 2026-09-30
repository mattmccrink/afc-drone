// =============================================================================
//  i2c_guard.ino  --  I2C bus-fault breaker (core 1; all Wire traffic is on core 1)
//
//  A line held low for good (a shorted harness, a dead device) makes EVERY
//  transaction wait out Wire's 25 ms timeout. A tick issues dozens of them, so
//  core 1 stalled for the whole fault, missed its 120 ms heartbeat, and the
//  watchdog reset the node -- over and over while the fault lasted, stopping
//  the compressor each time (bench 2026-09-30, 'i2cstick ... hard').
//
//  The breaker: the first timeout in a tick opens it. The rest of that tick
//  does no I2C, and no I2C runs for I2C_BACKOFF_MS; then one tick tries again
//  (Wire's own timeout recovery has already run a 9-clock bus clear). Worst
//  case per tick is one 25 ms timeout, so core 1 keeps beating. While the bus
//  is dead the sensors go STALE -> venturis invalid -> air severity rises
//  (CAUTION or worse): sensing loss is ALERTED, not turned into a reboot.
//  The PCA9685s keep their last pulse on their own while no writes arrive.
//
//  Armed at the end of setup1(): boot-time PROM reads must never skip a mux
//  select (a PROM read through the wrong mux channel would pass its CRC).
// =============================================================================
#include "config.h"

#if USE_REAL_I2C
#include <Wire.h>

volatile uint32_t g_i2c_trips   = 0;    // breaker openings since boot
volatile uint32_t g_i2c_trip_ms = 0;    // millis() of the latest opening (0 = never)
static bool       s_armed       = false;
static bool       s_tick_open   = false;  // opened during this tick
static uint32_t   s_hold_until  = 0;      // no I2C before this millis()

void i2c_guard_arm() {                  // end of setup1()
  Wire.clearTimeoutFlag();              // boot-time timeouts don't count
  s_armed = true;
}

void i2c_guard_tick_begin() { s_tick_open = false; }

// May core 1 issue another I2C transaction now? Call BEFORE each one (or each
// short burst that must not be split, e.g. a mux select + its read).
bool i2c_ok() {
  if (!s_armed) return true;
  if (s_tick_open) return false;
  if (Wire.getTimeoutFlag()) {          // the previous transaction waited out the timeout
    Wire.clearTimeoutFlag();
    uint32_t now = millis();
    s_tick_open  = true;
    s_hold_until = now + I2C_BACKOFF_MS;
    g_i2c_trips++;
    g_i2c_trip_ms = now ? now : 1;
    return false;
  }
  return (int32_t)(millis() - s_hold_until) >= 0;
}

void i2c_guard_print() {
  uint32_t t = g_i2c_trip_ms;
  if (!g_i2c_trips) Serial.println(F("[health] I2C breaker: never opened"));
  else Serial.printf("[health] I2C breaker: opened %lu time(s), last %lu ms ago\n",
                     (unsigned long)g_i2c_trips, (unsigned long)(millis() - t));
}

#else
void i2c_guard_arm() {}
void i2c_guard_tick_begin() {}
bool i2c_ok() { return true; }
void i2c_guard_print() { Serial.println(F("[health] I2C breaker: n/a (simulated I2C)")); }
#endif
