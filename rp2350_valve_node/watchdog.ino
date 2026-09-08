// =============================================================================
//  watchdog.ino  --  Dual-core liveness watchdog + RGB status  (core 0)
//
//  The hardware watchdog is petted by core 0 ONLY after it confirms BOTH cores
//  are alive: its own loop is obviously running, and core 1's heartbeat counter
//  has advanced within CORE1_STALL_TRIP_MS. A hung core 1 therefore resets the
//  chip even though core 0 is fine -- the failure mode a single-core pet misses.
//  An early-boot grace window pets unconditionally so startup ordering can't
//  trip a reset before core 1 is ticking.
//
//  RGB (active LOW): PRIMARY=green, SBUS=blue, SAFE=red. Disarmed -> slow blink.
//  Fallback -> red mixed in (green->yellow, blue->magenta). Sensor-stale -> fast
//  blink. LED is status only; it never gates control.
// -----------------------------------------------------------------------------
#include "config.h"

static void led_set(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? LOW : HIGH);   // active low
  digitalWrite(PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_LED_B, b ? LOW : HIGH);
}

void led_setup() {
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  led_set(false, false, false);
}

static void led_update(uint32_t now) {
  // base color by source
  bool r=false, g=false, b=false;
  switch (g_status.source) {
    case SRC_PRIMARY: g = true; break;
    case SRC_SBUS:    b = true; break;
    default:          r = true; break;      // SAFE
  }
  if (g_status.flow_fallback) r = true;      // mix red in to flag fallback

  // blink logic
  bool on = true;
  if (g_status.sensor_stale)      on = (now % 200) < 100;   // fast
  else if (!g_status.armed)       on = (now % 1000) < 500;  // slow (disarmed)

  if (!on) { led_set(false,false,false); return; }
  led_set(r, g, b);
}

void watchdog_setup() {
  rp2040.wdt_begin(WDT_TIMEOUT_MS);   // hardware watchdog
}

void watchdog_service(uint32_t now) {
  led_update(now);

  // ---- core 1 liveness ----
  static uint32_t last_hb = 0xFFFFFFFF;
  static uint32_t last_hb_change = 0;
  uint32_t hb = g_core1_heartbeat;
  if (hb != last_hb) { last_hb = hb; last_hb_change = now; }
  bool core1_ok = (now - last_hb_change) < CORE1_STALL_TRIP_MS;

  // ---- pet policy ----
  if (now < WDT_BOOT_GRACE_MS) {
    rp2040.wdt_reset();               // unconditional during early boot
  } else if (core1_ok) {
    rp2040.wdt_reset();               // both cores healthy -> pet
  }
  // else: withhold the pet -> hardware reset within WDT_TIMEOUT_MS
}
