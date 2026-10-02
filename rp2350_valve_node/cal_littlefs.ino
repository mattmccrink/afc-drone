// =============================================================================
//  cal_littlefs.ino  --  Curve-fit calibration store  (core 0, at boot)
//
//  Order of precedence: compiled defaults are applied FIRST (always valid), then
//  a CRC32-checked blob from LittleFS overrides them if present and intact. A
//  missing or corrupt store silently leaves the defaults in place -- the node is
//  always flyable-safe even with a trashed filesystem.
//
//  Scope note: this store holds curve-fit coefficients only. RPM_FALLBACK (30k)
//  is a compiled-in constant and is deliberately NOT overridable from flash.
//
//  Flash needs an FS partition: Tools -> Flash Size -> a "... with FS" option
//  (e.g. 4MB, 1MB FS) or LittleFS.begin() returns false and defaults are used.
// -----------------------------------------------------------------------------
#include "config.h"
#include "types.h"
#include <LittleFS.h>

#define CAL_PATH    "/cal.bin"
#define CAL_MAGIC   0x43414C33u   // "CAL3": servo curves moved to compiled servo_cal.h
#define CAL_MAGIC_V2 0x43414C32u  // "CAL2": had per-servo cubics; migrated on load

// (CalBlob is defined in types.h so the auto-generated prototypes can see it.)

static uint32_t crc32_calc(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (-(int32_t)(c & 1)));
  }
  return ~c;
}

// Compiled-in defaults: servo s driven by valve s/2, unity gain, zero bias, no
// zero offsets. (Servo curves are compiled in servo_cal.h, not stored here.)
void cal_apply_defaults() {
  for (int s = 0; s < SERVO_COUNT; ++s) {
    g_servo_valve_map[s] = (uint8_t)(s / 2);                          // 2 servos per valve
  }
  for (int v = 0; v < VALVE_COUNT; ++v) { g_valve_gain[v] = 1.0f; g_valve_bias[v] = 0.0f; g_dp_zero[v] = 0.0f; }
  g_cal_from_flash = false;
}

static void blob_from_ram(CalBlob& b) {
  b.magic = CAL_MAGIC;
  memcpy(b.servo_valve_map, g_servo_valve_map, sizeof(b.servo_valve_map));
  memcpy(b.valve_gain, g_valve_gain, sizeof(b.valve_gain));
  memcpy(b.valve_bias, g_valve_bias, sizeof(b.valve_bias));
  memcpy(b.dp_zero, g_dp_zero, sizeof(b.dp_zero));
  b.crc32 = crc32_calc((const uint8_t*)&b, sizeof(CalBlob) - sizeof(uint32_t));
}

static void ram_from_blob(const CalBlob& b) {
  memcpy(g_servo_valve_map, b.servo_valve_map, sizeof(g_servo_valve_map));
  memcpy(g_valve_gain, b.valve_gain, sizeof(g_valve_gain));
  memcpy(g_valve_bias, b.valve_bias, sizeof(g_valve_bias));
  memcpy(g_dp_zero, b.dp_zero, sizeof(g_dp_zero));
}

void cal_load() {
  cal_apply_defaults();                 // always start from valid, immutable defaults

  if (!LittleFS.begin()) {
    Serial.println(F("[cal] LittleFS unavailable -> using compiled defaults"));
    return;
  }
  File f = LittleFS.open(CAL_PATH, "r");
  if (!f) { Serial.println(F("[cal] no /cal.bin -> compiled defaults")); return; }

  size_t fsz = f.size();
  if (fsz == sizeof(CalBlobV2)) {
    // Old "CAL2" record (had per-servo cubics). Keep its map, gain/bias and zero
    // offsets; the cubics are ignored (curves are compiled now). Not rewritten
    // here -- a flash write stalls both cores -- so 'save' once to convert it.
    CalBlobV2 o;
    bool rd = f.read((uint8_t*)&o, sizeof(o)) == (int)sizeof(o);
    f.close();
    uint32_t want2 = crc32_calc((const uint8_t*)&o, sizeof(CalBlobV2) - sizeof(uint32_t));
    if (!rd || o.magic != CAL_MAGIC_V2 || o.crc32 != want2) {
      Serial.println(F("[cal] old-format /cal.bin unreadable -> compiled defaults"));
      return;
    }
    memcpy(g_servo_valve_map, o.servo_valve_map, sizeof(g_servo_valve_map));
    memcpy(g_valve_gain, o.valve_gain, sizeof(g_valve_gain));
    memcpy(g_valve_bias, o.valve_bias, sizeof(g_valve_bias));
    memcpy(g_dp_zero, o.dp_zero, sizeof(g_dp_zero));
    g_cal_from_flash = true;
    Serial.println(F("[cal] migrated CAL2 /cal.bin (map, gain/bias, zero kept; old cubics ignored) -- 'save' to rewrite as CAL3"));
    return;
  }
  CalBlob b;
  if (f.read((uint8_t*)&b, sizeof(b)) != (int)sizeof(b)) {
    f.close(); Serial.println(F("[cal] short read -> compiled defaults")); return;
  }
  f.close();
  uint32_t want = crc32_calc((const uint8_t*)&b, sizeof(CalBlob) - sizeof(uint32_t));
  if (b.magic != CAL_MAGIC || b.crc32 != want) {
    Serial.println(F("[cal] bad magic/CRC -> compiled defaults"));
    return;
  }
  ram_from_blob(b);
  g_cal_from_flash = true;
  Serial.println(F("[cal] loaded /cal.bin (CRC ok)"));
}

// Console 'save': persist the current in-RAM coefficients.
bool cal_save() {
  if (!LittleFS.begin()) {
    if (!LittleFS.format() || !LittleFS.begin()) return false;
  }
  CalBlob b; blob_from_ram(b);
  File f = LittleFS.open(CAL_PATH, "w");
  if (!f) return false;
  size_t n = f.write((const uint8_t*)&b, sizeof(b));
  f.close();
  if (n == sizeof(b)) { g_cal_from_flash = true; return true; }
  return false;
}
