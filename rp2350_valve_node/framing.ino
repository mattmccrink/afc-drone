// =============================================================================
//  framing.ino  --  Pi link over Serial2 (UART1, GP4/GP5, 230400)  (core 0)
//
//  Wire format:  [magic:4 LE][type:1][len:1][payload:len][crc8:1]
//  crc8 (poly 0x07) is taken over type|len|payload. The reader is a magic-resync
//  state machine: it never blocks and re-locks on the 4-byte magic after any
//  garble (same idiom as the teensyshot host link).
//
//  Serial2 is ALWAYS framed (Pi link); USB CDC is console-only text. There is no
//  mode switch any more (the old 'tlm on' handshake / g_binary_tlm are gone).
//
//  Telemetry payloads (all APPENDED fields keep earlier offsets unchanged):
//    CTRL_TLM   0x81  55 B = 43 base + 4 fault-tree ext + 8 surfaces (4x i16)
//                     flags: bit0 terminated, bit1 FC arm eligible (token live),
//                            bit2 SBUS arm eligible, bit3 SBUS arm sw,
//                            bit4 surfaces active (allocated), bit5 surface switch on
//    SENSOR_TLM 0x82  56 B
//    COMP_TLM   0x83  14 B = 13 base + flags (bit0 ESC thermal-derate suspect)
// -----------------------------------------------------------------------------
#include "config.h"
#include "types.h"

// ---- CRC8 (poly 0x07, init 0x00) ----
static uint8_t crc8(const uint8_t* p, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// ---- little-endian packers (unique names: shared TU with compressor.ino) ----
static inline void fput_u8 (uint8_t* b, int& i, uint8_t v){ b[i++]=v; }
static inline void fput_u16(uint8_t* b, int& i, uint16_t v){ b[i++]=v; b[i++]=v>>8; }
static inline void fput_i16(uint8_t* b, int& i, int16_t v){ fput_u16(b,i,(uint16_t)v); }
static inline void fput_u32(uint8_t* b, int& i, uint32_t v){ b[i++]=v; b[i++]=v>>8; b[i++]=v>>16; b[i++]=v>>24; }
static inline int16_t fget_i16(const uint8_t* p){ return (int16_t)(p[0] | (p[1]<<8)); }
static inline uint32_t fget_u32(const uint8_t* p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

// ---- outbound frame writer ----
static void send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t hdr[6];
  int i = 0;
  fput_u32(hdr, i, PI_MAGIC);
  hdr[i++] = type;
  hdr[i++] = len;
  uint8_t cc;
  { // crc over type|len|payload
    uint8_t tmp[2] = { type, len };
    uint8_t c = crc8(tmp, 2);
    // continue crc over payload
    for (uint8_t k = 0; k < len; ++k) { c ^= payload[k]; for (int b=0;b<8;++b) c = (c&0x80)?(uint8_t)((c<<1)^0x07):(uint8_t)(c<<1); }
    cc = c;
  }
  Serial2.write(hdr, 6);
  if (len) Serial2.write(payload, len);
  Serial2.write(&cc, 1);
}

// ---- inbound frame dispatch ----
static void on_frame(uint8_t type, const uint8_t* p, uint8_t len) {
  uint32_t now = millis();
  if (type == FT_CMD && len >= 12) {
    // roll,pitch,yaw,airspeed,alt,mdot_target : each int16 LE
    g_primary_in.roll        = fget_i16(p+0)  / 1000.0f;   // [-1,1]
    g_primary_in.pitch       = fget_i16(p+2)  / 1000.0f;
    g_primary_in.yaw         = fget_i16(p+4)  / 1000.0f;
    g_primary_in.throttle    = fget_i16(p+6)  / 1000.0f;   // reuse "airspeed" slot as throttle demo
    g_primary_in.mdot_target = fget_i16(p+10) / 10.0f;     // 0.1 g/s units
    g_primary_in.valid       = true;
    g_primary_in.stamp_ms    = now;
    g_status.mdot_target     = g_primary_in.mdot_target;
  } else if (type == FT_ARM && len >= 5) {
    g_arm_state   = p[0];
    g_arm_counter = fget_u32(p+1);
    g_arm_stamp   = now;
  }
}

// ---- magic-resync parser ----
static uint8_t  rx_state = 0;         // 0..3 magic, 4 type, 5 len, 6 payload, 7 crc
static uint8_t  rx_magic_pos = 0;
static uint8_t  rx_type, rx_len, rx_idx;
static uint8_t  rx_payload[256];
static const uint8_t PI_MAGIC_B[4] = {
  (uint8_t)(PI_MAGIC), (uint8_t)(PI_MAGIC>>8), (uint8_t)(PI_MAGIC>>16), (uint8_t)(PI_MAGIC>>24)
};

static void parse_byte(uint8_t c) {
  switch (rx_state) {
    case 0: case 1: case 2: case 3:
      if (c == PI_MAGIC_B[rx_state]) rx_state++;
      else rx_state = (c == PI_MAGIC_B[0]) ? 1 : 0;   // resync
      break;
    case 4: rx_type = c; rx_state = 5; break;
    case 5: rx_len = c; rx_idx = 0; rx_state = (rx_len ? 6 : 7); break;
    case 6:
      rx_payload[rx_idx++] = c;
      if (rx_idx >= rx_len) rx_state = 7;
      break;
    case 7: {
      // c == received crc
      uint8_t tmp[2] = { rx_type, rx_len };
      uint8_t cc = crc8(tmp, 2);
      for (uint8_t k = 0; k < rx_len; ++k) { cc ^= rx_payload[k]; for (int b=0;b<8;++b) cc = (cc&0x80)?(uint8_t)((cc<<1)^0x07):(uint8_t)(cc<<1); }
      if (cc == c) on_frame(rx_type, rx_payload, rx_len);
      rx_state = 0;
      break;
    }
  }
}

// ---- public API ----
void framing_setup() { rx_state = 0; }

void framing_pump() {
  // Pi link (binary frames) on Serial2 -- always framed
  while (Serial2.available()) parse_byte((uint8_t)Serial2.read());
  // Human console on USB -- always text
  while (Serial.available()) console_feed_char((uint8_t)Serial.read());
}

// ---- telemetry out (25 Hz each, alternating) ----
void tlm_service(uint32_t now) {
  static uint32_t last = 0;
  static uint8_t  which = 0;
  if ((now - last) < (uint32_t)(1000 / (PI_TLM_EACH_HZ * 2))) return;   // ~20 ms -> 50/s total
  last = now;

  SensorFrame fr; bool got = g_sensor_pub.snapshot(fr);
  if (!got) g_tlm_dropped++;              // contended cross-core SensorFrame read

  uint8_t pl[128]; int i;
  if (which == 0) {
    i = 0;
    fput_u8 (pl, i, (uint8_t)g_status.source);
    fput_u8 (pl, i, g_status.armed ? 1 : 0);
    fput_u8 (pl, i, (uint8_t)g_status.comp_mode);
    fput_u8 (pl, i, g_status.flow_fallback ? 1 : 0);
    fput_u16(pl, i, g_status.rpm_target);
    fput_u8 (pl, i, g_status.n_valid);
    for (int v = 0; v < VALVE_COUNT; ++v) fput_i16(pl, i, g_valve_dbg[v]);
    for (int s = 0; s < SERVO_COUNT; ++s) fput_u16(pl, i, got ? fr.servo_us[s] : 0);
    // ---- fault-tree extras (APPENDED; base offsets above unchanged) ----
    uint8_t mode = g_status.terminated            ? MODE_TERMINATED
                 : !g_status.armed                ? MODE_DISARMED
                 : g_status.source == SRC_PRIMARY ? MODE_PRIMARY_ARMED
                 : g_status.source == SRC_SBUS    ? MODE_SBUS_REVERSION
                 :                                  MODE_SAFE_HOLD;
    fput_u8(pl, i, mode);
    uint8_t flags = (g_status.terminated       ? 0x01 : 0)   // bit0 terminated
                  | (arb_fc_eligible()         ? 0x02 : 0)   // bit1 FC arm eligible (token live)
                  | (g_sbus_arm_seen_disarmed  ? 0x04 : 0)   // bit2 SBUS arm eligible
                  | (g_sbus_arm                ? 0x08 : 0)   // bit3 SBUS switch now
                  | (g_status.surf_active      ? 0x10 : 0)   // bit4 surfaces ACTIVE (allocated)
                  | (g_status.surf_engaged     ? 0x20 : 0);  // bit5 surface switch on
    fput_u8(pl, i, flags);
    float srp  = g_alloc_s_rp  < 0.f ? 0.f : (g_alloc_s_rp  > 1.f ? 1.f : g_alloc_s_rp);
    float syaw = g_alloc_s_yaw < 0.f ? 0.f : (g_alloc_s_yaw > 1.f ? 1.f : g_alloc_s_yaw);
    fput_u8(pl, i, (uint8_t)(srp  * 100.f + 0.5f));          // desat scales, %*100
    fput_u8(pl, i, (uint8_t)(syaw * 100.f + 0.5f));
    // ---- surfaces (APPENDED): commanded position, [-1000,1000], 0 = center ----
    for (int k = 0; k < SURF_COUNT; ++k) fput_i16(pl, i, g_surf_dbg[k]);
    send_frame(FT_CTRL_TLM, pl, (uint8_t)i);   // i == 55
  } else {
    i = 0;
    for (int v = 0; v < VALVE_COUNT; ++v) fput_i16(pl, i, (int16_t)lroundf((got?fr.p_up[v]:0)  * 10.0f));
    for (int v = 0; v < VALVE_COUNT; ++v) fput_i16(pl, i, (int16_t)lroundf((got?fr.p_lo[v]:0)  * 10.0f));
    for (int v = 0; v < VALVE_COUNT; ++v) fput_i16(pl, i, (int16_t)lroundf((got?fr.t_die[v]:0) * 100.0f));
    for (int v = 0; v < VALVE_COUNT; ++v) fput_i16(pl, i, (int16_t)lroundf((got?fr.mdot[v]:0)  * 10.0f));
    fput_i16(pl, i, (int16_t)lroundf((got?fr.mdot_total:0) * 10.0f));
    uint16_t mask = 0;
    for (int v = 0; v < VALVE_COUNT; ++v) if (got && fr.valid[v]) mask |= (1u << v);
    fput_u16(pl, i, mask);
    fput_u32(pl, i, now);
    send_frame(FT_SENSOR_TLM, pl, (uint8_t)i);
  }
  which ^= 1;
    // ---- COMP_TLM: measured compressor telemetry, own 25 Hz cadence ----
  static uint32_t last_comp = 0;
  if ((now - last_comp) >= (uint32_t)(1000 / PI_TLM_EACH_HZ)) {
    last_comp = now;
    uint8_t cp[16]; int j = 0;
    fput_u16(cp, j, g_comp.volt_cv);
    fput_u16(cp, j, g_comp.amp_ca);
    fput_i16(cp, j, g_comp.rpm10);
    fput_u8 (cp, j, g_comp.temp_c);
    fput_u8 (cp, j, (uint8_t)g_comp.err);   // i8 on wire, cast back on Pi
    fput_u8 (cp, j, g_comp.ok ? 1 : 0);
    fput_u32(cp, j, now);
    fput_u8 (cp, j, g_status.comp_thermal ? 0x01 : 0);   // APPENDED flags: bit0 thermal suspect
    send_frame(FT_COMP_TLM, cp, (uint8_t)j);   // j == 14
  }
}
