// =============================================================================
//  framing.ino  --  Pi link over USB CDC  (core 0)
//
//  Wire format:  [magic:4 LE][type:1][len:1][payload:len][crc8:1]
//  crc8 (poly 0x07) is taken over type|len|payload. The reader is a magic-resync
//  state machine: it never blocks and re-locks on the 4-byte magic after any
//  garble (same idiom as the teensyshot host link).
//
//  USB is shared with the human console. To keep both clean, the port is
//  MODE-AWARE: in text mode (default) incoming bytes go to the console line
//  reader; in 'tlm on' (binary) mode they go to this frame parser and telemetry
//  frames stream out. A production build could interleave both on one stream;
//  the alpha separates them for a legible bench console.
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
  Serial.write(hdr, 6);
  if (len) Serial.write(payload, len);
  Serial.write(&cc, 1);
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
  while (Serial.available()) {
    uint8_t c = (uint8_t)Serial.read();
    if (g_binary_tlm) parse_byte(c);       // binary mode: feed the frame parser
    else              console_feed_char(c); // text mode: feed the console line reader
  }
}

// ---- telemetry out (25 Hz each, alternating) ----
void tlm_service(uint32_t now) {
  if (!g_binary_tlm) return;
  static uint32_t last = 0;
  static uint8_t  which = 0;
  if ((now - last) < (uint32_t)(1000 / (PI_TLM_EACH_HZ * 2))) return;   // ~20 ms -> 50/s total
  last = now;

  SensorFrame fr; bool got = g_sensor_pub.snapshot(fr);
  ValveCmd vc;    g_valve_pub.snapshot(vc);

  uint8_t pl[128]; int i;
  if (which == 0) {
    i = 0;
    fput_u8 (pl, i, (uint8_t)g_status.source);
    fput_u8 (pl, i, g_status.armed ? 1 : 0);
    fput_u8 (pl, i, (uint8_t)g_status.comp_mode);
    fput_u8 (pl, i, g_status.flow_fallback ? 1 : 0);
    fput_u16(pl, i, g_status.rpm_target);
    fput_u8 (pl, i, g_status.n_valid);
    for (int v = 0; v < VALVE_COUNT; ++v) fput_i16(pl, i, vc.valve[v]);
    for (int s = 0; s < SERVO_COUNT; ++s) fput_u16(pl, i, got ? fr.servo_us[s] : 0);
    send_frame(FT_CTRL_TLM, pl, (uint8_t)i);
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
}
