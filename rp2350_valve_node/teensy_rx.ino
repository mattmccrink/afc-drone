// =============================================================================
//  teensy_rx.ino  --  Read ESCPID_comm telemetry replies from the Motor Teensy
//
//  The Teensy answers every Host_comm we send with a fixed 64-byte ESCPID_comm
//  frame (magic 0x43305735 + struct-of-arrays, NO CRC / type / len). That is a
//  DIFFERENT wire format from the Pi link's framed protocol in framing.ino --
//  do not route it through parse_byte(); it has its own magic-resync reader here.
//
//  Runs on CORE 0, which owns the Teensy UART (see architecture section 3). Call
//  teensy_rx_service() every loop() tick. Latched values feed three consumers:
//    (a) console  -> teensy_tlm_print()
//    (b) mass-flow loop -> compressor_update() reads g_comp_rpm10 / g_teensy_ok
//    (c) Pi        -> tlm_service() appends them to CTRL_TLM (separate step)
//
//  NOTE: reads the SAME port teensy_stream() writes on. The committed tree uses
//  Serial2 (GP4/GP5); the bench build uses Serial1 (GP0/GP1). Keep TEENSY_PORT
//  below in sync with teensy_stream() and the wiring -- they MUST match.
// =============================================================================
#include "config.h"
#include "types.h"

#ifndef TEENSY_PORT
#define TEENSY_PORT          Serial1     // <-- MUST match teensy_stream()'s port
#endif
#define ESCPID_REPLY_LEN     64
#define TEENSY_TLM_STALE_MS  100         // no reply for this long -> not fresh

CompTlm g_comp;               // the one and only definition

static void teensy_decode(const uint8_t* f) {
  g_comp.err     = (int8_t)f[4];
  g_comp.temp_c  = f[10];
  g_comp.volt_cv = rx_u16(f, 28);
  g_comp.amp_ca  = rx_u16(f, 40);
  g_comp.rpm10   = rx_i16(f, 52);
  g_comp.tlm_ms  = millis();
  g_comp.ok      = true;
}

// ---- little-endian getters (mirror compressor.ino's put_* packers) ----------
static inline uint16_t rx_u16(const uint8_t* b, int i) {
  return (uint16_t)b[i] | ((uint16_t)b[i + 1] << 8);
}
static inline int16_t rx_i16(const uint8_t* b, int i) {
  return (int16_t)rx_u16(b, i);
}

// Drain the Teensy UART, resync on the 4-byte magic, decode complete 64-B frames.
// Non-blocking; leftover bytes persist across calls in the static buffer.
void teensy_rx_service(uint32_t now) {
  static const uint8_t MG[4] = { 0x35, 0x57, 0x30, 0x43 };  // 0x43305735, little-endian
  static uint8_t buf[ESCPID_REPLY_LEN];
  static int     n = 0;

  while (TEENSY_PORT.available()) {
    uint8_t c = TEENSY_PORT.read();
    if (n < 4) {                       // still matching/aligning the magic
      if (c == MG[n]) {
        buf[n++] = c;
      } else {                         // mismatch: does this byte start a new magic?
        n = (c == MG[0]) ? 1 : 0;
        if (n) buf[0] = c;
      }
    } else {                           // body
      buf[n++] = c;
      if (n >= ESCPID_REPLY_LEN) { teensy_decode(buf); n = 0; }
    }
  }

  if (g_comp.ok && (now - g_comp.tlm_ms) > TEENSY_TLM_STALE_MS)
    g_comp.ok = false;               // link/telemetry dropped
}

// Gated human console line (~3 Hz). Call from console_service() or loop().
void teensy_tlm_print(uint32_t now) {
  static uint32_t last = 0;
  if (now - last < 300) return;
  last = now;
  if (g_comp.ok)
    Serial.printf("[comp] %.2f V  %.2f A  %d rpm  %d C  err=%d\n",
                  g_comp.volt_cv * 0.01f, g_comp.amp_ca * 0.01f,
                  (int)g_comp.rpm10 * 10, (int)g_comp.temp_c, (int)g_comp.err);
  else
    Serial.println("[comp] telemetry STALE (no Teensy reply)");
}
