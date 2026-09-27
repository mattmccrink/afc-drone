// Host mock of ESCCMD: one ESC whose telemetry the test drives directly.
#pragma once
#include <stdint.h>
#define ESCCMD_TIMER_PERIOD     2000
#define ESCCMD_MAX_ESC          6
#define ESCCMD_TIC_OCCURED      1
#define DSHOT_CMD_MOTOR_STOP    0
#define DSHOT_CMD_MAX           47
#define ESCCMD_ERROR_TLM_LOST   -10
#define ESCCMD_ERROR_DSHOT      -1
#define ESCCMD_ERROR_TLM_CRC    -5
#define ESCCMD_ERROR_TLM_TEMP   -8
struct MockEsc {
  uint32_t rx = 0;        // good packets received (test increments)
  bool     valid = false; // ESCCMD_tlm_valid of the LAST packet
  int16_t  rpm10 = 0;     // last packet rpm, 10 rpm units
  int8_t   err = 0;
  uint16_t cmd = DSHOT_CMD_MOTOR_STOP;
  uint16_t thwd = 20;     // throttle watchdog (ticks since last throttle)
  bool     emulated = false;
  bool     tlm = false;   // ESCCMD_tlm: telemetry requested (set on start, cleared at MOTOR_STOP)
  int8_t   inject_err = 0;// test hook: error latched on the next tic (e.g. -1 DShot)
};
extern MockEsc g_esc;
inline void    ESCCMD_init(uint8_t) {}
inline int     ESCCMD_arm_all() { return 0; }
inline int     ESCCMD_start_timer() { return 0; }
inline int     ESCCMD_stop(uint8_t) { g_esc.cmd = DSHOT_CMD_MOTOR_STOP; return 0; }
inline int     ESCCMD_tic() {                // one 2 ms tick per loop() call
  if (++g_esc.thwd >= 20 && g_esc.cmd != DSHOT_CMD_MOTOR_STOP) {
    g_esc.cmd = DSHOT_CMD_MOTOR_STOP; g_esc.valid = false; g_esc.err = 0;   // AFC MOTOR_STOP behaviour
    g_esc.tlm = false;
  }
  if (g_esc.inject_err) { g_esc.err = g_esc.inject_err; g_esc.inject_err = 0; }
  return ESCCMD_TIC_OCCURED;
}
inline int     ESCCMD_throttle(uint8_t, int16_t t) {
  if (g_esc.thwd >= 20) g_esc.err = 0;       // restart clears last_error
  if (!g_esc.tlm) g_esc.tlm = true;
  g_esc.cmd = (uint16_t)(DSHOT_CMD_MAX + 1 + t); g_esc.thwd = 0; return 0;
}
inline int     ESCCMD_read_err(uint8_t, int8_t* e) { *e = g_esc.err; g_esc.err = 0; return 0; }
inline int     ESCCMD_read_cmd(uint8_t, uint16_t* c) { *c = g_esc.cmd; return 0; }
inline int     ESCCMD_read_tlm_status(uint8_t) { return (g_esc.valid && g_esc.tlm) ? 0 : -6; }   // as ESCCMD
inline int     ESCCMD_read_deg(uint8_t, uint8_t* d) { *d = 25; return g_esc.valid ? 0 : -6; }
inline int     ESCCMD_read_volt(uint8_t, uint16_t* v) { *v = 1200; return g_esc.valid ? 0 : -6; }
inline int     ESCCMD_read_amp(uint8_t, uint16_t* a) { *a = 100; return g_esc.valid ? 0 : -6; }
inline int     ESCCMD_read_rpm(uint8_t, int16_t* r) { if (!g_esc.valid) return -6; *r = g_esc.rpm10; return 0; }
inline uint32_t ESCCMD_read_tlm_rx_cnt(uint8_t) { return g_esc.rx; }
inline bool    ESCCMD_is_emulated() { return g_esc.emulated; }
