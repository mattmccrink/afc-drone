// =============================================================================
//  console.ino  --  Human bench console over USB CDC  (core 0)
//
//  Drives the whole node on a bare board: force sources, arm/disarm, simulate
//  primary loss and SBUS failsafe, set sticks and mass-flow target, inject
//  sensor faults, and save calibration. Line-based; type 'help'.
// -----------------------------------------------------------------------------
#include "config.h"

static bool    con_mon_on = false;
static bool    con_pstream = false;   // 2 Hz LabVIEW pressure stream
// servo bring-up sweep state (core 0 computes waveform, core 1 writes)
static bool    sweep_active = false;
static uint16_t sweep_min = 1000, sweep_max = 2000;
static uint32_t sweep_ms = 2000, sweep_t0 = 0;
static char    con_line[96];
static uint8_t con_len = 0;

static const char* src_name(Source s) {
  switch (s) { case SRC_PRIMARY: return "PRIMARY"; case SRC_SBUS: return "SBUS"; default: return "SAFE"; }
}
static const char* mode_name(CompMode m) {
  switch (m) { case COMP_TRACK: return "TRACK"; case COMP_DIRECT: return "DIRECT";
               case COMP_FALLBACK: return "FALLBACK"; default: return "STOPPED"; }
}

static void print_status() {
  Serial.printf(
    "[st] src=%-7s arm=%d(live=%d) prim=%d | comp=%-8s rpm=%5u | mdot=%.1f/%.1f g/s nvalid=%u%s%s | cal=%s | hb=%lu\n",
    src_name(g_status.source), g_status.armed?1:0, g_status.arm_live?1:0, g_status.primary_present?1:0,
    mode_name(g_status.comp_mode), g_status.rpm_target,
    (double)g_status.mdot_total, (double)g_status.mdot_target, g_status.n_valid,
    g_status.flow_fallback ? " FALLBACK" : "", g_status.sensor_stale ? " STALE" : "",
    g_cal_from_flash ? "flash" : "default", (unsigned long)g_core1_heartbeat);
}

static void print_help() {
  Serial.println(F(
    "commands:\n"
    "  help                    this list\n"
    "  status                  one-shot status line\n"
    "  health                  reprint boot/health summary (sensor PROM, cal, liveness)\n"
    "  mon on|off              periodic status line (2 Hz)\n"
    "  tlm on|off              binary telemetry mode (RX+TX switch to frames)\n"
    "  src auto|primary|sbus|safe   force/release the active source\n"
    "  arm | disarm            simulated Pixhawk arm channel\n"
    "  prim on|off             simulate primary (Pi) presence\n"
    "  sbusfs on|off           simulate SBUS failsafe flag\n"
    "  sbuslost on|off         simulate SBUS frame-lost flag\n"
    "  stick R P Y T           set SBUS sticks (floats: roll pitch yaw throttle)\n"
    "  fault valve N on|off    inject per-valve sensor fault (N=0..5)\n"
    "  fault agg on|off        inject aggregate flow fault\n"
    "  mdot X                  set mass-flow target (g/s)\n"
    "  sens                    dump per-valve p_up/p_lo/dp/T/mdot from the frame\n"
    "  pstream on|off          2 Hz LabVIEW stream: !A=12 pressures, !B=12 temps\n"
    "  servo <A|B|C> <ch> <us> drive one PCA9685 channel (bring-up); servo off to stop\n"
    "  sweep <A|B|C> <ch> [min max ms]  triangle-sweep one channel (default 1000 2000 2000)\n"
    "  save                    persist current calibration to LittleFS\n"
    "  zero                    capture no-flow dp offset per valve, then save\n"
    "  calshow                 show calibration source + servo0 coeffs + dp_zero"));
}

static bool eq(const char* a, const char* b) { return strcmp(a, b) == 0; }

// Resolve a servo-driver name to a device index: A/B/C (any case) or 0/1/2.
static int dev_index(const char* s) {
  if (!strcasecmp(s, "A") || eq(s, "0")) return 0;
  if (!strcasecmp(s, "B") || eq(s, "1")) return 1;
  if (!strcasecmp(s, "C") || eq(s, "2")) return 2;
  return -1;
}
static bool onoff(const char* s, bool& out) {
  if (eq(s,"on"))  { out = true;  return true; }
  if (eq(s,"off")) { out = false; return true; }
  return false;
}

static void dispatch(char* line) {
  // tokenize on spaces
  char* tok[6]; int n = 0;
  char* p = strtok(line, " \t");
  while (p && n < 6) { tok[n++] = p; p = strtok(nullptr, " \t"); }
  if (n == 0) return;

  if (eq(tok[0], "help"))        { print_help(); }
  else if (eq(tok[0], "status")) { print_status(); }
  else if (eq(tok[0], "health")) {
    Serial.println(F("RP2350 valve/sensor node -- health"));
    Serial.printf("[health] core1 heartbeat=%lu (should advance between calls)\n",
                  (unsigned long)g_core1_heartbeat);
    Serial.printf("[health] cal source=%s\n", g_cal_from_flash ? "flash" : "default");
    sensors_health_print();
    servos_health_print();
  }
  else if (eq(tok[0], "mon") && n >= 2)  { onoff(tok[1], con_mon_on); }
  else if (eq(tok[0], "pstream") && n >= 2) {
    bool b; if (onoff(tok[1], b)) {
      con_pstream = b;
      if (b) {                              // one-time column headers for LabVIEW
        Serial.print("!HA");
        for (int v = 0; v < VALVE_COUNT; ++v) Serial.printf(",v%d_up,v%d_lo", v, v);
        Serial.println();
        Serial.print("!HB");
        for (int v = 0; v < VALVE_COUNT; ++v) Serial.printf(",v%d_up,v%d_lo", v, v);
        Serial.println();
      }
    }
  }
  else if (eq(tok[0], "tlm") && n >= 2)  {
    bool b; if (onoff(tok[1], b)) { g_binary_tlm = b;
      if (!b) Serial.println(F("[tlm] text mode")); }
  }
  else if (eq(tok[0], "src") && n >= 2) {
    if      (eq(tok[1],"auto"))    arbitration_force(SRC_SAFE, false);
    else if (eq(tok[1],"primary")) arbitration_force(SRC_PRIMARY, true);
    else if (eq(tok[1],"sbus"))    arbitration_force(SRC_SBUS, true);
    else if (eq(tok[1],"safe"))    arbitration_force(SRC_SAFE, true);
    Serial.printf("[src] %s\n", tok[1]);
  }
  else if (eq(tok[0], "arm"))    { g_sim_arm_intent = true;  Serial.println(F("[arm] intent=ARMED")); }
  else if (eq(tok[0], "disarm")) { g_sim_arm_intent = false; Serial.println(F("[arm] intent=DISARMED")); }
  else if (eq(tok[0], "prim") && n >= 2) {
    bool b; if (onoff(tok[1], b)) { g_status.primary_present = b; Serial.printf("[prim] %d\n", b?1:0); }
  }
  else if (eq(tok[0], "sbusfs") && n >= 2)   { bool b; if (onoff(tok[1], b)) g_sbus_failsafe  = b; }
  else if (eq(tok[0], "sbuslost") && n >= 2) { bool b; if (onoff(tok[1], b)) g_sbus_framelost = b; }
  else if (eq(tok[0], "stick") && n >= 5) {
    g_sbus_in.roll     = atof(tok[1]);
    g_sbus_in.pitch    = atof(tok[2]);
    g_sbus_in.yaw      = atof(tok[3]);
    g_sbus_in.throttle = atof(tok[4]);
    Serial.printf("[stick] r=%.2f p=%.2f y=%.2f t=%.2f\n",
      (double)g_sbus_in.roll,(double)g_sbus_in.pitch,(double)g_sbus_in.yaw,(double)g_sbus_in.throttle);
  }
  else if (eq(tok[0], "fault") && n >= 3) {
    if (eq(tok[1], "agg")) { bool b; if (onoff(tok[2], b)) { g_fault_aggregate = b; Serial.printf("[fault] agg=%d\n", b?1:0);} }
    else if (eq(tok[1], "valve") && n >= 4) {
      int idx = atoi(tok[2]); bool b;
      if (idx >= 0 && idx < VALVE_COUNT && onoff(tok[3], b)) { g_fault_valve[idx] = b; Serial.printf("[fault] valve%d=%d\n", idx, b?1:0);}
    }
  }
  else if (eq(tok[0], "servo")) {
    if (n >= 2 && eq(tok[1], "off")) {
      g_servo_manual = false; sweep_active = false;
      Serial.println(F("[servo] manual OFF (back to curve-fit path)"));
    } else if (n >= 4) {
      int d = dev_index(tok[1]); int ch = atoi(tok[2]); int us = atoi(tok[3]);
      if (d < 0)                              Serial.println(F("[servo] bad device (A/B/C)"));
      else if (ch < 0 || ch >= PCA9685_MAX_CH) Serial.printf("[servo] bad ch (0..%d)\n", PCA9685_MAX_CH-1);
      else if (us < SERVO_US_MIN || us > SERVO_US_MAX) Serial.printf("[servo] bad us (%d..%d)\n", SERVO_US_MIN, SERVO_US_MAX);
      else {
        sweep_active = false;
        g_servo_man_dev = d; g_servo_man_ch = ch; g_servo_man_us = us; g_servo_manual = true;
        Serial.printf("[servo] manual dev%c ch%d = %d us\n", 'A'+d, ch, us);
      }
    } else Serial.println(F("[servo] usage: servo <A|B|C> <ch> <us> | servo off"));
  }
  else if (eq(tok[0], "sweep") && n >= 3) {
    int d = dev_index(tok[1]); int ch = atoi(tok[2]);
    uint16_t mn = (n >= 4) ? atoi(tok[3]) : 1000;
    uint16_t mx = (n >= 5) ? atoi(tok[4]) : 2000;
    uint32_t ms = (n >= 6) ? (uint32_t)atoi(tok[5]) : 2000;
    if (d < 0)                               Serial.println(F("[sweep] bad device (A/B/C)"));
    else if (ch < 0 || ch >= PCA9685_MAX_CH) Serial.printf("[sweep] bad ch (0..%d)\n", PCA9685_MAX_CH-1);
    else if (mn < SERVO_US_MIN || mx > SERVO_US_MAX || mn >= mx || ms < 100)
                                             Serial.println(F("[sweep] bad range/period"));
    else {
      sweep_min = mn; sweep_max = mx; sweep_ms = ms; sweep_t0 = millis();
      g_servo_man_dev = d; g_servo_man_ch = ch; g_servo_manual = true; sweep_active = true;
      Serial.printf("[sweep] dev%c ch%d %u..%u us over %lu ms (servo off to stop)\n",
                    'A'+d, ch, mn, mx, (unsigned long)ms);
    }
  }
  else if (eq(tok[0], "mdot") && n >= 2) { g_status.mdot_target = atof(tok[1]); Serial.printf("[mdot] target=%.1f\n",(double)g_status.mdot_target); }
  else if (eq(tok[0], "sens")) {
    SensorFrame fr;
    if (!g_sensor_pub.snapshot(fr)) { Serial.println(F("[sens] no frame")); return; }
    for (int v = 0; v < VALVE_COUNT; ++v) {
      //if (fr.p_up[v] <= 0.0f && fr.p_lo[v] <= 0.0f) continue;   // skip unpopulated valves
      float dpraw  = fr.p_up[v] - fr.p_lo[v];
      float dpcorr = dpraw - g_dp_zero[v];
      Serial.printf("[sens] v%d pu=%.2f pl=%.2f dpraw=%.2f dpz=%.2f dpcorr=%.2f mbar t=%.2f C mdot=%.3f g/s %s\n",
        v, (double)fr.p_up[v], (double)fr.p_lo[v], (double)dpraw,
        (double)g_dp_zero[v], (double)dpcorr, (double)fr.t_die[v],
        (double)fr.mdot[v], fr.valid[v] ? "OK" : "--");
    }
    Serial.printf("[sens] total=%.3f g/s  nvalid=%u\n", (double)fr.mdot_total, fr.n_valid); 
  }           
  else if (eq(tok[0], "save")) { Serial.println(cal_save() ? F("[cal] saved") : F("[cal] SAVE FAILED")); }
  else if (eq(tok[0], "zero")) {
    // Capture the current no-flow (p_up - p_lo) per valve as the offset. MUST be
    // done at genuine no-flow (compressor stopped, still air).
    SensorFrame fr;
    if (!g_sensor_pub.snapshot(fr)) { Serial.println(F("[zero] no sensor frame")); return; }
    int n = 0;
    for (int v = 0; v < VALVE_COUNT; ++v) {
      if (fr.p_up[v] > 0.0f && fr.p_lo[v] > 0.0f) {   // a paired valve is present
        g_dp_zero[v] = fr.p_up[v] - fr.p_lo[v];
        n++;
      }
    }
    bool ok = cal_save();
    Serial.printf("[zero] captured %d valve(s); save %s\n", n, ok ? "OK" : "FAILED");
  }
  else if (eq(tok[0], "calshow")) {
    Serial.printf("[cal] source=%s servo0 c=[%.1f %.1f %.1f %.1f] map0=%u gain0=%.2f\n",
      g_cal_from_flash?"flash":"default",
      (double)g_servo_cubic[0][0],(double)g_servo_cubic[0][1],(double)g_servo_cubic[0][2],(double)g_servo_cubic[0][3],
      g_servo_valve_map[0], (double)g_valve_gain[0]);
    Serial.print("[cal] dp_zero =");
    for (int v = 0; v < VALVE_COUNT; ++v) Serial.printf(" %.2f", (double)g_dp_zero[v]);
    Serial.println(" mbar");
  }
  else { Serial.printf("? %s (try 'help')\n", tok[0]); }
}

// Fed one byte at a time by framing_pump() in text mode.
void console_feed_char(uint8_t c) {
  if (c == '\r') return;
  if (c == '\n') { con_line[con_len] = 0; if (con_len) dispatch(con_line); con_len = 0; return; }
  if (con_len < sizeof(con_line) - 1) con_line[con_len++] = (char)c;
}

void console_service(uint32_t now) {
  // servo sweep waveform: triangle min->max->min over sweep_ms; core 1 writes it
  if (sweep_active && g_servo_manual) {
    uint32_t p = (now - sweep_t0) % sweep_ms;
    float ph = (float)p / (float)sweep_ms;                 // 0..1
    float frac = (ph < 0.5f) ? (2.0f * ph) : (2.0f - 2.0f * ph);   // triangle 0..1..0
    g_servo_man_us = sweep_min + (uint16_t)lroundf(frac * (sweep_max - sweep_min));
  }

  // 2 Hz calibration stream for LabVIEW (text mode only): all 12 pressures (!A)
  // and all 12 temps (!B), fixed width -- unpopulated valves read 0 until wired.
  if (con_pstream && !g_binary_tlm) {
    static uint32_t last_ps = 0;
    if ((now - last_ps) >= 500) {
      last_ps = now;
      SensorFrame fr;
      if (g_sensor_pub.snapshot(fr)) {
        Serial.print("!A");
        for (int v = 0; v < VALVE_COUNT; ++v)
          Serial.printf(",%.3f,%.3f", (double)fr.p_up[v], (double)fr.p_lo[v]);
        Serial.println();
        Serial.print("!B");
        for (int v = 0; v < VALVE_COUNT; ++v)
          Serial.printf(",%.3f,%.3f", (double)fr.t_die[v], (double)fr.t_lo[v]);
        Serial.println();
      }
    }
  }

  if (!con_mon_on || g_binary_tlm) return;
  static uint32_t last = 0;
  if ((now - last) < 500) return;
  last = now;
  print_status();
}
