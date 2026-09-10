// =============================================================================
//  rp2350_valve_node.ino  --  Active Flow Control Valve/Sensor Node  (ALPHA)
//
//  Dual-core RP2350 (Pimoroni Tiny 2350) firmware skeleton. Real control logic;
//  simulated hardware leaves (sensors, servos, SBUS) so the whole thing runs and
//  is drivable on a bare board over USB + the onboard RGB LED.
//
//  Core 0 (setup/loop):   USB comms, source arbitration, arm enforcement,
//                         allocation, mass-flow->rpm outer loop, Teensy stream,
//                         watchdog + status LED.
//  Core 1 (setup1/loop1): fixed 100 Hz I2C service -- sensor pipeline, venturi
//                         mass-flow, curve-fit servo expansion, servo writes.
//
//  Cog mental map: core 0 = "comms/allocation cog", core 1 = "I2C service cog",
//  SBUS = its own "receiver cog" (PIO, stubbed in alpha). Shared hub RAM =
//  the two SpscPublisher seqlocks below.
//
//  See rp2350_valve_node_architecture.md for the design of record and README.md
//  for build/bench instructions.
// =============================================================================
#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "Wire.h"

// -----------------------------------------------------------------------------
//  Shared, cross-core state  (defined here; visible to all module tabs)
// -----------------------------------------------------------------------------
SpscPublisher<SensorFrame> g_sensor_pub;   // core 1 -> core 0
SpscPublisher<ValveCmd>    g_valve_pub;    // core 0 -> core 1


volatile uint32_t g_i2c_rc_hist[8]   = {0};  // endTransmission rc; [5]=timeout, [2]=addr NACK
volatile uint32_t g_loop1_overruns   = 0;    // ticks that blew the 10 ms deadline
volatile uint32_t g_i2c_last_fail_tk = 0;    // core-1 tick of the most recent nonzero rc

volatile uint32_t g_core1_heartbeat = 0;   // core 1 tick counter (liveness)
uint16_t g_servo_us_echo[SERVO_COUNT] = { 0 };   // resolved servo us (core 1 only)

// Manual servo bring-up override: console (core 0) posts a target; core 1 writes
// it inside its servo tick so all I2C stays on core 1. Bench bring-up only --
// bypasses the valve-command staleness failsafe while active.
volatile bool     g_servo_manual  = false;
volatile uint8_t  g_servo_man_dev = 0;      // PCA9685 device index 0..2
volatile uint8_t  g_servo_man_ch  = 0;      // channel 0..PCA9685_MAX_CH-1
volatile uint16_t g_servo_man_tbl[PCA9685_COUNT][PCA9685_MAX_CH] = {{0}};
volatile uint8_t  g_servo_wave_dev = 0;
volatile uint8_t  g_servo_wave_ch  = 0;
volatile uint16_t g_servo_man_us  = SERVO_US_NEUTRAL;
volatile uint32_t g_current_rpm_cmd = 0;   // core 0 -> core 1 (closes sim flow loop)
volatile bool     g_core0_ready = false;
volatile bool     g_core1_ready = false;

// -----------------------------------------------------------------------------
//  Command / arm "inboxes" -- written by the sim OR by real binary frames,
// read by arbitration/allocation. (All producers/consumers are on core 0.)
StickInput g_primary_in = { 0,0,0,0, MDOT_TARGET_DEFAULT, false, 0 };
StickInput g_sbus_in    = { 0,0,0,0, 0.0f,                false, 0 };

volatile uint8_t  g_arm_state   = 0;   // last arm-token state (1 = armed intent)
volatile uint32_t g_arm_counter = 0;   // last arm-token monotonic counter
volatile uint32_t g_arm_stamp   = 0;   // millis() when the token last updated
bool g_sim_arm_intent = false;         // simulated Pixhawk arm channel (console arm/disarm)

bool g_sbus_failsafe  = false;         // simulated SBUS failsafe flag
bool g_sbus_framelost = false;         // simulated SBUS frame-lost flag

// Fault injection (console-set on core 0; read by the core-1 sensor sim).
volatile bool g_fault_valve[VALVE_COUNT] = { false };
volatile bool g_fault_aggregate = false;

// Curve-fit calibration (loaded by cal_littlefs; consumed by servos).
float   g_servo_cubic[SERVO_COUNT][4];      // per-servo c0..c3 (us = c0 + c1 x + c2 x^2 + c3 x^3)
uint8_t g_servo_valve_map[SERVO_COUNT];     // which valve drives each servo
float   g_valve_gain[VALVE_COUNT];          // per-valve gain schedule
float   g_valve_bias[VALVE_COUNT];          // per-valve bias schedule
float   g_dp_zero[VALVE_COUNT] = { 0 };     // per-valve no-flow dp offset (mbar), from cal / 'zero'
bool    g_cal_from_flash = false;           // true if coeffs came from LittleFS

// -----------------------------------------------------------------------------
//  Node status snapshot  (produced by core 0; read by console/tlm/LED)
// -----------------------------------------------------------------------------
NodeStatus g_status;

// Console output mode: text (human) vs binary telemetry (Pi/script).
bool g_binary_tlm = false;

// -----------------------------------------------------------------------------
//  Forward decls implemented in module tabs (Arduino auto-prototypes too, but
//  explicit decls keep the intent legible).
// -----------------------------------------------------------------------------
void framing_setup();
void framing_pump();                       // service USB RX (binary + text)
void tlm_service(uint32_t now);            // emit binary telemetry if enabled
void console_service(uint32_t now);        // periodic human status line

void primary_sim_update(uint32_t now);
void sbus_sim_update(uint32_t now);

void arbitration_update(uint32_t now);     // -> g_status.source, .armed, .arm_live
void allocation_update(uint32_t now);      // -> publishes ValveCmd
void compressor_update(uint32_t now);      // -> rpm, Teensy stream, g_current_rpm_cmd

void sensors_setup();
void sensors_tick(uint32_t tick);          // core 1
void servos_setup();
void servos_service(uint32_t tick);        // core 1

void cal_load();                           // LittleFS -> curve-fit coeffs (or defaults)
void watchdog_setup();
void watchdog_service(uint32_t now);       // RGB + conditional hardware pet
void led_setup();

// =============================================================================
//  CORE 0
// =============================================================================
void setup() {
  Serial.begin(115200);
  led_setup();
  framing_setup();
  cal_load();                 // curve-fit coefficients: flash cal or compiled defaults
  #if USE_REAL_SBUS
  sbus_real_setup();
  #endif

  // Teensy (compressor) link on uart1 / Serial2, GP4 TX / GP5 RX.
  Serial2.setTX(PIN_TEENSY_TX);
  Serial2.setRX(PIN_TEENSY_RX);
  Serial2.begin(TEENSY_BAUD);

  pinMode(PIN_USER_BTN, INPUT_PULLUP);   // BOOT/USER, active low

  g_core0_ready = true;

  // Wait for core 1 to finish its I2C bring-up BEFORE arming the watchdog, so a
  // slow or momentarily-stuck first probe can't trip a reset loop during boot.
  // (The watchdog is armed last, right before loop() starts petting it.)
  uint32_t t0 = millis();
  while (!g_core1_ready && (millis() - t0) < 2000) { delay(1); }

  Serial.println();
  Serial.println(F("RP2350 valve/sensor node -- ALPHA"));
  Serial.println(F("type 'help' for the bench console"));

  watchdog_setup();   // arm the hardware watchdog LAST
}

// User button edge -> toggle simulated primary presence (simulate Pi loss).
static void poll_user_button(uint32_t now) {
  static bool     last = true;      // pulled-up = released = HIGH
  static uint32_t last_change = 0;
  bool cur = digitalRead(PIN_USER_BTN);
  if (cur != last && (now - last_change) > 40) {   // debounce
    last_change = now;
    if (cur == LOW) {              // pressed
      g_status.primary_present = !g_status.primary_present;
      Serial.printf("[btn] primary_present -> %s\n",
                    g_status.primary_present ? "ON" : "OFF");
    }
    last = cur;
  }
}

void loop() {
  uint32_t now = millis();

  framing_pump();               // ingest USB (binary frames and/or text commands)
  poll_user_button(now);

  primary_sim_update(now);      // simulated Pi command + arm token
  #if !USE_REAL_SBUS
  sbus_sim_update(now);         // simulated reversionary SBUS
  #else
  sbus_real_update(now);
  #endif
  arbitration_update(now);      // choose source; enforce arm-token liveness
  allocation_update(now);       // mixing matrix -> 6 valves -> publish to core 1
  compressor_update(now);       // outer loop / direct / fallback -> Teensy stream

  tlm_service(now);             // binary telemetry out (if 'tlm on')
  console_service(now);         // human status line (if 'mon on')
  watchdog_service(now);        // RGB status + conditional hardware watchdog pet
}

// =============================================================================
//  CORE 1  --  fixed 100 Hz I2C service
// =============================================================================
// Release a slave that may be holding SDA low from an aborted transfer at
// power-up (a power-ramp race -- the intermittent boot brownout/reset cause).
// Harmless when the bus is already free.
#if USE_REAL_I2C
static void bus_clear() {
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
  delayMicroseconds(50);
  if (digitalRead(PIN_I2C_SDA) == HIGH) return;         // bus free
  pinMode(PIN_I2C_SCL, OUTPUT);
  for (int i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; ++i) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_I2C_SDA, OUTPUT);                          // STOP: SDA low->high, SCL high
  digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
}
#endif

void setup1() {
#if USE_REAL_I2C
  delay(100);                 // let rails settle before touching the bus
  bus_clear();                // release a stuck SDA before Wire takes the pins
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);      // scanner proved 400 kHz stable on this bus
  Wire.setTimeout(25 /*ms*/, true);   // 2nd arg = reset the peripheral on timeout
#endif
  sensors_setup();
  servos_setup();
  g_core1_ready = true;
}

void loop1() {
  static uint32_t tick = 0;
  static uint32_t next_deadline_us = 0;
  if (next_deadline_us == 0) next_deadline_us = micros() + TICK_US;

  // ---- the 10 ms tick body ----
  sensors_tick(tick);                       // read-back, venturi, publish frame
  if (g_core0_ready && (tick % SERVO_EVERY_N) == 0) {  // every other tick -> 50 Hz
    servos_service(tick);                   // gated: cal_load() completes before g_core0_ready
  }

  g_core1_heartbeat = tick;                 // liveness beat (watched by core 0)
  tick++;

  // ---- pace to the deadline ----
  int32_t remaining = (int32_t)(next_deadline_us - micros());
  if (remaining > 0) {
    // busy-wait the last stretch for tighter jitter than delay() gives
    if (remaining > 200) delayMicroseconds(remaining - 100);
    while ((int32_t)(next_deadline_us - micros()) > 0) { /* spin */ }
  }
  next_deadline_us += TICK_US;
  // If we ever overran badly, resync the deadline so we don't spiral.
  if ((int32_t)(micros() - next_deadline_us) > (int32_t)TICK_US) {
    next_deadline_us = micros() + TICK_US;
    g_loop1_overruns++;
  }
}
