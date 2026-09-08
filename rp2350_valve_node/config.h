// =============================================================================
//  config.h  --  RP2350 Valve/Sensor Node (ALPHA)
//
//  Single place for pins, I2C addresses, timing, and tunables. Everything that
//  a bench/hardware change would touch lives here. Values marked  <<OPEN #n>>
//  correspond to the open decisions in rp2350_valve_node_architecture.md and are
//  seeded with clearly-marked placeholders for the bare-board alpha.
//
//  Target: Pimoroni Tiny 2350 (RP2350A), earlephilhower arduino-pico, IDE 2.x.
// =============================================================================
#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
//  Build-mode switches
// -----------------------------------------------------------------------------
// ALPHA runs entirely on simulated hardware leaves (no sensors/servos/SBUS RX
// required). Flip these to 1 as real hardware arrives; the interfaces do not
// change, only the backend behind them.
#define USE_REAL_I2C     1     // 0 = simulated sensors/servos, no Wire traffic
#define USE_REAL_SBUS    0     // 0 = simulated SBUS source, PIO program stubbed
#define USE_REAL_TEENSY  1     // 1 = actually emit Host_comm frames on Serial2
                               //     (harmless with nothing attached)

// -----------------------------------------------------------------------------
//  Pin map  (Tiny 2350 -- 12 broken-out GPIO; see tiny2350_pinout_diagram.pdf)
// -----------------------------------------------------------------------------
//  Single merged I2C bus lives on the Qw/ST (Qwiic/STEMMA) header = I2C0.
#define PIN_I2C_SDA        12   // Qwiic SDA  (RP2350 I2C0 SDA)
#define PIN_I2C_SCL        13   // Qwiic SCL  (RP2350 I2C0 SCL)

#define PIN_TEENSY_TX       4   // -> Motor Teensy  (uart1 / Serial2 TX)
#define PIN_TEENSY_RX       5   // <- Motor Teensy  (uart1 / Serial2 RX)

#define PIN_SBUS            6   // reversionary SBUS RX-B (PIO; simulated in alpha)

#define PIN_OE_COANDA       7   // active-low output-enable, coanda servo bank
#define PIN_OE_EMERG       26   // active-low output-enable, emergency surfaces
                                //   (separate OE domain -- routine coanda
                                //    dead-man must NOT disable the surfaces)

#define PIN_LED_R          18   // onboard RGB, ACTIVE LOW
#define PIN_LED_G          19
#define PIN_LED_B          20
#define PIN_USER_BTN       23   // onboard BOOT/USER, ACTIVE LOW
                                //   press = simulate primary (Pi) loss

// Spare, broken out: GP0 GP1 GP2 GP3 GP27 GP28 GP29

// -----------------------------------------------------------------------------
//  I2C device addresses   <<OPEN #1 -- confirm against hardware>>
// -----------------------------------------------------------------------------
//  Merged-bus address hygiene (all three families share one address space):
//    - MS5837 sensors all answer 0x76, DOWNSTREAM of the muxes.
//    - PCA9685 all-call default = 0x70 (our ALL_LED_OFF failsafe target).
//    - PCA9685 default sub-addresses = 0x71/0x72/0x73 (disabled at reset, but
//      we keep the muxes off them anyway).
//    => PCA9545 switches use 0x74/0x75/0x77 -- clear of 0x70, 0x71-73, and 0x76.
#define ADDR_MS5837        0x76
#define ADDR_PCA9545_A     0x71
#define ADDR_PCA9545_B     0x72
#define ADDR_PCA9545_C     0x74
#define ADDR_PCA9685_0     0x42   // servo driver "A"
#define ADDR_PCA9685_1     0x44   // servo driver "B"
#define ADDR_PCA9685_2     0x48   // servo driver "C"
#define PCA9685_COUNT         3
#define PCA9685_MAX_CH        6    // channels used per device (<=16 chip max)
#define ADDR_PCA9685_ALL   0x70   // all-call: broadcast ALL_LED_OFF failsafe

// Software servo failsafe via the PCA9685 all-call (backup to the OE hardware
// disable). KEEP 0 until the mux is strapped OFF 0x70 -- the all-call address
// (0x70) collides with a mux left at 0x70, and a broadcast would scramble the
// sensor mux routing. Set to 1 only after the mux moves (e.g. to 0x74/0x75/0x77).
#define USE_SERVO_ALLCALL_FAILSAFE 0

// -----------------------------------------------------------------------------
//  Counts
// -----------------------------------------------------------------------------
#define VALVE_COUNT         6     // aero-intent valve positions (allocator out)
#define SERVO_COUNT        12     // physical servos (curve-fit expansion out)
#define SENSOR_COUNT       12     // MS5837 pressure sensors

// -----------------------------------------------------------------------------
//  Core-1 I2C service timing
// -----------------------------------------------------------------------------
#define TICK_HZ           100
#define TICK_US         10000UL   // 10 ms
#define SERVO_EVERY_N       2     // write servos every other tick -> 50 Hz
#define D2_CADENCE_TICKS   16     // temperature (D2) broadcast cadence (~6 Hz)
#define MS5837_OSR       8192     // max oversampling for low noise (~0.016 mbar RMS).
                                  //   >4096 needs a 2-tick conversion latency (see below);
                                  //   dial back toward 1024 if you want 100 Hz sensor updates.

// ---- MS5837-02BA read tuning (real-sensor path) ----
// Variant confirmed at bring-up: -02BA (the -02BA compensation yields local
// atmospheric; -30BA yields ~20 bar nonsense on the same raw counts).
#define ROLE_UP 0                 // upstream sensor of a valve pair (higher static)
#define ROLE_LO 1                 // throat sensor (lower static)

// OSR -> conversion command + read-back latency (in 100 Hz ticks). A conversion
// must finish before it's read; <=4096 fits one 10 ms tick, 8192 (~18 ms) needs two.
#if   MS5837_OSR == 256
  #define MS_D1_CMD 0x40
  #define MS_D2_CMD 0x50
  #define CONV_LATENCY_TICKS 1
#elif MS5837_OSR == 512
  #define MS_D1_CMD 0x42
  #define MS_D2_CMD 0x52
  #define CONV_LATENCY_TICKS 1
#elif MS5837_OSR == 1024
  #define MS_D1_CMD 0x44
  #define MS_D2_CMD 0x54
  #define CONV_LATENCY_TICKS 1
#elif MS5837_OSR == 2048
  #define MS_D1_CMD 0x46
  #define MS_D2_CMD 0x56
  #define CONV_LATENCY_TICKS 1
#elif MS5837_OSR == 4096
  #define MS_D1_CMD 0x48
  #define MS_D2_CMD 0x58
  #define CONV_LATENCY_TICKS 1
#elif MS5837_OSR == 8192
  #define MS_D1_CMD 0x4A
  #define MS_D2_CMD 0x5A
  #define CONV_LATENCY_TICKS 3     // ~18 ms > 10 ms tick + 1-tick buffer for header-> read 3 ticks after the kick
#else
  #error "MS5837_OSR must be 256/512/1024/2048/4096/8192"
#endif
#define MS_ADC_READ_CMD  0x00
#define MS_RESET_CMD     0x1E

// Per-sensor EMA on compensated pressure: p += alpha*(new - p). Smaller alpha =
// more smoothing + more lag. alpha=0.1 ~ 10-sample time constant (~200 ms at the
// 50 Hz OSR-8192 sensor rate) -- negligible against sub-1-Hz dynamics.
#define SENS_EMA_ALPHA     0.10f

// real-sensor guards + venturi
#define MS_P_RANGE_MIN    300.0f  // mbar; below -> invalid
#define MS_P_RANGE_MAX   2000.0f  // mbar; -02BA ceiling
#define MS_T_RANGE_MIN    -40.0f  // C
#define MS_T_RANGE_MAX     85.0f  // C
#define VENTURI_DP_MIN     0.05f  // mbar; zero-corrected dp below this reads as NO FLOW.
#define AREA_INLET    0.00038795  // m^2 ; Cross Sectional Area at the inlet of the venturi
#define AREA_EXIT     0.00010693  // m^2 ; Area at the inlet of the venturi
#define AREA_RATIO    3.628074     // Area Inlet / Area Exit [Used for Venturi Calc]

                                  //   Set to ~3-5x the OBSERVED post-filter dpcorr noise
                                  //   (watch 'sens' at no flow). 0.05 -> ~0.6 g/s floor at
                                  //   K=0.18. Lower it only as far as the measured noise allows.
#define R_AIR             287.05f // J/(kg K), dry-air specific gas constant
                                  //   beta 0.7, D=2.54 cm -> K~0.18). Geometry w/ Cd=0.98
                                  //   predicts ~2x less dp, so Cd is uncertain: CALIBRATE
                                  //   K against a reference flow before trusting mdot magnitude.

// Core-1 drives servos to failsafe if a valve command ages past this
// (independent of core-0 arbitration).
#define CMD_TIMEOUT_MS    100

// -----------------------------------------------------------------------------
//  Core-0 arbitration / arm timing
// -----------------------------------------------------------------------------
#define USB_CMD_TIMEOUT_MS       80   // PRIMARY considered fresh below this age
#define PRIMARY_TO_SBUS_HOLD_MS 150   // hysteresis: USB must be stale this long
#define SBUS_TO_PRIMARY_HOLD_MS 200   // ...and fresh this long before switching back

#define ARM_HEARTBEAT_MS       1000   // expected arm-token cadence (1 Hz)
#define ARM_LOSS_TIMEOUT_MS   10000   // powered-reversion window, then disarm
                                      //   <<default per spec; configurable>>

// -----------------------------------------------------------------------------
//  Teensy (compressor) link -- reuse teensyshot Host_comm format & magic
// -----------------------------------------------------------------------------
#define TEENSY_BAUD           115200
#define TEENSY_MAGIC      0x43305735  // "teensyshot" magic, both directions
#define TEENSY_STREAM_HZ         50   // >=25 Hz to satisfy the ~40 ms dead-man;
                                      //   resend static values, never gate on change
#define TEENSY_NB_SLOTS           6   // Host_comm sizes to 6 -> 64-byte packet
// teensyshot firmware PID defaults (raw ints; firmware * 1e-4). Never send 0xFFFF
// on all four gains of slot 0 -- that reboots the Teensy.
#define TEENSY_DEF_P    400
#define TEENSY_DEF_I   1050
#define TEENSY_DEF_D   1000
#define TEENSY_DEF_F   9900

// -----------------------------------------------------------------------------
//  Compressor command / mass-flow outer loop
// -----------------------------------------------------------------------------
#define RPM_FALLBACK      30000   // <<COMPILED-IN & IMMUTABLE>> applied when the
                                  //   aggregate mass-flow estimate is untrustworthy.
                                  //   Fails toward MORE airflow (coanda authority).
                                  //   Flash cal MAY override the operating target,
                                  //   but this constant is never flash-only.
#define RPM_CMD_MAX       60000   // ceiling on any commanded rpm
#define RPM_CMD_MIN           0
#define RPM_SLEW_PER_S    20000   // outer-loop rate limit (rpm/s) -- gentle

#define MDOT_TARGET_DEFAULT  40.0f  // g/s (used until a source commands otherwise)
#define MDOT_PLAUSIBLE_MIN    0.0f  // g/s  aggregate plausibility window
#define MDOT_PLAUSIBLE_MAX  400.0f  // g/s
#define MIN_VALID_VALVES        4   // fewer valid valves than this -> untrustworthy
#define FLOW_PI_KP           50.0f  // rpm per (g/s) error
#define FLOW_PI_KI           20.0f  // rpm per (g/s * s)
#define FLOW_FALLBACK_HOLD_MS 250   // hysteresis into/out of fallback (anti-chatter)

// Direct-rpm mapping for SBUS manual reversion  <<OPEN #5 -- direct rpm>>
#define SBUS_RPM_MIN          0
#define SBUS_RPM_MAX      45000

// -----------------------------------------------------------------------------
//  Valve / servo ranges and curve-fit
// -----------------------------------------------------------------------------
#define VALVE_POS_MIN      (-1000)  // int16 aero-intent range
#define VALVE_POS_MAX      ( 1000)
#define SERVO_US_MIN         1000
#define SERVO_US_NEUTRAL     1500
#define SERVO_US_MAX         2000

// DEFINED-SAFE valve pose  <<OPEN #4 -- REPLACE with aero-correct pose>>
// PLACEHOLDER: neutral. The real pose almost certainly biases toward AIRFLOW to
// preserve coanda authority in the terminal no-command state, NOT geometric zero.
// Do not fly this placeholder.
#define DEFINED_SAFE_VALVE_POSE { 0, 0, 0, 0, 0, 0 }

// -----------------------------------------------------------------------------
//  Pi link (USB CDC) binary framing
// -----------------------------------------------------------------------------
#define PI_MAGIC       0x52503233   // distinct from the Teensy magic
#define FT_CMD               0x01   // Pi -> node : fast command
#define FT_ARM               0x02   // Pi -> node : slow arm token (1 Hz + on-change)
#define FT_CTRL_TLM          0x81   // node -> Pi : control telemetry (25 Hz)
#define FT_SENSOR_TLM        0x82   // node -> Pi : sensor telemetry (25 Hz)
#define PI_TLM_EACH_HZ         25   // each of the two out-frames at this rate

// -----------------------------------------------------------------------------
//  Watchdog
// -----------------------------------------------------------------------------
#define WDT_TIMEOUT_MS       500   // hardware watchdog period
#define WDT_BOOT_GRACE_MS   1500   // pet unconditionally during early boot
#define CORE1_STALL_TRIP_MS  120   // core-1 heartbeat must advance within this
