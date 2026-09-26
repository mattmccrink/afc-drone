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
#define USE_REAL_PRIMARY 1   // 0 = simulated Pi command+arm (sweep), 1 = real FT_CMD/FT_ARM only
#define USE_REAL_I2C     1     // 0 = simulated sensors/servos, no Wire traffic
#define USE_REAL_SBUS    1     // 0 = simulated SBUS source, PIO program stubbed
#define USE_REAL_TEENSY  1     // 1 = actually emit Host_comm frames on Serial1
                               //     (harmless with nothing attached)

// Bench-only fault hooks (console 'hang core0|core1', 'tdrop N'). They exist to
// exercise the watchdog / reset / dropped-frame paths (T-S3, T-S7, T-C4, T-C5).
// MUST be 0 in any build that flies: a hook left reachable is a way to hang a
// core or starve the compressor from the console (the LINK_TEST lesson).
#define BENCH_HOOKS      0
#if BENCH_HOOKS
  #warning "BENCH_HOOKS=1: bench fault hooks compiled in -- NOT a flight build"
#endif

// -----------------------------------------------------------------------------
//  Pin map  (Tiny 2350 -- 12 broken-out GPIO; see tiny2350_pinout_diagram.pdf)
// -----------------------------------------------------------------------------
//  Single merged I2C bus lives on the Qw/ST (Qwiic/STEMMA) header = I2C0.
#define PIN_I2C_SDA        12   // Qwiic SDA  (RP2350 I2C0 SDA)
#define PIN_I2C_SCL        13   // Qwiic SCL  (RP2350 I2C0 SCL)

#define PIN_PI_TX   4          // GP4 -> Pi RXD (was PIN_TEENSY_TX)
#define PIN_PI_RX   5          // GP5 <- Pi TXD (was PIN_TEENSY_RX)
#define PI_BAUD     230400     // soldered single-ended run; CRC-protected, low-rate

#define PIN_SBUS            6   // reversionary SBUS RX-B (PIO; simulated in alpha)

#define PIN_OE_PCA          7   // active-low output-enable, ALL PCA9685 boards.
                                //   Held LOW (enabled) permanently from boot: the
                                //   valve + surface servos always receive a pulse.
                                //   Failsafe = drive the defined pose, never
                                //   de-power (hobby servos go limp on pulse loss).
                                //   GP26 (old emergency-surface OE) is now spare.

#define PIN_LED_R          18   // onboard RGB, ACTIVE LOW
#define PIN_LED_G          19
#define PIN_LED_B          20
#define PIN_USER_BTN       23   // onboard BOOT/USER, ACTIVE LOW
                                //   press = simulate primary (Pi) loss

// Spare, broken out: GP2 GP3 GP26 GP27 GP28 GP29   (GP0/1 = Teensy, GP4/5 = Pi)

// -----------------------------------------------------------------------------
//  I2C device addresses   <<OPEN #1 -- confirm against hardware>>
// -----------------------------------------------------------------------------
//  Merged-bus address hygiene (all three families share one address space):
//    - MS5837 sensors all answer 0x76, DOWNSTREAM of the muxes.
//    - PCA9685 all-call default = 0x70 (unused, but kept clear of the muxes).
//    - PCA9685 default sub-addresses = 0x71/0x72/0x73 (disabled at reset, but
//      we keep the muxes off them anyway).
//    => PCA9545 switches use 0x74/0x75/0x77 -- clear of 0x70, 0x71-73, and 0x76.
#define ADDR_MS5837        0x76
#define ADDR_PCA9545_A     0x74
#define ADDR_PCA9545_B     0x75
#define ADDR_PCA9545_C     0x77
#define ADDR_PCA9685_0     0x44   // servo driver "A"
#define ADDR_PCA9685_1     0x48   // servo driver "B"
#define ADDR_PCA9685_2     0x50   // servo driver "C"
#define PCA9685_COUNT         3
#define PCA9685_MAX_CH        6    // channels written per device (<=16 chip max):
                                   //   ch0-3 = valve servos, ch4-5 = surfaces/spare
#define VALVE_SERVOS_PER_PCA  4    // valve servo s -> PCA s/4, channel s%4 (SERVO_OUT_MAP)
// PCA9685 all-call (0x70) is NOT used: the servo failsafe drives a defined pose
// and never de-powers the outputs (Q7), so the ALL_LED_OFF backup was removed.

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
// ---- per-sensor / per-venturi health (real I2C path; sensors.ino) ----
// A venturi is VALID only if both of its sensors are healthy AND the pair is
// plausible. Invalid venturis drop out of mdot_total and n_valid; too few valid
// -> compressor holds RPM_FALLBACK (compressor.ino, MIN_VALID_VALVES).
//   <<PLACEHOLDERS -- tighten against observed behaviour; see 'vhealth'>>
#define SENS_STALE_MS         200   // no good pressure (D1) read this long -> STALE
                                    //   (D1 lands every 30 ms, 60 ms across a D2, so ~6
                                    //   consecutive misses; rides through the odd timeout)
#define SENS_T_STALE_MS      2000   // no good temperature (D2) read this long -> STALE
                                    //   (D2 lands ~every 0.5 s)
#define SENS_FROZEN_N          30   // this many IDENTICAL consecutive raw D1 counts -> FROZEN
                                    //   (~1 s). At OSR 8192 a live -02BA's raw noise is tens
                                    //   of LSB, so an exact repeat run this long means a
                                    //   stuck part / stuck bus, not a quiet sensor.
#define SENS_D1_JUMP_MAX   100000UL // raw D1 counts; a read differing from the previous good
                                    //   one (< SENS_STALE_MS old) by more is a SPIKE: rejected
                                    //   and counted as a failed read (no bus CRC on I2C).
                                    //   ~0.00044 mbar/count on a typical -02BA -> ~44 mbar in
                                    //   30 ms. A real step is accepted on the NEXT read if that
                                    //   read confirms the new level (costs ~30 ms).
#define SENS_D2_JUMP_MAX   160000UL // raw D2 counts; same rule for temperature (~5 C per
                                    //   ~0.5 s D2 interval on a typical -02BA). A corrupt D2
                                    //   would otherwise skew compensation until the next D2.
#define SENS_FAIL_WEIGHT        2   // loss-rate score: +this per failed read, -1 per good read
#define SENS_FAIL_TRIP         60   //   -> LOSSY while score >= this. Trips when more than
                                    //   ~1/(1+WEIGHT) = 1/3 of reads fail, sustained.
#define SENS_RECOVER_MS       500   // a slot must stay clean this long after ANY fault
                                    //   before its venturi counts as valid again (anti-flap)
#define VENTURI_DP_NEG_MAX   5.0f   // mbar; zero-corrected dp below -this -> IMPLAUSIBLE
                                    //   (throat reading above upstream: swapped / failed
                                    //   sensor). Needs 'zero' captured, or raw sensor
                                    //   offsets can approach this margin.

// 'zero' guards: an offset this large is a sensor/port fault, not an offset to
// absorb; and it needs genuinely still air -- the rotor COASTS after disarm and
// coast-down isn't observable over the link, so wait this long after disarm.
#define ZERO_MAX_OFFSET     10.0f   // mbar  <<tighten to ~3x observed pair spread>>
#define ZERO_SETTLE_MS      30000   // ms after the last armed moment  <<set from measured coast-down>>

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
#define ARM_LOSS_TIMEOUT_MS   10000   // FC-token liveness window (reversion, not disarm)
                                      //   <<default per spec; configurable>>
#define ARM_TOKEN_FRESH_MS     1500   // PRIMARY ACTS on the token only if it advanced this
                                      //   recently (1.5 heartbeats). An older value (e.g. from
                                      //   before a termination/bridge restart) is never obeyed;
                                      //   the node holds its state until a fresh token lands.

// Total-comms-loss flight termination: how long BOTH sources may be dead (SRC_SAFE)
// before the node cuts air and comes down. Debounce against a transient double
// dropout -- long enough that a real reversion (~150 ms) always wins, short enough
// that the airframe doesn't fly far uncontrolled. <<TUNE to airframe/risk.>>
#define SAFE_TERMINATE_MS      3000

// -----------------------------------------------------------------------------
//  Teensy (compressor) link -- reuse teensyshot Host_comm format & magic
// -----------------------------------------------------------------------------
#define TEENSY_BAUD           921600
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
#define MIN_VALID_VALVES VALVE_COUNT // fewer valid venturis than this -> untrustworthy ->
                                    //   RPM_FALLBACK. = ALL six (was 4): an invalid venturi
                                    //   contributes 0 to mdot_total, so tracking on a partial
                                    //   sum would UNDER-read flow and the mdot PI would drive
                                    //   rpm up to chase flow it can't see. Lower this only
                                    //   together with a missing-flow estimate for dead venturis.
#define FLOW_PI_KP           50.0f  // rpm per (g/s) error
#define FLOW_PI_KI           20.0f  // rpm per (g/s * s)
#define FLOW_FALLBACK_HOLD_MS 250   // hysteresis into/out of fallback (anti-chatter)
#define COMP_MDOT_LOOP        0   // 0 = stub (armed -> RPM_FALLBACK); 1 = mdot PI (tracks only with all venturis valid)

// Spin-up/spin-down shaping lives on the TEENSY (reference rate limit, ~5 s to
// 30k). The Tiny does NOT ramp from 0 on a run edge; it sends the target and the
// Teensy shapes it. RPM_SLEW_PER_S above only shapes in-run target changes.
#define COMP_SPINUP_GRACE_MS  6000  // Teensy ramp (5 s) + margin; flag checks wait this long

// ESC thermal-derate suspicion flag (advisory only -- no control action).
// Sets when the ESC is hot AND rpm has sagged below target for a sustained
// window (the signature of the ESC self-throttling while the PID pushes).
// Clears when it cools OR rpm recovers. Reported in COMP_TLM -> /afc/compressor.
#define COMP_THERM_ON_C        90   // deg C, set threshold
#define COMP_THERM_OFF_C       85   // deg C, clear threshold (hysteresis)
#define COMP_THERM_LAG_FRAC  0.10f  // rpm < (1-this)*target counts as sagging
#define COMP_THERM_OK_FRAC   0.05f  // rpm >= (1-this)*target counts as recovered
#define COMP_THERM_SUSTAIN_MS 3000  // sag must persist this long while hot

// -----------------------------------------------------------------------------
//  Valve / servo ranges and curve-fit
// -----------------------------------------------------------------------------
#define VALVE_POS_MIN      (-1000)  // int16 aero-intent range
#define VALVE_POS_MAX      ( 1000)
#define SERVO_US_MIN         1000
#define SERVO_US_NEUTRAL     1500
#define SERVO_US_MAX         2000

// DEFINED-SAFE valve pose (valve domain, through the curve fit). Decision
// 2026-09-22: "0" = aero neutral; per-valve bias is calibrated out in the curve
// fit (g_valve_bias / servo cubic c0), so 0 here means centered after cal.
// Applied: at boot until a source is heard, in SAFE, when terminated, and on a
// stale core-0 command (core 1 keeps pulsing this pose; servos never go limp).
#define DEFINED_SAFE_VALVE_POSE { 0, 0, 0, 0, 0, 0 }
#define VALVE_TRIM_NORM { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }   // per-valve neutral, NORMALIZED [-1,1], 0=center. <<POPULATE>>

// -----------------------------------------------------------------------------
//  Traditional control surfaces (rung 2a -- pilot-selected "maximum effort")
// -----------------------------------------------------------------------------
//  Engaged by an SBUS switch (SBUS_CH_SURF in sbus_real.ino). Engaged = AFC +
//  surfaces allocated TOGETHER through one combined pseudo-inverse over all 10
//  actuators (same moment per unit demand -> PX4 loop gain unchanged; more total
//  authority). Compressor and AFC stay on. Disengaged, SAFE, or terminated =
//  surfaces held at their configured center. Surfaces share the valve PCA9685s.
#define SURF_COUNT            4
//  Surface index -> PCA9685 {device, channel}.  0=L wing (A ch4), 1=R wing
//  (B ch4), 2=L canard (C ch4), 3=R canard (C ch5).
#define SURF_OUT_MAP     { {0,4}, {1,4}, {2,4}, {2,5} }
#define SURF_US_CENTER   { 1500, 1500, 1500, 1500 }   // us at 0 command <<SET per surface>>
#define SURF_US_THROW    {  400,  400,  400,  400 }   // us per unit command (|u|<=1)
#define SURF_DIR         {   +1,   +1,   +1,   +1 }   // +1 / -1 to reverse a servo
//  Surface effectiveness (rows roll,pitch,yaw; cols surfaces 0..3), same units
//  and sign convention as B_EFF.  <<PLACEHOLDER: signs/magnitudes unverified.>>
//  Wings = ailerons (antisymmetric roll); canards = symmetric pitch; no yaw.
#define B_SURF_INIT { \
  { +0.50f, -0.50f,  0.00f,  0.00f },   /* roll  */ \
  {  0.00f,  0.00f, +0.50f, +0.50f },   /* pitch */ \
  {  0.00f,  0.00f,  0.00f,  0.00f } }  /* yaw   */

// -----------------------------------------------------------------------------
//  Pi link (Serial2 UART, 230400) binary framing
// -----------------------------------------------------------------------------
#define PI_MAGIC       0x52503233   // distinct from the Teensy magic
#define FT_CMD               0x01   // Pi -> node : fast command
#define FT_ARM               0x02   // Pi -> node : slow arm token (1 Hz + on-change)
#define FT_CTRL_TLM          0x81   // node -> Pi : control telemetry (25 Hz)
#define FT_SENSOR_TLM        0x82   // node -> Pi : sensor telemetry (25 Hz)
#define FT_COMP_TLM          0x83    // node -> Pi : measured compressor telemetry (ESC)
#define PI_TLM_EACH_HZ         25   // each of the two out-frames at this rate

// -----------------------------------------------------------------------------
//  Watchdog
// -----------------------------------------------------------------------------
#define WDT_TIMEOUT_MS       500   // hardware watchdog period
#define WDT_BOOT_GRACE_MS   1500   // pet unconditionally during early boot
#define CORE1_STALL_TRIP_MS  120   // core-1 heartbeat must advance within this
