// =============================================================================
//  servo_cal.h  --  compiled servo calibration: one 4th-order fit per servo
//
//      us = c0 + c1*x + c2*x^2 + c3*x^3 + c4*x^4
//
//  x   = the valve command in [-1, 1] (after the per-valve gain/bias, default
//        1 and 0), i.e. standard control logic.
//  us  = the PWM pulse the servo needs, 1000..2000. The fit already carries the
//        servo's own midpoint and endpoints (e.g. mid 1450, end 1920).
//
//  Servo s is PCA9685 board 'A'+s/4, channel s%4; which VALVE drives it is the
//  runtime map ('map' on the console, saved to flash). Surfaces are separate
//  (SURF_* in config.h).
//
//  These are one-time coanda-valve calibrations, compiled in on purpose: they are
//  NOT stored in or overridden by /cal.bin (which keeps only the servo->valve
//  map, per-valve gain/bias and the venturi zero offsets).
//
//  Checks on the console ('calshow') and at boot ('health'): each fit evaluated at
//  x = -1, 0, +1, its min/max over [-1, 1] against SERVO_US_LIMITS, and whether it
//  is monotonic (a linearizing fit should be).
// =============================================================================
#pragma once

// 1 = the table below is still the linear placeholder. While set, the node
// reports its calibration as incomplete (CTRL_TLM flags2 bit3 clear: dashboard
// CAL DEFAULT, afc_preflight FAIL). Set to 0 when the real fits are in.
#define SERVO_CAL_PLACEHOLDER 1

//                       c0 (us)    c1        c2        c3        c4
#define SERVO_QUARTIC { \
  /* s0  A ch0 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s1  A ch1 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s2  A ch2 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s3  A ch3 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s4  B ch0 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s5  B ch1 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s6  B ch2 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s7  B ch3 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s8  C ch0 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s9  C ch1 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s10 C ch2 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
  /* s11 C ch3 */ {  1500.0f,  500.0f,    0.0f,    0.0f,    0.0f }, \
}

// Per-servo hard clamp, us: set to the calibrated endpoints (the fit's range at
// x = -1 and +1, plus a few us). A fit that overshoots between its data points,
// or a gain/bias that pushes x past 1, can then never drive a servo past its
// measured endpoint. Always also clamped to SERVO_US_MIN..SERVO_US_MAX.
//                     min    max
#define SERVO_US_LIMITS { \
  /* s0  */ { 1000, 2000 }, /* s1  */ { 1000, 2000 }, /* s2  */ { 1000, 2000 }, \
  /* s3  */ { 1000, 2000 }, /* s4  */ { 1000, 2000 }, /* s5  */ { 1000, 2000 }, \
  /* s6  */ { 1000, 2000 }, /* s7  */ { 1000, 2000 }, /* s8  */ { 1000, 2000 }, \
  /* s9  */ { 1000, 2000 }, /* s10 */ { 1000, 2000 }, /* s11 */ { 1000, 2000 }, \
}
