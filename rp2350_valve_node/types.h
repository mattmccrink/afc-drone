// =============================================================================
//  types.h  --  Shared data structures + lock-free inter-core publisher
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

// -----------------------------------------------------------------------------
//  Inter-core payloads  (mirror the architecture spec, section 5)
// -----------------------------------------------------------------------------

// core 0 -> core 1 : commanded valve positions (aero intent)
struct ValveCmd {
  int16_t  valve[VALVE_COUNT];   // aero-intent positions
  uint32_t stamp_ms;             // set by core 0; core 1 checks staleness
};

// core 1 -> core 0 : sensor + resolved-servo frame
struct SensorFrame {
  float    p_up[VALVE_COUNT];    // upstream pressure  (mbar)
  float    p_lo[VALVE_COUNT];    // throat pressure    (mbar)
  float    t_die[VALVE_COUNT];   // upstream sensor die temp (°C) -- density temp
  float    t_lo[VALVE_COUNT];    // throat sensor die temp (°C) -- cal/tempco telemetry
  float    mdot[VALVE_COUNT];    // per-valve mass flow (g/s)
  float    mdot_total;           // summed over VALID valves only
  uint16_t servo_us[SERVO_COUNT];// resolved servo commands (telemetry echo)
  uint8_t  valid[VALVE_COUNT];   // per-valve validity flag (see sensors_sim)
  uint8_t  n_valid;              // count of valid valves
};

// -----------------------------------------------------------------------------
//  Node enums + app-level structs
//  (defined here, not in the main .ino, so Arduino's auto-generated prototypes
//   -- which are hoisted above the main sketch body -- can see these types.)
// -----------------------------------------------------------------------------
enum Source   { SRC_PRIMARY = 0, SRC_SBUS = 1, SRC_SAFE = 2 };
enum CompMode { COMP_STOPPED = 0, COMP_TRACK, COMP_DIRECT, COMP_FALLBACK };

// A normalized command input; shared shape for every source.
struct StickInput {
  float    roll, pitch, yaw, throttle;   // roll/pitch/yaw in [-1,1], throttle [0,1]
  float    mdot_target;                  // g/s (primary only; sbus uses direct rpm)
  bool     valid;
  uint32_t stamp_ms;
};

struct NodeStatus {
  Source   source          = SRC_SAFE;
  bool     armed           = false;
  bool     arm_live        = false;
  bool     primary_present = true;       // sim: "is the Pi/primary alive?"
  uint16_t rpm_target      = 0;
  CompMode comp_mode       = COMP_STOPPED;
  float    mdot_total      = 0.0f;
  float    mdot_target     = MDOT_TARGET_DEFAULT;
  uint8_t  n_valid         = 0;
  bool     flow_fallback   = false;
  bool     sensor_stale    = false;
};

// One physical MS5837 behind the mux tree: its mux, channel, and which
// (valve, role) it serves. The real sensor path (sensors.ino) walks a table of
// these; the mapping is data, so open decision #1 is an edit, not a rewrite.
struct SensorSlot {
  uint8_t mux;      // mux I2C address (e.g. 0x70)
  uint8_t ch;       // channel 0..7
  uint8_t valve;    // 0..VALVE_COUNT-1
  uint8_t role;     // ROLE_UP or ROLE_LO
};

// Calibration blob persisted to LittleFS (see cal_littlefs.ino). Defined here,
// not in the tab, so Arduino's hoisted prototypes for the helpers that take it
// by reference can see the type.
struct CalBlob {
  uint32_t magic;
  float    servo_cubic[SERVO_COUNT][4];
  uint8_t  servo_valve_map[SERVO_COUNT];
  float    valve_gain[VALVE_COUNT];
  float    valve_bias[VALVE_COUNT];
  float    dp_zero[VALVE_COUNT];   // per-valve no-flow (p_up - p_lo) offset, mbar
  uint32_t crc32;                  // over all preceding bytes
};

// -----------------------------------------------------------------------------
//  SpscPublisher -- single-writer / single-reader lock-free publish.
//
//  Realized as a *seqlock* rather than a 2-buffer index flip: a plain double
//  buffer with a flipped index is NOT torn-read-safe if the reader can be slow
//  relative to the writer (writer may reopen the buffer the reader still holds).
//  A seqlock gives the same single-writer/single-reader publish the spec calls
//  for, with guaranteed torn-read safety and a natural staleness signal (the
//  monotonic `seq`). Writer bumps seq odd, writes, bumps seq even; reader
//  retries if it observes an odd or changed seq.
//
//  Correct here because writer (core 1, 100 Hz) holds the odd window for only a
//  few microseconds while the reader (core 0) copies out -- collisions are rare
//  and bounded; on the (practically never) exhausted-retry case the reader keeps
//  its last-good copy and flags stale.
// -----------------------------------------------------------------------------
template <typename T>
class SpscPublisher {
public:
  volatile uint32_t seq = 0;     // even = stable, odd = write in progress
  T data;

  // ---- writer side (call from exactly one core) ----
  T& begin_write() { seq++; __sync_synchronize(); return data; }
  void end_write() { __sync_synchronize(); seq++; }

  // ---- reader side (call from exactly one core) ----
  // Returns true and fills `out` with a clean snapshot; false if it could not
  // obtain one within the retry budget (writer contention -- effectively never).
  bool snapshot(T& out) {
    for (int tries = 0; tries < 8; ++tries) {
      uint32_t s0 = seq; __sync_synchronize();
      if (s0 & 1u) continue;                 // writer mid-update
      out = data; __sync_synchronize();
      uint32_t s1 = seq;
      if (s0 == s1) return true;             // stable across the copy
    }
    return false;
  }

  uint32_t sequence() const { return seq; }  // for non-advancing == stale checks
};
