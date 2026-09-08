// =============================================================================
//  allocation.ino  --  Control allocation  (core 0)
//
//  Roll/pitch/yaw (+ collective) -> 6 valve positions via a fixed mixing matrix.
//  The SAME allocator serves both sources; SBUS reversion is an alternate input,
//  not an alternate allocator (spec section 6). In DEFINED-SAFE, valves go to the
//  <<OPEN #4>> placeholder pose. Result is published to core 1 as a ValveCmd.
//
//  Note: arm gates the COMPRESSOR only. Valves still move when disarmed (harmless
//  without air; useful for ground checks) -- so allocation runs regardless of arm.
// -----------------------------------------------------------------------------
#include "config.h"

// Mixing matrix  <<PLACEHOLDER -- replace with the real force/moment map>>.
// Columns: {roll, pitch, yaw, collective}. Rows: the 6 valves. Units: fraction of
// full valve travel per unit stick. Kept modest so combined demand rarely rails.
static const float ALLOC_MIX[VALVE_COUNT][4] = {
  //  roll    pitch    yaw    collective
  {  0.60f,  0.30f,  0.20f,  0.50f },   // valve 0
  { -0.60f,  0.30f, -0.20f,  0.50f },   // valve 1
  {  0.60f, -0.30f, -0.20f,  0.50f },   // valve 2
  { -0.60f, -0.30f,  0.20f,  0.50f },   // valve 3
  {  0.00f,  0.45f,  0.35f,  0.50f },   // valve 4
  {  0.00f, -0.45f, -0.35f,  0.50f },   // valve 5
};

static const int16_t ALLOC_SAFE_POSE[VALVE_COUNT] = DEFINED_SAFE_VALVE_POSE;

static inline int16_t clamp_valve(float v) {
  if (v < VALVE_POS_MIN) v = VALVE_POS_MIN;
  if (v > VALVE_POS_MAX) v = VALVE_POS_MAX;
  return (int16_t)lroundf(v);
}

void allocation_update(uint32_t now) {
  ValveCmd& out = g_valve_pub.begin_write();

  if (g_status.source == SRC_SAFE) {
    // Terminal no-command state: hold the defined-safe pose.
    for (int v = 0; v < VALVE_COUNT; ++v) out.valve[v] = ALLOC_SAFE_POSE[v];
  } else {
    const StickInput& in = (g_status.source == SRC_PRIMARY) ? g_primary_in : g_sbus_in;
    float collective = in.throttle;                 // [0,1]
    for (int v = 0; v < VALVE_COUNT; ++v) {
      float f = ALLOC_MIX[v][0] * in.roll
              + ALLOC_MIX[v][1] * in.pitch
              + ALLOC_MIX[v][2] * in.yaw
              + ALLOC_MIX[v][3] * collective;
      out.valve[v] = clamp_valve(f * VALVE_POS_MAX);
    }
  }

  out.stamp_ms = now;                 // core 1 uses this for staleness -> failsafe
  g_valve_pub.end_write();
}
