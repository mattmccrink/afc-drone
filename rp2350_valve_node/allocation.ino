// =============================================================================
//  allocation.ino  --  Control allocation  (core 0)
//
//  PX4-style allocation: author the EFFECTIVENESS matrix B (moment produced per
//  unit valve travel), and allocate by its pseudo-inverse -- the same math as
//  PX4's ControlAllocationPseudoInverse, with a coanda B instead of a rotor one.
//
//      tau = B u                         (forward physics, B is 3x6)
//      u   = collective*1 + Binv * tau   (Binv = B^T (B B^T)^-1, min-norm, 6x3)
//
//  Collective is common-mode (all valves): for a symmetric layout 1 ~ null(B),
//  so it adds mass-flow without a net moment and rides under the allocation.
//  Saturation: yaw-priority sequential desaturation (roll/pitch preserved first).
//
//  Sources: the SAME allocator serves PRIMARY and SBUS (spec section 6).
//  SAFE (or a latched termination) -> defined-safe pose; arm gates the compressor.
// -----------------------------------------------------------------------------
#include "config.h"
#include <math.h>

// --- Effectiveness matrix B (rows: roll,pitch,yaw; cols: valves 0..5) ---------
//  Signs from geometry (valves 0-3 outboard, 4-5 canards).
//  <<POPULATE MAGNITUDES from CFD / wind-tunnel / flight system-ID.>>  The values
//  below carry the sign pattern of the old hand mix as a PLACEHOLDER so the
//  pseudo-inverse is exercised end-to-end; they are NOT physically calibrated.
static const float B_EFF[3][VALVE_COUNT] = {
  //  v0      v1      v2      v3      v4      v5
  { +0.60f, -0.60f, +0.60f, -0.60f,  0.00f,  0.00f },   // roll
  { +0.30f, +0.30f, -0.30f, -0.30f, +0.45f, -0.45f },   // pitch
  { +0.20f, -0.20f, -0.20f, +0.20f, +0.35f, -0.35f },   // yaw
};

static float Binv[VALVE_COUNT][3];      // pseudo-inverse (6x3), computed at boot
static const int16_t ALLOC_SAFE_POSE[VALVE_COUNT] = DEFINED_SAFE_VALVE_POSE;
static const float u_trim[VALVE_COUNT] = VALVE_TRIM_NORM;

static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

static inline int16_t clamp_valve(float v) {
  if (v < VALVE_POS_MIN) v = VALVE_POS_MIN;
  if (v > VALVE_POS_MAX) v = VALVE_POS_MAX;
  return (int16_t)lroundf(v);
}

// 3x3 inverse (B B^T is small and well-conditioned for a sane B).
static bool inv3(const float m[3][3], float o[3][3]) {
  float det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
            + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
  if (fabsf(det) < 1e-9f) return false;
  float id = 1.0f/det;
  o[0][0]= (m[1][1]*m[2][2]-m[1][2]*m[2][1])*id;
  o[0][1]=-(m[0][1]*m[2][2]-m[0][2]*m[2][1])*id;
  o[0][2]= (m[0][1]*m[1][2]-m[0][2]*m[1][1])*id;
  o[1][0]=-(m[1][0]*m[2][2]-m[1][2]*m[2][0])*id;
  o[1][1]= (m[0][0]*m[2][2]-m[0][2]*m[2][0])*id;
  o[1][2]=-(m[0][0]*m[1][2]-m[0][2]*m[1][0])*id;
  o[2][0]= (m[1][0]*m[2][1]-m[1][1]*m[2][0])*id;
  o[2][1]=-(m[0][0]*m[2][1]-m[0][1]*m[2][0])*id;
  o[2][2]= (m[0][0]*m[1][1]-m[0][1]*m[1][0])*id;
  return true;
}

// Compute Binv = B^T (B B^T)^-1 once at boot.
void allocation_setup() {
  float BBt[3][3];
  for (int i=0;i<3;i++) for (int j=0;j<3;j++){
    float s=0; for (int k=0;k<VALVE_COUNT;k++) s+=B_EFF[i][k]*B_EFF[j][k];
    BBt[i][j]=s;
  }
  float inv[3][3];
  if (!inv3(BBt, inv)) {                 // singular B -> zero authority (safe)
    for (int v=0;v<VALVE_COUNT;v++) for(int i=0;i<3;i++) Binv[v][i]=0.0f;
    Serial.println(F("[alloc] WARNING: B B^T singular; check B_EFF"));
    return;
  }
  for (int v=0;v<VALVE_COUNT;v++)
    for (int i=0;i<3;i++){
      float s=0; for (int k=0;k<3;k++) s+=B_EFF[k][v]*inv[k][i];
      Binv[v][i]=s;
    }
}

// Control-authority schedule hook: coanda effectiveness scales with dynamic
// pressure / jet momentum.  Return normalized authority (1.0 = design point).
// Since B = authority*B0, Binv scales as 1/authority -> just divide the moment
// part below.  <<HOOK>> replace with clampf(k * qbar_or_mdot, AUTH_MIN, 1.0f).
static inline float alloc_authority() {
  return 1.0f;
}

// Largest s in [0,1] keeping base[v] + s*delta[v] within [0,1] for every valve.
// A pre-existing base violation forces s -> 0: a lower-priority axis must not
// paper over a higher-priority axis's saturation.
static float feasible_scale(const float base[VALVE_COUNT], const float delta[VALVE_COUNT]) {
  float s = 1.0f;
  for (int v = 0; v < VALVE_COUNT; ++v) {
    float b = base[v], d = delta[v];
    if (b > 1.0f + 1e-5f || b < -1.0f - 1e-5f) { s = 0.0f; continue; }   // was: b < -1e-5f
    if (d > 1e-6f) {                 // rising toward the upper limit (+1)
      if (b + d > 1.0f) s = fminf(s, (1.0f - b) / d);
    } else if (d < -1e-6f) {         // falling toward the lower limit (-1)
      if (b + d < -1.0f) s = fminf(s, (-1.0f - b) / d);                  // was: < 0.0f, (0.0f - b)/d
    }
  }
  return s < 0.0f ? 0.0f : s;
}

void allocation_update(uint32_t now) {
  ValveCmd& out = g_valve_pub.begin_write();

  if (g_status.source == SRC_SAFE || g_status.terminated) {
    // No live command source, or a latched termination: park at the safe pose.
    for (int v = 0; v < VALVE_COUNT; ++v) out.valve[v] = ALLOC_SAFE_POSE[v];
    g_alloc_s_rp = g_alloc_s_yaw = 1.0f;     // desaturation n/a in SAFE
  } else {
    const StickInput& in = (g_status.source == SRC_PRIMARY) ? g_primary_in : g_sbus_in;
    const float tau[3]     = { in.roll, in.pitch, in.yaw };   // [-1,1]
    const float inv_auth   = 1.0f / alloc_authority();

    // --- Yaw-priority sequential desaturation (PX4 sequential-desaturation style).
    //     Roll+pitch (attitude) are highest priority: preserved first and scaled
    //     TOGETHER so their ratio (the attitude command direction) is kept. Yaw is
    //     lowest priority and only fills remaining headroom. Collective is fixed.
    float cbase[VALVE_COUNT], rpdelta[VALVE_COUNT], ydelta[VALVE_COUNT];
    for (int v = 0; v < VALVE_COUNT; ++v) {
      cbase[v]   = u_trim[v];
      rpdelta[v] = inv_auth * (Binv[v][0]*tau[0] + Binv[v][1]*tau[1]);  // roll + pitch
      ydelta[v]  = inv_auth * (Binv[v][2]*tau[2]);                      // yaw
    }
    // Step 1: maximum attitude with yaw = 0 (roll & pitch scaled together).
    float s_rp = feasible_scale(cbase, rpdelta);
    // Step 2: give yaw whatever headroom is left once attitude is placed.
    float rpbase[VALVE_COUNT];
    for (int v = 0; v < VALVE_COUNT; ++v) rpbase[v] = u_trim[v] + s_rp*rpdelta[v];
    float s_yaw = feasible_scale(rpbase, ydelta);

    for (int v = 0; v < VALVE_COUNT; ++v) {
      float cmd = clampf(u_trim[v] + s_rp*rpdelta[v] + s_yaw*ydelta[v], -1.0f, 1.0f);
      out.valve[v] = clamp_valve(cmd * VALVE_POS_MAX);
    }  
  }
  for (int v = 0; v < VALVE_COUNT; ++v) g_valve_dbg[v] = out.valve[v];  // core-0 console copy
  out.stamp_ms = now;
  g_valve_pub.end_write();
  out.stamp_ms = now;                 // core 1 uses this for staleness -> failsafe
  g_valve_pub.end_write();
}