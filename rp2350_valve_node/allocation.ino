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
//
//  Traditional surfaces (pilot switch, "maximum effort"): when engaged, the SAME
//  demand is allocated over ALL actuators through the pseudo-inverse of the
//  combined matrix B_ALL = [B_EFF | B_SURF] (3x10). Same moment per unit demand
//  as AFC-only (PX4 loop gain unchanged), more total authority before
//  saturation. Handover on the switch is an immediate step. Disengaged: AFC-only
//  6x3 inverse, surfaces held at their configured center (command 0).
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

static const float B_SURF[3][SURF_COUNT] = B_SURF_INIT;   // surfaces (config.h)

#define ALLOC_N_ALL (VALVE_COUNT + SURF_COUNT)   // combined actuator count (10)

static float Binv[VALVE_COUNT][3];      // AFC-only pseudo-inverse (6x3), computed at boot
static float BinvAll[ALLOC_N_ALL][3];   // AFC+surfaces pseudo-inverse (10x3), computed at boot
static const int16_t ALLOC_SAFE_POSE[VALVE_COUNT] = DEFINED_SAFE_VALVE_POSE;
static const float u_trim[VALVE_COUNT] = VALVE_TRIM_NORM;

// Column k of the combined effectiveness matrix: valves 0..5, then surfaces.
static inline float b_all(int row, int k) {
  return (k < VALVE_COUNT) ? B_EFF[row][k] : B_SURF[row][k - VALVE_COUNT];
}

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

// Generic min-norm pseudo-inverse of a 3xN matrix given by column accessor col():
//   out (Nx3) = B^T (B B^T)^-1.  Returns false (and zeros out) if B B^T is singular.
static bool pinv3xN(int n, float (*col)(int, int), float out[][3]) {
  float BBt[3][3];
  for (int i=0;i<3;i++) for (int j=0;j<3;j++){
    float acc=0; for (int k=0;k<n;k++) acc+=col(i,k)*col(j,k);
    BBt[i][j]=acc;
  }
  float inv[3][3];
  if (!inv3(BBt, inv)) {                 // singular B -> zero authority (safe)
    for (int v=0;v<n;v++) for(int i=0;i<3;i++) out[v][i]=0.0f;
    return false;
  }
  for (int v=0;v<n;v++)
    for (int i=0;i<3;i++){
      float acc=0; for (int k=0;k<3;k++) acc+=col(k,v)*inv[k][i];
      out[v][i]=acc;
    }
  return true;
}
static float b_eff_col(int row, int k) { return B_EFF[row][k]; }

// Compute both inverses once at boot.
void allocation_setup() {
  if (!pinv3xN(VALVE_COUNT, b_eff_col, Binv))
    Serial.println(F("[alloc] WARNING: B B^T singular; check B_EFF"));
  if (!pinv3xN(ALLOC_N_ALL, b_all, BinvAll))
    Serial.println(F("[alloc] WARNING: B_ALL B_ALL^T singular; check B_SURF"));
}

// Control-authority schedule hook: coanda effectiveness scales with dynamic
// pressure / jet momentum.  Return normalized authority (1.0 = design point).
// Since B = authority*B0, Binv scales as 1/authority -> just divide the moment
// part below.  <<HOOK>> replace with clampf(k * qbar_or_mdot, AUTH_MIN, 1.0f).
static inline float alloc_authority() {
  return 1.0f;
}

// Largest s in [0,1] keeping base[v] + s*delta[v] within [-1,1] for all n actuators.
// A pre-existing base violation forces s -> 0: a lower-priority axis must not
// paper over a higher-priority axis's saturation.
static float feasible_scale(int n, const float* base, const float* delta) {
  float s = 1.0f;
  for (int v = 0; v < n; ++v) {
    float b = base[v], d = delta[v];
    if (b > 1.0f + 1e-5f || b < -1.0f - 1e-5f) { s = 0.0f; continue; }
    if (d > 1e-6f) {                 // rising toward the upper limit (+1)
      if (b + d > 1.0f) s = fminf(s, (1.0f - b) / d);
    } else if (d < -1e-6f) {         // falling toward the lower limit (-1)
      if (b + d < -1.0f) s = fminf(s, (-1.0f - b) / d);
    }
  }
  return s < 0.0f ? 0.0f : s;
}

void allocation_update(uint32_t now) {
  ValveCmd out;                       // compute fully into a local, then publish
                                      // (keeps the seqlock write window tiny)
  bool surf_active = false;

  if (g_status.source == SRC_SAFE || g_status.terminated) {
    // No live command source, or a latched termination: park at the safe pose,
    // surfaces at their configured center.
    for (int v = 0; v < VALVE_COUNT; ++v) out.valve[v] = ALLOC_SAFE_POSE[v];
    for (int k = 0; k < SURF_COUNT;  ++k) out.surf[k]  = 0;
    g_alloc_s_rp = g_alloc_s_yaw = 1.0f;     // desaturation n/a in SAFE
  } else {
    const StickInput& in = (g_status.source == SRC_PRIMARY) ? g_primary_in : g_sbus_in;
    const float tau[3]     = { in.roll, in.pitch, in.yaw };   // [-1,1]
    const float inv_auth   = 1.0f / alloc_authority();

    surf_active = g_status.surf_active;    // == surf_engaged here (live source, not terminated)
    const int n = surf_active ? ALLOC_N_ALL : VALVE_COUNT;   // actuators in play
    const float (*Bi)[3] = surf_active ? (const float (*)[3])BinvAll
                                       : (const float (*)[3])Binv;

    // --- Yaw-priority sequential desaturation (PX4 sequential-desaturation style).
    //     Roll+pitch (attitude) are highest priority: preserved first and scaled
    //     TOGETHER so their ratio (the attitude command direction) is kept. Yaw is
    //     lowest priority and only fills remaining headroom. Trim is fixed.
    float cbase[ALLOC_N_ALL], rpdelta[ALLOC_N_ALL], ydelta[ALLOC_N_ALL];
    for (int k = 0; k < n; ++k) {
      cbase[k]   = (k < VALVE_COUNT) ? u_trim[k] : 0.0f;               // surfaces trim = center
      rpdelta[k] = inv_auth * (Bi[k][0]*tau[0] + Bi[k][1]*tau[1]);    // roll + pitch
      ydelta[k]  = inv_auth * (Bi[k][2]*tau[2]);                      // yaw
    }
    // Step 1: maximum attitude with yaw = 0 (roll & pitch scaled together).
    float s_rp = feasible_scale(n, cbase, rpdelta);
    // Step 2: give yaw whatever headroom is left once attitude is placed.
    float rpbase[ALLOC_N_ALL];
    for (int k = 0; k < n; ++k) rpbase[k] = cbase[k] + s_rp*rpdelta[k];
    float s_yaw = feasible_scale(n, rpbase, ydelta);
    g_alloc_s_rp = s_rp; g_alloc_s_yaw = s_yaw;

    for (int k = 0; k < n; ++k) {
      float cmd = clampf(cbase[k] + s_rp*rpdelta[k] + s_yaw*ydelta[k], -1.0f, 1.0f);
      if (k < VALVE_COUNT) out.valve[k] = clamp_valve(cmd * VALVE_POS_MAX);
      else                 out.surf[k - VALVE_COUNT] = clamp_valve(cmd * VALVE_POS_MAX);
    }
    if (!surf_active) for (int k = 0; k < SURF_COUNT; ++k) out.surf[k] = 0;   // centered
  }
  for (int v = 0; v < VALVE_COUNT; ++v) g_valve_dbg[v] = out.valve[v];  // core-0 console copy
  for (int k = 0; k < SURF_COUNT;  ++k) g_surf_dbg[k]  = out.surf[k];
  out.stamp_ms = now;                 // core 1 uses this for staleness -> failsafe

  // Publish in one short odd-seq window (single struct copy), so core 1's
  // snapshot() almost never catches the buffer mid-write -> no servo neutral-bounce.
  ValveCmd& dst = g_valve_pub.begin_write();
  dst = out;
  g_valve_pub.end_write();
}
