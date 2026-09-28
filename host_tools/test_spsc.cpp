// Host test for SpscPublisher (rp2350_valve_node/types.h): core1 -> core0
// SensorFrame hand-off. Two threads stand in for the two cores.
// Build + run from repo root:
//   g++ -std=c++17 -O2 -Wall -Wextra -pthread -I host_tools/stub -I rp2350_valve_node
//       host_tools/test_spsc.cpp -o /tmp/tspsc && /tmp/tspsc
//
// Reproduces the 2026-09-28 bench fault: the writer computed the whole frame
// INSIDE the odd-seq window, a tight-polling reader exhausted its retries, and
// the caller treated the miss as "sensors stale". Then checks the fix: build
// the frame outside the window + publish() one copy, read via read_latest().
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include "types.h"

static int fails = 0;
static void check(const char* n, bool ok) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", n); if (!ok) fails++; }

// A frame whose fields are all derived from one counter, so a torn read shows up.
static void fill(SensorFrame& f, uint32_t k) {
  for (int v = 0; v < VALVE_COUNT; ++v) {
    f.p_up[v] = (float)k; f.p_lo[v] = (float)k + 1; f.t_die[v] = (float)k + 2;
    f.t_lo[v] = (float)k + 3; f.mdot[v] = (float)k + 4; f.valid[v] = 1; f.why[v] = 0;
  }
  f.mdot_total = (float)k; f.n_valid = VALVE_COUNT;
  for (int s = 0; s < SERVO_COUNT; ++s) f.servo_us[s] = (uint16_t)k;
}
static bool consistent(const SensorFrame& f) {
  float k = f.mdot_total;
  for (int v = 0; v < VALVE_COUNT; ++v)
    if (f.p_up[v] != k || f.p_lo[v] != k + 1 || f.t_die[v] != k + 2 || f.t_lo[v] != k + 3 || f.mdot[v] != k + 4)
      return false;
  for (int s = 0; s < SERVO_COUNT; ++s) if (f.servo_us[s] != (uint16_t)k) return false;
  return true;
}
// ~tens of microseconds of "venturi math"
static float burn(int n) { volatile float x = 1.0f; for (int i = 0; i < n; ++i) x = x * 1.0000001f + 0.5f; return x; }

struct Result { uint64_t reads = 0, misses = 0, torn = 0, no_frame = 0; };

// OLD writer: seq odd for the whole computation (what sensors*.ino did).
static void writer_old(SpscPublisher<SensorFrame>& p, std::atomic<bool>& run) {
  uint32_t k = 1;
  while (run) {
    p.seq++; __sync_synchronize();
    burn(20000);                               // compute inside the window
    fill(p.data, k++);
    __sync_synchronize(); p.seq++;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
// NEW writer: compute outside, publish() one copy.
static void writer_new(SpscPublisher<SensorFrame>& p, std::atomic<bool>& run) {
  uint32_t k = 1;
  while (run) {
    SensorFrame f{};
    burn(20000);
    fill(f, k++);
    p.publish(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

template <typename W, typename R>
static Result race(W writer, R reader, int ms) {
  SpscPublisher<SensorFrame> p;
  SensorFrame f0{}; fill(f0, 0); p.publish(f0);            // a first frame exists
  std::atomic<bool> run{true};
  std::thread w([&] { writer(p, run); });
  Result r;
  auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (std::chrono::steady_clock::now() < t_end) reader(p, r);   // tight poll, like core 0
  run = false; w.join();
  return r;
}

int main() {
  printf("old pattern (compute inside the odd window) + snapshot() as callers used it\n");
  Result a = race(writer_old, [](SpscPublisher<SensorFrame>& p, Result& r) {
    SensorFrame f; r.reads++;
    if (!p.snapshot(f)) r.misses++;                 // compressor.ino: miss => sensor_stale
    else if (!consistent(f)) r.torn++;
  }, 1500);
  printf("    reads=%llu misses=%llu torn=%llu\n", (unsigned long long)a.reads,
         (unsigned long long)a.misses, (unsigned long long)a.torn);
  check("reproduces the fault: a tight poller misses reads (would flag STALE)", a.misses > 0);
  check("seqlock itself never returns a torn frame", a.torn == 0);

  printf("fix: publish() one copy + read_latest()\n");
  Result b = race(writer_new, [](SpscPublisher<SensorFrame>& p, Result& r) {
    SensorFrame f; bool fresh = false; r.reads++;
    if (!p.read_latest(f, &fresh)) r.no_frame++;
    if (!fresh) r.misses++;
    if (!consistent(f)) r.torn++;
  }, 1500);
  printf("    reads=%llu contended=%llu no_frame=%llu torn=%llu\n", (unsigned long long)b.reads,
         (unsigned long long)b.misses, (unsigned long long)b.no_frame, (unsigned long long)b.torn);
  check("read_latest never reports 'no frame' once one exists", b.no_frame == 0);
  check("every frame handed to callers is consistent (incl. cached ones)", b.torn == 0);
  check("contention is rare with the short window (< 1e-4 of reads)",
        b.misses * 10000 < b.reads);

  printf("read_latest before the first frame\n");
  SpscPublisher<SensorFrame> q; SensorFrame f;
  check("returns false until the writer has published once", !q.read_latest(f));

  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
