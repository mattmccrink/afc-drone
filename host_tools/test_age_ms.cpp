// Host test for age_ms() (rp2350_valve_node/types.h).
// Build + run from repo root:
//   g++ -std=c++17 -Wall -Wextra -I host_tools/stub -I rp2350_valve_node
//       host_tools/test_age_ms.cpp -o /tmp/tage && /tmp/tage
#include <cstdio>
#include "types.h"

static int fails = 0;
static void check(const char* n, bool ok) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", n); if (!ok) fails++; }

int main() {
  const uint32_t STALE = 100;   // TEENSY_TLM_STALE_MS
  printf("the straddle (2026-09-28): loop read now=1000, decode stamped millis()=1001\n");
  uint32_t now = 1000, stamp = 1001;
  check("old check: (now - stamp) > STALE fires on a fresh reply (the bug)", (now - stamp) > STALE);
  check("age_ms: stamp 1 ms ahead of now -> age 0 -> fresh", age_ms(now, stamp) == 0 && !(age_ms(now, stamp) > STALE));
  printf("ordinary ages\n");
  check("age 0", age_ms(5000, 5000) == 0);
  check("age 99 -> fresh", age_ms(5099, 5000) == 99);
  check("age 101 -> stale", age_ms(5101, 5000) > STALE);
  printf("millis() rollover (~49.7 days)\n");
  check("stamp just before wrap, now just after -> small age", age_ms(5u, 0xFFFFFFF0u) == 21);
  check("stamp just after wrap, now just before (stamp ahead) -> 0", age_ms(0xFFFFFFF0u, 5u) == 0);
  printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails ? 1 : 0;
}
