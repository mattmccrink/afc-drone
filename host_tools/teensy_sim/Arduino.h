// Host mock of the bits of Arduino/Teensy that ESCPID.ino touches.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <deque>
extern uint32_t g_ms;
inline uint32_t millis() { return g_ms; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void noInterrupts() {}
inline void interrupts() {}
static uint32_t SCB_AIRCR_dummy; 
#define SCB_AIRCR SCB_AIRCR_dummy
struct MockUSB {
  void begin(int) {}
  void println(const char*) {}
  bool dtr() { return false; }
  int availableForWrite() { return 0; }
  template <class... A> void printf(const char*, A...) {}
};
struct MockUart {
  std::deque<uint8_t> rx; size_t tx_bytes = 0;
  void begin(int) {}
  void addMemoryForRead(void*, size_t) {}
  int available() { return (int)rx.size(); }
  int read() { int c = rx.front(); rx.pop_front(); return c; }
  void write(const uint8_t*, size_t n) { tx_bytes += n; }
};
extern MockUSB Serial;
extern MockUart Serial3;
