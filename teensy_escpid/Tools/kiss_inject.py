#!/usr/bin/env python3
"""
kiss_inject.py -- impersonate the ESC's KISS telemetry line into the Teensy.

Bench test for the Teensy S17 patch (telemetry loss -> open loop -> hand-back)
with NO ESC attached. A 3.3 V USB-serial adapter drives Teensy pin 0 (Serial1
RX, the ESC telemetry input) with 10-byte KISS frames, exactly as teensyshot's
ESCCMD expects them:

    [0] temp C  [1:3] volt 0.01 V  [3:5] amp 0.01 A  [5:7] mAh  [7:9] eRPM/100  [9] CRC8
    (big-endian 16-bit fields, CRC-8 poly 0x07 init 0 over bytes 0..8)

ESCCMD turns the eRPM/100 field into rpm as field * 20 / ESCCMD_TLM_NB_POLES (4),
i.e. rpm = field * 50.

WIRING (Teensy 4 pins are 3.3 V ONLY -- set the adapter to 3.3 V logic):
    adapter TX  -> Teensy pin 0 (RX1)
    adapter GND -> Teensy GND
    (adapter RX unconnected; nothing on the DShot pin)

    pip install pyserial
    python kiss_inject.py COM7            # Windows (adapter's port)
    python kiss_inject.py /dev/ttyUSB0    # Linux / Pi

Then type commands at the prompt (the sender keeps running in the background):
    good [rpm]      steady valid frames (default 30000 rpm)
    stop            send nothing (telemetry wire 'cut')
    garbage [field] CRC-VALID frames with a nonsense eRPM field (default 0xFFFF),
                    one per second, nothing in between
    crcbad [rpm]    frames with a WRONG CRC at the normal rate
    noise           continuous random bytes
    sparse [hz]     valid frames at a low rate (default 3 Hz)
    once0           ONE valid frame at 0 rpm, then silence (spike just before loss)
    rate [hz]       frame rate for good/crcbad (default 250)
    status          show the current mode
    quit
"""
import os
import random
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pip install pyserial")

POLES = 4                     # must match ESCCMD_TLM_NB_POLES
BAUD = 115200                 # ESCCMD_TLM_UART_SPEED


def crc8(data: bytes) -> int:
    """ESCCMD_crc8: CRC-8, poly 0x07, init 0, MSB first."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def rpm_to_field(rpm: float) -> int:
    """Motor rpm -> KISS eRPM/100 field (ESCCMD: rpm10 = field * 20 / POLES)."""
    return max(0, min(0xFFFF, int(round(rpm * POLES / 200.0))))


def frame(rpm_field: int, temp=25, volt_cv=1200, amp_ca=100, mah=0, good_crc=True) -> bytes:
    body = bytes([
        temp & 0xFF,
        (volt_cv >> 8) & 0xFF, volt_cv & 0xFF,
        (amp_ca >> 8) & 0xFF, amp_ca & 0xFF,
        (mah >> 8) & 0xFF, mah & 0xFF,
        (rpm_field >> 8) & 0xFF, rpm_field & 0xFF,
    ])
    c = crc8(body)
    return body + bytes([c if good_crc else c ^ 0x5A])


class Injector:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, timeout=0, write_timeout=0.5)
        self.lock = threading.Lock()
        self.mode, self.arg = "stop", None
        self.rate = 250.0
        self.sent = 0
        self.running = True
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def set(self, mode, arg=None):
        with self.lock:
            self.mode, self.arg = mode, arg

    @staticmethod
    def _sleep_until(t_next):
        # coarse sleep, then a short spin for ~0.1 ms timing on Windows/Linux
        while True:
            dt = t_next - time.perf_counter()
            if dt <= 0:
                return
            if dt > 0.002:
                time.sleep(dt - 0.0015)

    def _run(self):
        t_next = time.perf_counter()
        last_garbage = 0.0
        while self.running:
            with self.lock:
                mode, arg, rate = self.mode, self.arg, self.rate
            now = time.perf_counter()
            if mode == "good":
                self.ser.write(frame(rpm_to_field(arg)))
                self.sent += 1
                t_next += 1.0 / rate
            elif mode == "crcbad":
                self.ser.write(frame(rpm_to_field(arg), good_crc=False))
                self.sent += 1
                t_next += 1.0 / rate
            elif mode == "sparse":
                self.ser.write(frame(rpm_to_field(30000)))
                self.sent += 1
                t_next += 1.0 / arg
            elif mode == "garbage":
                if now - last_garbage >= 1.0:
                    self.ser.write(frame(arg))
                    self.sent += 1
                    last_garbage = now
                t_next += 0.01
            elif mode == "noise":
                self.ser.write(os.urandom(64))           # ~5.6 ms of line time
                t_next += 64 * 10 / BAUD
            elif mode == "once0":
                self.ser.write(frame(0))
                self.sent += 1
                self.set("stop")
                t_next += 0.01
            else:                                         # stop
                t_next += 0.01
            if t_next < time.perf_counter() - 0.05:       # fell behind: resync
                t_next = time.perf_counter()
            self._sleep_until(t_next)

    def close(self):
        self.running = False
        self.t.join(timeout=1)
        self.ser.close()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    inj = Injector(sys.argv[1])
    print(f"open {sys.argv[1]} @ {BAUD}. mode=stop. type 'help' for commands.")
    try:
        while True:
            try:
                parts = input("kiss> ").split()
            except EOFError:
                break
            if not parts:
                continue
            cmd, rest = parts[0].lower(), parts[1:]
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd == "help":
                print(__doc__[__doc__.index("Then type"):])
            elif cmd == "good":
                rpm = float(rest[0]) if rest else 30000.0
                inj.set("good", rpm)
                print(f"good: {rpm:.0f} rpm (field {rpm_to_field(rpm)}) at {inj.rate:.0f} Hz")
            elif cmd == "stop":
                inj.set("stop"); print("stop: no frames")
            elif cmd == "garbage":
                field = int(rest[0], 0) if rest else 0xFFFF
                inj.set("garbage", field)
                print(f"garbage: CRC-valid frame, eRPM field 0x{field:04X}, 1 per second")
            elif cmd == "crcbad":
                rpm = float(rest[0]) if rest else 30000.0
                inj.set("crcbad", rpm); print("crcbad: wrong-CRC frames at the normal rate")
            elif cmd == "noise":
                inj.set("noise"); print("noise: continuous random bytes")
            elif cmd == "sparse":
                hz = float(rest[0]) if rest else 3.0
                inj.set("sparse", hz); print(f"sparse: valid 30000 rpm frames at {hz} Hz")
            elif cmd == "once0":
                inj.set("once0"); print("once0: one 0-rpm frame, then silence")
            elif cmd == "rate":
                inj.rate = float(rest[0]) if rest else 250.0; print(f"rate: {inj.rate:.0f} Hz")
            elif cmd == "status":
                print(f"mode={inj.mode} arg={inj.arg} rate={inj.rate:.0f} Hz sent={inj.sent}")
            else:
                print("? (help)")
    finally:
        inj.close()


if __name__ == "__main__":
    main()
