# RP2350 Valve/Sensor Node — ALPHA

Dual-core RP2350 firmware skeleton for the active-flow-control valve/sensor node,
targeting a **bare Pimoroni Tiny 2350**. The control logic is real; the
hardware-touching leaves (MS5837 sensors, PCA9685 servo drivers, SBUS receiver)
are **simulated** behind clean interfaces so the entire control flow, timing,
framing, arbitration, arm enforcement, and fail-toward-airflow behavior run and
are drivable on a board with nothing else attached.

This is the code-generation output of the architecture session; the design of
record is `rp2350_valve_node_architecture.md`.

---

## What runs in alpha

- **Core 1** — fixed 100 Hz I2C service tick: synthetic MS5837 pipeline
  (read-back-then-kick, D1 every tick, D2 every ~16 ticks), venturi mass-flow,
  per-valve validity guards, and curve-fit expansion (6 valves → 12 servo µs)
  with the sim PCA9685 write at 50 Hz.
- **Core 0** — Pi link (Serial2) + USB console, source arbitration (PRIMARY → SBUS → DEFINED-SAFE with
  hysteresis), Pixhawk-owned arm-token enforcement, control allocation, the
  mass-flow → rpm outer loop with the compiled-in 30 k fallback, the teensyshot
  `Host_comm` stream to the Motor Teensy, the watchdog, and the RGB status LED.
- The synthetic pressures **respond to the commanded rpm**, so the mass-flow PI
  loop actually closes on the bench — you can watch it converge.

Simulated (flip to real later via the `USE_REAL_*` switches in `config.h`):
sensors, servos/PCA9685, and SBUS (its inverted-8E2 PIO decoder is documented as
a stub in `sbus_sim.ino`). The Teensy stream is emitted for real on `Serial1`.

---

## Build & upload (Arduino IDE 2.x)

1. Install the **earlephilhower arduino-pico** core (Boards Manager → "Raspberry
   Pi Pico/RP2040/RP2350").
2. **Tools → Board:** *Pimoroni Tiny 2350* (or *Generic RP2350* if you prefer).
3. **Tools → Flash Size:** pick an option that includes a filesystem, e.g.
   **4MB (Sketch 3MB, FS 1MB)**. LittleFS needs an FS partition; without one
   `cal_load()` just falls back to compiled defaults (the node still runs).
4. First upload: hold **BOOT**, tap **RESET** (or plug USB) to enter UF2
   bootloader, then Upload. Subsequent uploads auto-reset.
5. Open **Serial Monitor at 115200**, line ending **Newline**, and type `help`.

All source files live in one sketch folder and compile as one program — the
`.ino` tabs share globals defined in `rp2350_valve_node.ino`; `config.h` and
`types.h` are the headers.

---

## Wiring (as built)

| Link | Tiny pins | Peer | Baud |
|---|---|---|---|
| Teensy compressor (`Serial1`) | GP0 TX → Teensy pin 15 (RX3); GP1 RX ← Teensy pin 14 (TX3) | teensyshot `Host_comm`/`ESCPID_comm` | 921600 |
| Pi (`Serial2`) | GP4 TX → Pi pin 29 (RXD3); GP5 RX ← Pi pin 7 (TXD3) | framed CMD/ARM in, CTRL/SENSOR/COMP out | 230400 |
| SBUS | GP6 (PIO, inverted) | receiver | 100000 |
| PCA9685 OE (all boards) | GP7, held LOW (enabled) permanently | — | — |
| I2C0 | SDA GP12 / SCL GP13 | muxes 0x74/0x75/0x77, PCA9685 0x44/0x48/0x50 | 400 kHz |

USB is a **console only** (no binary mode). Common ground on every link.

**Servo layout:** each PCA9685 (A/B/C) carries 4 valve servos on ch0–3 (valve
servo *s* → PCA *s*/4, ch *s*%4). Surfaces: A ch4 = L wing, B ch4 = R wing,
C ch4/ch5 = L/R canard. Center/throw/direction per surface in `config.h`.

---

## Console commands

```
help                         list commands
status                       one-shot status line (src/arm/elig, comp rpm + THERMAL, valves, surf)
health                       reprint boot/health summary
mon on|off                   periodic status line (2 Hz)
src auto|primary|sbus|safe   force / release the active source (never leave forced)
surf on|off|auto             force traditional surfaces / follow the SBUS switch (bench)
arm | disarm                 simulated Pixhawk arm channel (USE_REAL_PRIMARY=0 only)
prim on|off                  simulate primary (Pi) presence
sbusfs on|off / sbuslost on|off   simulate SBUS flags (sim SBUS only)
sbus                         print real SBUS decode (raw ch, arm, surf switch)
stick R P Y T                set SBUS sticks (sim SBUS only)
fault valve N on|off / fault agg on|off   inject sensor faults
mdot X                       set mass-flow target (g/s)
sens / pstream on|off        sensor dump / 2 Hz LabVIEW stream
servo|sweep|step ...         PCA9685 bring-up (bypasses the staleness failsafe)
save / zero / calshow        calibration store
```

The onboard **BOOT/USER button (GP23)** toggles simulated primary presence.

---

## Failsafe behavior (as built — decisions 2026-09-22)

- **Arm follows the live link.** PRIMARY: a recent FC token (advanced ≤ 1.5 s
  ago) governs immediately, whatever the reset history; ARMED also clears a
  termination. SBUS: pilot switch after a clean disarmed frame. A stalled FC
  token hands control to SBUS if usable, else PRIMARY holds.
- **Stop = silence to the Teensy** (no frames while disarmed/terminated) → DShot
  MOTOR_STOP → coast. Spin-up is shaped on the Teensy (~5 s to 30k).
- **Servos never go limp:** stale core-0 command (and boot) → defined-safe valve
  pose (0 through the curve fit) + centered surfaces.
- **Total loss 3 s → terminate** (air off); only if armed at least once.

---

## RGB status legend (active-low onboard LED)

| Color | Meaning |
|-------|---------|
| Green | source = PRIMARY |
| Blue  | source = SBUS (reversion) |
| Red   | source = DEFINED-SAFE |
| Yellow / Magenta | fallback active (red mixed into green/blue) |
| Slow blink | disarmed |
| Fast blink | sensor data stale |

---

## Where the open decisions live

| # | Decision | File / symbol | Status |
|---|----------|---------------|--------|
| 1 | Mux channel map & PCA9685 addresses | `config.h` — `ADDR_*`, `SENSOR_MAP` | verify on genuine parts |
| 2 | Curve-fit form (cubic) | `cal_littlefs.ino`, `servos.ino` — `curve_us()` | open |
| 3 | OSR | `config.h` — `MS5837_OSR` (8192) | set |
| 4 | Both-sources-lost pose | `config.h` — `DEFINED_SAFE_VALVE_POSE` = 0 (neutral via curve fit) | **decided 2026-09-22** |
| 5 | SBUS throttle mapping | removed (throttle commands nothing; fixed 30k while armed) | closed |

`B_EFF`, `B_SURF`, `VALVE_TRIM_NORM` and the surface centers are placeholders
pending tunnel / bench calibration.
