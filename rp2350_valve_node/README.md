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
- **Core 0** — USB comms, source arbitration (PRIMARY → SBUS → DEFINED-SAFE with
  hysteresis), Pixhawk-owned arm-token enforcement, control allocation, the
  mass-flow → rpm outer loop with the compiled-in 30 k fallback, the teensyshot
  `Host_comm` stream to the Motor Teensy, the watchdog, and the RGB status LED.
- The synthetic pressures **respond to the commanded rpm**, so the mass-flow PI
  loop actually closes on the bench — you can watch it converge.

Simulated (flip to real later via the `USE_REAL_*` switches in `config.h`):
sensors, servos/PCA9685, and SBUS (its inverted-8E2 PIO decoder is documented as
a stub in `sbus_sim.ino`). The Teensy stream is emitted for real on `Serial2`.

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

## Wiring

Alpha needs **nothing** attached. Optional:

- **Motor Teensy** on `Serial2`: node **GP4 (TX) → Teensy RX**, node
  **GP5 (RX) ← Teensy TX**, common ground. (The Teensy must be adapted to read
  `Host_comm` from a UART rather than USB — that's the compressor project's open
  item; the byte format is already the one it understands.)
- Real sensors/servos later land on the **Qwiic/STEMMA header** = the single
  merged I2C0 bus (**SDA GP12 / SCL GP13**).

---

## Console commands

```
help                         list commands
status                       one-shot status line
mon on|off                   periodic status line (2 Hz)
tlm on|off                   binary telemetry mode (RX+TX switch to frames)
src auto|primary|sbus|safe   force / release the active source
arm | disarm                 simulated Pixhawk arm channel
prim on|off                  simulate primary (Pi) presence
sbusfs on|off                simulate SBUS failsafe flag
sbuslost on|off              simulate SBUS frame-lost flag
stick R P Y T                set SBUS sticks (floats: roll pitch yaw throttle)
fault valve N on|off         inject per-valve sensor fault (N = 0..5)
fault agg on|off             inject aggregate flow fault
mdot X                       set mass-flow target (g/s)
save                         persist current calibration to LittleFS
calshow                      show calibration source + servo0 coeffs
```

The onboard **BOOT/USER button (GP23)** toggles simulated primary presence — a
one-press way to force a PRIMARY → SBUS/SAFE reversion.

### Things to try

- `mon on` then watch `src=PRIMARY`, `comp=TRACK`, and `mdot` converge to target.
- `mdot 80` — see rpm climb as the PI loop chases the new target.
- Press the button (or `prim off`) — after the hysteresis window the source
  drops to SBUS (or SAFE if SBUS isn't driven); LED changes accordingly.
- `arm` / `disarm` — the compressor stream starts/stops (disarm = stream ceases,
  which is exactly how the Teensy dead-man stops the motor).
- `fault agg on` — aggregate flow goes untrustworthy; `comp` flips to `FALLBACK`
  and rpm pins to 30 000 (fails toward airflow). `fault agg off` recovers.
- `fault valve 2 on` — drops one valve out of the flow sum; watch `nvalid`.

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

## Where the open decisions live (all seeded with marked placeholders)

| # | Decision | File / symbol |
|---|----------|---------------|
| 1 | Mux channel map & PCA9685 addresses | `config.h` — `ADDR_*` |
| 2 | Curve-fit form (cubic) | `cal_littlefs.ino` — `cal_apply_defaults()`, `servos.ino` — `curve_us()` |
| 3 | OSR 1024 | `config.h` — `MS5837_OSR` |
| 4 | Both-sources-lost pose & compressor | `config.h` — `DEFINED_SAFE_VALVE_POSE`; `compressor.ino` (fallback) |
| 5 | SBUS throttle → **direct rpm** | `compressor.ino` (SRC_SBUS branch), `config.h` — `SBUS_RPM_*` |

`DEFINED_SAFE_VALVE_POSE` is a **neutral placeholder** — replace with the
aero-correct airflow-preserving pose before any real flow. The mixing matrix in
`allocation.ino` is likewise a placeholder.

---

## Next steps toward hardware

1. Set `USE_REAL_I2C 1` and implement the MS5837/PCA9545/PCA9685 transactions
   behind the existing `sensors_tick` / `pca9685_write_us` seams.
2. Implement the PIO SBUS decoder (`sbus_sim.ino` bottom) and set
   `USE_REAL_SBUS 1`.
3. Replace placeholder mixing matrix, curve-fit coefficients, and the
   defined-safe pose with calibrated values; `save` them to LittleFS.
4. Confirm the Teensy accepts `Host_comm` over UART and tune the mass-flow PI
   under real compressor load.
