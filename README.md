# Active Flow Control Research Drone

Monorepo for an ~80 lb active-flow-control research UAV: a compressor feeds six
coanda slots that provide pitch / roll / yaw authority in place of conventional
control surfaces (which are retained as a reversion rung).

## Layout

| Path | What | Status |
|---|---|---|
| `rp2350_valve_node/` | RP2350 (Tiny 2350) dual-core valve/sensor node: MS5837 pressure sensing, PCA9685 servos, mass-flow to rpm outer loop, source arbitration, arm enforcement | Active |
| `teensy_escpid/` | Teensy compressor controller (teensyshot-based DShot ESC, closed-loop RPM) | Working at bench |
| `rpi_ros2/` | Raspberry Pi ROS2 hub: supervisor + telemetry bridge, Pixhawk arm-token relay | Not started |
| `host_tools/` | Python bench tools (control panels, serial test clients) | — |
| `docs/` | Architecture specs, handoffs, project tracker, datasheets | — |

## Build / dev notes

- Firmware folders are **Arduino sketches** (Arduino IDE 2.x; earlephilhower
  arduino-pico core for the RP2350, Teensyduino for the Teensy). Open the sketch
  *folder* in the IDE.
- Arduino requires each sketch folder to contain a `.ino` file **matching the
  folder name** (e.g. `rp2350_valve_node/rp2350_valve_node.ino`). See
  `teensy_escpid/README.md` for the one rename that implies.
- **Do not keep this repo inside a cloud-synced folder** (OneDrive / Dropbox /
  iCloud). Git's `.git/` database and live file-sync corrupt each other. Keep it
  on a plain local path such as `C:\dev\afc-drone\`; GitHub is your cloud copy
  once you push.

## Where to start

- System architecture: `docs/rp2350_valve_node_architecture.md`
- Current task state: `docs/project_tracker.md`
