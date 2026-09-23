# AFC Pi ROS2 Bridge — `afc_bridge`

Supervisor/bridge node between the Pixhawk (uXRCE-DDS, `px4_msgs`) and the RP2350
valve/sensor node. Forwards pre-allocation torque/thrust demand + the arm token to
the Tiny, and republishes the Tiny's telemetry as typed messages for rosbag and the
browser dashboard. The Pi is a supervisor/telemetry bridge — never in the
compressor loop.

This is `rpi_ros2/` in the monorepo. Two packages:

|Package|Type|Contents|
|-|-|-|
|`afc_bridge_msgs`|`ament_cmake`|`ValveNodeCtrl`, `ValveNodeSensor`, `ValveNodeComp`, `ValveNodeHealth`|
|`afc_bridge`|`ament_python`|the node + `framing.py` / `arm_token.py`, launch files, tools|

See [docs/BRINGUP.md](docs/BRINGUP.md) for first-time build + FC bring-up and
[docs/RUNBOOK.md](docs/RUNBOOK.md) for every-time operation.

## Ports (Pi 4) — the Tiny is NEVER `ttyAMA0`

|Link|Pi device|Pins|Baud|Owner|
|-|-|-|-|-|
|Pixhawk TELEM2 (uXRCE-DDS)|`/dev/serial0` → `ttyAMA0`|GPIO14/15|921600|MicroXRCEAgent|
|Tiny `Serial2`|`uart3` → `/dev/ttyAMA1` (base `fe201600`)|GPIO4/5|230400|`bridge_node`|

`/boot/firmware/config.txt`: `enable_uart=1`, `dtoverlay=disable-bt`,
`dtoverlay=uart3` (spelled correctly). Confirm the mapping with
`dmesg | grep -iE 'ttyAMA'`. The bridge refuses to open the FC port
(`forbidden_ports`, plus the launch's `agent_dev`) and holds its own port
exclusively (flock + `TIOCEXCL`; root is exempt from `TIOCEXCL`).

## Build

```bash
cd ~/afc-drone/rpi_ros2
colcon build --packages-select afc_bridge_msgs afc_bridge
source install/setup.bash
```

After editing any `.msg`: `rm -rf build install log && colcon build` (interface cache).

## Run

```bash
# everything (agent + bridge + rosbridge), defaults are the flight ports:
ros2 launch afc_bridge afc_system.launch.py
# bridge only, with a correlation rosbag:
ros2 launch afc_bridge bridge.launch.py record:=true bag_uri:=afc_$(date +%F_%H%M)
```

Published: `/afc/ctrl_tlm` (`ValveNodeCtrl`), `/afc/sensor_tlm`
(`ValveNodeSensor`), `/afc/compressor` (`ValveNodeComp`, incl. `thermal_suspect`),
`/afc/health` (`ValveNodeHealth`, 1 Hz). The bag records those plus the FC
command and arm inputs.

## Key parameters

|Param|Default|Note|
|-|-|-|
|`serial_port`|`/dev/ttyAMA1`|Tiny on uart3|
|`forbidden_ports`|`[/dev/serial0, /dev/ttyAMA0]`|never opened by the bridge|
|`agent_port`|`""`|extra forbidden port (afc_system passes `agent_dev`)|
|`torque_topic`|`/fmu/out/vehicle_torque_setpoint`|r/p/y demand|
|`thrust_topic`|`/fmu/out/vehicle_thrust_setpoint`|thrust demand|
|`status_topic`|`/fmu/out/vehicle_status_v1`|arm source (PX4 1.16+ versioned name)|
|`thrust_sign`|`-1.0`|FRD z → throttle sign; confirm on hardware|
|`cmd_rate_hz`|`50.0`|CMD forward rate|
|`cmd_stale_after_s`|`0.1`|stop forwarding if the newest setpoint ages past this|
|`mdot_target_gps`|`40.0`|constant for now|
|`arm_heartbeat_hz`|`1.0`|FT_ARM cadence (+ on-change)|
|`arm_stale_after_s`|`0.5`|stop advancing the arm counter if vehicle_status ages past this|

The old `send_tlm_handshake` / `tlm on` handshake is gone: the Tiny's `Serial2`
is always framed and its USB is a console only.

## Off-hardware wire test

```bash
cd src/afc_bridge && PYTHONPATH=. python3 tools/wire_check.py
```

Covers CMD/ARM out, CTRL_TLM (43/47/55 B), SENSOR_TLM, COMP_TLM (13/14 B) in,
magic resync, CRC-drop recovery, and the arm-token fresh / on-change / stale
behavior.

## Safety semantics (as built, Tiny side — see the tracker)

* **CMD forwarded only while the setpoints are fresh** — a stale primary stops
  the stream and the Tiny reverts to SBUS (or SAFE).
* **Arm counter advances only while vehicle_status is fresh** — never frozen and
  resent. On PRIMARY the Tiny obeys a *recent* token (advanced ≤ 1.5 s ago)
  immediately, with no seen-disarmed precondition. A token that stalls ≥ 10 s
  hands control to the pilot on SBUS if SBUS is usable; otherwise PRIMARY holds
  state. It never disarms by itself.
* **Total loss of both sources for 3 s → termination** (air off). An ARMED FC
  token that returns clears it at once; on SBUS the pilot must cycle the switch.
