# AFC Pi ROS2 Bridge — `afc\_bridge`

Supervisor/bridge node between MAVROS and the RP2350 valve/sensor node over
USB-CDC. Forwards pre-mix control demand + the arm token to the tiny, and
republishes the tiny's `CTRL\_TLM`/`SENSOR\_TLM` as typed messages for rosbag.

This is `rpi\_ros2/` in the monorepo. Two packages:

|Package|Type|Contents|
|-|-|-|
|`afc\_bridge\_msgs`|`ament\_cmake`|`ValveNodeCtrl`, `ValveNodeSensor`|
|`afc\_bridge`|`ament\_python`|the node + `framing.py` / `arm\_token.py`|

See \[docs/BRINGUP.md](docs/BRINGUP.md) for full build + FC bring-up (all-DDS, #25873, WSL/Pi notes).

## Build

```bash
cd \~/ros2\_ws                       # your workspace
cp -r afc\_bridge\_msgs afc\_bridge src/   # or drop this src/ in place
colcon build --packages-select afc\_bridge\_msgs afc\_bridge
source install/setup.bash
```

`afc\_bridge\_msgs` builds first (the node depends on it). Runtime deps:
`rclpy`, `std\_msgs`, `mavros\_msgs`, `python3-serial`. Install the last two if
missing: `sudo apt install ros-$ROS\_DISTRO-mavros-msgs python3-serial`.

## Run

```bash
ros2 launch afc\_bridge bridge.launch.py serial\_port:=/dev/ttyACM0
# with a correlation rosbag:
ros2 launch afc\_bridge bridge.launch.py record:=true bag\_uri:=afc\_$(date +%F\_%H%M)
```

Or the node directly:

```bash
ros2 run afc\_bridge bridge\_node --ros-args -p serial\_port:=/dev/ttyACM0
```

Published: `/afc/ctrl\_tlm` (`ValveNodeCtrl`), `/afc/sensor\_tlm`
(`ValveNodeSensor`). The bag records those two plus the MAVROS command and arm
inputs, so post-flight you have both what was sent and what came back.

## Two things to confirm on the vehicle

1. **CMD topic.** Default is `mavros/target\_actuator\_control`
(`ActuatorControl`, `group\_mix 0`, `controls\[0:4] = roll,pitch,yaw,thrust`).
Confirm it's actually publishing on your PX4 build:
`ros2 topic echo /mavros/target\_actuator\_control`. If that message is dead on
a newer PX4, repoint with `-p cmd\_topic:=... -p cmd\_group\_mix:=...` — it's a
param, not a code change. `thrust\_lo/hi` default to `\[0,1]`; widen to
`\[-1,1]` for reversible thrust.
2. **Binary handshake.** The node sends `tlm on\\n` on connect to flip the tiny
into binary frame mode (alpha boots in text/console mode). If your flight
firmware boots straight into binary, set `-p send\_tlm\_handshake:=false`.

## Key parameters

|Param|Default|Note|
|-|-|-|
|`serial\_port`|`/dev/ttyACM0`|USB-CDC to the tiny|
|`cmd\_topic`|`/mavros/target\_actuator\_control`|pre-mix control demand|
|`cmd\_group\_mix`|`0`|PX4 flight-control group|
|`cmd\_rate\_hz`|`50.0`|CMD forward rate|
|`cmd\_stale\_after\_s`|`0.1`|stop forwarding if MAVROS control ages past this (→ tiny reverts to SBUS)|
|`mdot\_target\_gps`|`40.0`|constant for now; future airspeed-driven CMU input|
|`arm\_heartbeat\_hz`|`1.0`|FT\_ARM cadence (+ on-change)|
|`arm\_stale\_after\_s`|`0.5`|stop advancing the arm counter if `/mavros/state` ages past this|
|`send\_tlm\_handshake`|`true`|send `tlm on\\n` on connect|

## Off-hardware wire test

`framing.py` and `arm\_token.py` are ROS-free and unit-tested against a
byte-accurate mirror of the tiny (`parse\_byte`/`on\_frame`/`tlm\_service`):

```bash
cd afc\_bridge \&\& PYTHONPATH=. python3 test/test\_wire.py
```

Covers CMD/ARM out, CTRL\_TLM/SENSOR\_TLM in, magic resync, CRC-drop recovery,
and the arm-token fresh / on-change / stale→10 s-window→disarm behavior.

## Safety notes baked in

* **CMD forwarded only while the MAVROS control input is fresh** — a stale
primary stops the stream so the tiny's `USB\_CMD\_TIMEOUT\_MS` reverts to SBUS.
* **Arm counter advances only while `/mavros/state` is fresh** — never
frozen-and-resent through an FCU dropout; the tiny rides the 10 s
powered-reversion window then disarms.
* The tiny counts that window from the **last counter advance**, so at 1 Hz the
usable window after a dropout is 10 s minus up to one heartbeat. Bump
`arm\_heartbeat\_hz` to 2–5 if you ever want it tight to 10 s.

