# AFC Bridge — Build & Bring-Up Guide (`rpi_ros2`)

> **CURRENT ARCHITECTURE: all-DDS (no MAVROS).** The dual-stack sections below
> are historical; see **Appendix B** for the current command path, DDS bring-up,
> and the #25873 keep-alive. Read Appendix B first.


Stand up the Pi ROS2 bridge between a PX4 flight controller and the RP2350
valve node. Written against **PX4 1.17 (control allocation)** with a
**dual-stack** command path. The bridge<->RP2350 half is **running on the Pi**
(Jammy/Humble/arm64). The command path (MAVROS + uXRCE-DDS to the Pixhawk) is
native-serial on the Pi; it has NO viable path on WSL (see notes) so do it here.

> Status placeholders to fill in once verified on your hardware are marked
> **[CONFIRM]**. Update them after a good run so this reads as fact, not guess.

---

## 1. Architecture (why it's built this way)

```
Pixhawk ──MAVLink──> MAVROS ─────────> /mavros/state ──────┐   (ARM path)
Pixhawk ──uORB──> uxrce_dds_client <=DDS=> MicroXRCEAgent ──┤
                       /fmu/out/vehicle_torque_setpoint ────┤   (CMD path)
                       /fmu/out/vehicle_thrust_setpoint ────┤
                                                            v
                                                    [ afc_bridge node ]
                                                            │ serial (USB-CDC)
                                                            v
                                    RP2350: allocation -> valves/servos/compressor
                                    RP2350 ──CTRL_TLM/SENSOR_TLM──> /afc/* + rosbag
```

- **Allocation stays on the RP2350.** PX4's effectiveness matrix can't model
  coanda-slot authority (nonlinear, flow-state-dependent, mass-flow-coupled),
  and the tiny owning the final valve command is what keeps SBUS reversion /
  fail-toward-airflow independent of the FC link. So PX4 supplies only the
  **pre-allocation torque/thrust demand**; the tiny allocates.
- **Dual-stack, deliberately.** On 1.17 the old `actuator_controls` /
  `ACTUATOR_CONTROL_TARGET` (and thus `/mavros/target_actuator_control`) is gone
  with the mixer. The normalised r/p/y/thrust demand now lives in
  `vehicle_torque_setpoint` + `vehicle_thrust_setpoint`, which are px4_msgs over
  uXRCE-DDS — not MAVROS. So: **MAVROS for arm, uXRCE-DDS for command.**

Field mapping into `FT_CMD` (all normalised [-1,1] body FRD):
`roll,pitch,yaw = vehicle_torque_setpoint.xyz[0:3]`;
`thrust = thrust_sign * vehicle_thrust_setpoint.xyz[2]`.

---

## 2. One-time setup

### 2.1 System dependencies
```bash
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-msgs \
                 python3-serial
# MAVROS geographic datasets (once) -- MAVROS won't start without them:
ros2 run mavros install_geographiclib_datasets.sh   # or the packaged script
# MicroXRCEAgent (the uXRCE-DDS agent): install via snap, apt (eProsima repo),
# or build Micro-XRCE-DDS-Agent from source. Confirm: MicroXRCEAgent --help
```

### 2.2 px4_msgs — **version match is critical**
The px4_msgs package MUST match the firmware's message definitions. Modern PX4
msgs carry a `MESSAGE_VERSION`; a mismatch = silent empty topics that look
exactly like "not publishing." Two ways, most-robust last:
```bash
cd ~/dev/afc-drone/rpi_ros2/src
# (a) matching release branch:
git clone -b release/1.17 https://github.com/PX4/px4_msgs.git    # matches PX4 1.17 (CONFIRMED)
# (b) MOST ROBUST: regenerate from YOUR firmware checkout, guaranteeing a match:
#   rm px4_msgs/msg/*.msg
#   cp ~/PX4-Autopilot/msg/*.msg           px4_msgs/msg/
#   cp ~/PX4-Autopilot/msg/versioned/*.msg px4_msgs/msg/   # if present on your ver
```

### 2.3 PX4 firmware — expose the setpoints outbound
The torque/thrust setpoints are registered `/fmu/in/` (offboard inject) by
default; to READ the controller's demand you add the `/fmu/out/` direction.
Edit `src/modules/uxrce_dds_client/dds_topics.yaml`, under `publications:`
```yaml
  - topic: /fmu/out/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint
  - topic: /fmu/out/vehicle_thrust_setpoint
    type: px4_msgs::msg::VehicleThrustSetpoint
```
Then rebuild/reflash (or rebuild SITL). Set `UXRCE_DDS_CFG` so the client points
at the agent's transport. CONFIRMED: PX4 1.17, transport = **serial** (USB/TELEM2);
this FC has no UDP DDS path. `uxrce_dds_client` module was disabled for flash on the
alpha/beta build -- re-enable via `make <target> boardconfig` (modules -> uxrce_dds_client).

---

## 3. Build the workspace
```bash
cd ~/dev/afc-drone/rpi_ros2
colcon build --packages-select px4_msgs                       # big; build first
colcon build --packages-select afc_bridge_msgs afc_bridge
source install/setup.bash
```
- Build **from `rpi_ros2/`** so `build/ install/ log/` land in the gitignored path.
- **After editing any `.msg`**, wipe the interface cache or the old fields linger:
  `rm -rf build install log && colcon build`.

---

## 4. Runtime bring-up

### 4.1 Serial device (the tiny)
```bash
ls -l /dev/serial/by-id/          # stable per-device symlink; ACMx numbers swap
```
Use the by-id path for `serial_port`. On WSL, `by-id` may be empty (thin udev) —
fall back to a udev rule pinning by USB vendor id (RP2350 = 2e8a):
```
# /etc/udev/rules.d/99-afc.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", SYMLINK+="tiny2350"
```

### 4.2 WSL topology that works (recommended)
Learned the hard way — this sidesteps usbip's composite-USB problem:
- **Tiny → usbipd** (simple single-CDC device, forwards cleanly):
  `usbipd list` → `usbipd bind --busid <x>` (admin, once) → `usbipd attach --wsl --busid <x>`.
  Re-attach after every physical replug; confirm it shows **Attached**, not just Shared.
- **Pixhawk → QGC → UDP** (do NOT usbip the FC — its composite device forwards
  only `-if00`, the console, not `-if02`, the MAVLink stream). Instead let QGC on
  Windows own the FC and forward MAVLink over UDP into WSL:
  - QGC → Application Settings → MAVLink → enable forwarding.
  - **Forward to the WSL2 IP, not localhost** (WSL2 is NAT'd behind Windows):
    `ip addr show eth0` in WSL → set QGC's forward target to `<wsl-eth0-ip>:14550`.
    **[CONFIRM]** port (default 14550; this session used 14450).
  - Allow the inbound UDP through Windows Firewall if it's dropped.

### 4.3 uXRCE-DDS agent
```bash
MicroXRCEAgent udp4 -p 8888        # match PX4 uxrce_dds_client transport/port
```

### 4.4 Launch everything
```bash
ros2 launch afc_bridge afc_system.launch.py \
     fcu_url:=udp://:14550@ \
     serial_port:=/dev/serial/by-id/usb-...RP2350... \
     start_agent:=true
```
`udp://:14550@` = MAVROS binds locally and learns the remote from the first
packet. `start_agent:=true` launches MicroXRCEAgent from the launch file; omit
if you run it yourself.

---

## 5. Verification checklist (in order)
```bash
# ARM path (MAVROS):
ros2 topic echo /mavros/state                 # connected: true, armed reflects switch

# CMD path (DDS) -- agent + client linked:
ros2 topic hz /fmu/out/vehicle_torque_setpoint # ~250 Hz    [CONFIRM]
ros2 topic echo /fmu/out/vehicle_thrust_setpoint  # check xyz[2] SIGN, armed in a rate mode

# Bridge output + link health:
ros2 topic echo /afc/health                    # ser=up, ctrl/sens ~25Hz, crc=0,
                                               # src=PRIMARY, arm_fresh, arm_counter climbing
ros2 topic echo /afc/ctrl_tlm                  # valve[] tracks commanded roll
```
- **thrust_sign [CONFIRM]:** with the vehicle commanding climb, is
  `vehicle_thrust_setpoint.xyz[2]` negative? If yes, `thrust_sign=-1.0` (default)
  gives positive throttle. If your build signs it the other way, flip the param.
- **Zeros on the bench are expected:** torque setpoint is the rate-controller
  output — near-zero until armed in a mode that runs it. Not a bug.
- **Arm fail-safe:** stop the state stream (or `fake_mavros.py --drop-arm-after 8`)
  and watch `/afc/health`: `arm_fresh`→false immediately, `arm_counter` freezes,
  `last_armed`→false ~10 s later (powered-reversion window).

---

## 6. Troubleshooting — symptom → cause → fix
(Every row below cost time this session; read before re-debugging.)

| Symptom | Cause | Fix |
|---|---|---|
| `echo` fails: "message type invalid", but `topic list` shows it | Terminal didn't source the workspace overlay | `source install/setup.bash`; `ros2 interface show <type>` is the litmus |
| MAVROS "device opened" but no messages | Port opened, no HEARTBEAT (not the FC, or wrong device) | `ros2 topic echo /mavros/state` → `connected` flag; use by-id |
| QGC data not reaching WSL | WSL2 is NAT'd; `localhost` ≠ Windows | QGC forward to WSL `eth0` IP; open Windows firewall |
| Pixhawk over usbip shows only `-if00` | Composite USB; MAVLink is on `-if02`, not forwarded | Don't usbip the FC — use QGC→UDP forward instead |
| `cat /dev/ttyACM*` shows nothing though device is alive | `cat` doesn't raise DTR / set raw termios (usbip-sensitive) | Probe with pyserial (raises DTR); `cat` is not a valid test here |
| ~1 s of data then silence | Passive reader / console iface / no heartbeat sent back | Drive with MAVROS (sends heartbeats); close Windows apps holding the port |
| `/mavros/target_actuator_control` silent (1.17) | Control allocator replaced the mixer/actuator_controls | Use `vehicle_torque/thrust_setpoint` via DDS |
| `/fmu/out/vehicle_*_setpoint` absent | Registered `/fmu/in/` only by default | Add `/fmu/out/` to `dds_topics.yaml` + rebuild firmware |
| DDS topic exists but `echo` empty / errors | px4_msgs version ≠ firmware, or reliable QoS | Match px4_msgs to firmware; subscribe best-effort (SENSOR_DATA) |
| Torque topic all zeros | Disarmed / not in a rate-controlled mode | Arm in the right mode; zeros on bench are normal |
| `/dev/ttyACM0/1` swap between the tiny and FC | Non-deterministic USB enumeration | by-id symlink or udev rule; here the FC is UDP so only the tiny is serial |
| Edited a `.msg`, node still sees old fields | colcon interface cache | `rm -rf build install log && colcon build` |
| `tests_require` UserWarning at build | Deprecated setuptools option | Delete the line in `setup.py` (cosmetic) |

---

## 7. Reference — topics & params

**Subscribes:** `/mavros/state` (arm), `/fmu/out/vehicle_torque_setpoint`,
`/fmu/out/vehicle_thrust_setpoint` (command).
**Publishes:** `/afc/ctrl_tlm` (`ValveNodeCtrl`), `/afc/sensor_tlm`
(`ValveNodeSensor`), `/afc/health` (`ValveNodeHealth`).

| Param | Default | Note |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | USB-CDC to the tiny; prefer a by-id path |
| `torque_topic` | `/fmu/out/vehicle_torque_setpoint` | r/p/y demand |
| `thrust_topic` | `/fmu/out/vehicle_thrust_setpoint` | thrust demand |
| `state_topic` | `/mavros/state` | arm source |
| `thrust_sign` | `-1.0` | FRD z→throttle sign; **[CONFIRM]** on hardware |
| `cmd_rate_hz` | `50.0` | CMD forward rate (setpoints arrive ~250 Hz; 5:1 decimation) |
| `cmd_stale_after_s` | `0.1` | stop forwarding if newest setpoint ages past this |
| `mdot_target_gps` | `40.0` | constant for now; future airspeed-driven CMU input |
| `arm_heartbeat_hz` | `1.0` | FT_ARM cadence (+ on-change) |
| `arm_stale_after_s` | `0.5` | stop advancing the arm counter if `/mavros/state` ages past this |
| `send_tlm_handshake` | `true` | send `tlm on\n` on connect (alpha boots in text mode) |

**Command freshness:** keyed on the NEWER of torque/thrust (both ride one DDS
client at 250 Hz; a one-topic stall is unlikely). For strict both-fresh, track
the two rx times and test the older — one-line change in `_cmd_fresh`.

---

## Appendix A — Raspberry Pi first-time bring-up (the gauntlet)

Reusable for any onboard Pi flashed/idle a while. These all bit us on a
datalogger Pi idle ~1.5 years; do them in order before the ROS2 install above.

**A.1 Get it online**
- Pi's single `wlan0` can't be hotspot (AP) + Wi-Fi client at once. Use Ethernet,
  or `sudo nmcli connection down Hotspot` → join Wi-Fi → bring hotspot back after.
- Direct PC↔Pi cable has no DHCP. Either plug the Pi into a router, or use Windows
  **ICS** (Internet Connection Sharing): PC becomes gateway `192.168.137.1`; set the
  Pi static `192.168.137.10/24`.
- **"Network is unreachable" after a static IP = no default route:**
  `sudo ip route add default via 192.168.137.1`, then DNS
  (`echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf`). Make both **permanent**
  or a reboot drops you offline mid-install:
  `sudo nmcli con modify <con> ipv4.gateway 192.168.137.1 ipv4.dns 8.8.8.8`.
- ICS is flaky: if the Pi routes but packets die at the PC, toggle sharing off/on
  on the Windows internet adapter.

**A.2 Fix apt (long-idle Pi)**
- **Expired ROS key** `EXPKEYSIG F42ED6FBAB17C654`: install the maintained
  `ros2-apt-source` package (self-updating key). If it errors
  **`Conflicting values ... Signed-By`**, an old `ros2.list` is still present —
  `sudo rm /etc/apt/sources.list.d/ros2*.list` (the stale one), keep the new.
- **`apt update` hangs after all `Hit:` lines** (not network):
  - lock held by `apt-check`/`unattended-upgrades` → let it finish, or
    `sudo systemctl stop unattended-upgrades` for the session;
  - IPv6 black-hole on NAT'd v4-only internet → `-o Acquire::ForceIPv4=true`;
  - `gpgv` blocked on **entropy** (headless Pi) → `sudo apt install haveged`.
  - A long `sudo apt upgrade` churning the backlog often clears all of the above.
- Then `sudo apt full-upgrade`, reboot.

**A.3 Check the Pi can host the stack**
```bash
uname -m          # aarch64 required (armv7l/32-bit = dead end for modern ROS2)
lsb_release -a    # distro -> ROS2 version; must match what you built against
free -h ; df -h / # RAM + disk for the px4_msgs build
```
Jammy = Humble (correct here). 32-bit needs a full 64-bit reflash first.

**A.4 Build gotchas (Pi-specific)**
- **px4_msgs build ~1 h and can OOM.** Heavy template C++ × hundreds of msgs; on
  limited RAM it swaps to SD and thrashes. `colcon build --parallel-workers 1` is
  often *faster* (fits RAM); add swap first if 2 GB
  (`sudo fallocate -l 2G /swapfile; sudo chmod 600 /swapfile; sudo mkswap /swapfile; sudo swapon /swapfile`).
  Install `ccache` for reruns. Build px4_msgs once; `--packages-skip px4_msgs` after.
- **colcon artifacts are NOT relocatable** — `install/` bakes absolute paths.
  Decide the workspace home first and build there; never move `build/ install/ log/`.
  `--symlink-install` lets Python (`afc_bridge`) edits skip rebuilds.
- **Source the overlay in every terminal** (`source install/setup.bash`) or msgs
  read as "invalid type"; put base-ROS-then-overlay in `~/.bashrc`.

**A.5 Why the FC command path is a Pi job, not WSL**
- MAVROS worked on WSL only via **QGC re-emitting MAVLink as UDP** into WSL (point
  QGC's forward at the **WSL eth0 IP**, not localhost — WSL2 is NAT'd — and open the
  firewall). QGC forwards MAVLink only.
- **uXRCE-DDS has no such forwarder**, and usbip forwarded only the Pixhawk's
  composite `-if00` (console), not `-if02` (data). So DDS is architecturally absent
  on WSL. On the Pi the Pixhawk is a native serial device and it's straightforward.
- Debug: `cat` is unreliable over usbip (no DTR/raw) — probe with **pyserial**.
  "~1 s then silence" = passive reader / console interface, not a dead port.

---

## Appendix B — CURRENT command path: all-DDS (supersedes the dual-stack sections above)

The dual-stack framing earlier in this doc (MAVROS for arm + DDS for command) was
superseded. **Everything is uXRCE-DDS now; MAVROS is dropped** from the bridge.
QGC / arming / mission planning live on a **separate telemetry radio** (MAVLink),
so the Pi↔FC link carries only DDS telemetry — one link, one middleware.

**Command + arm sources (all px4_msgs over uXRCE-DDS):**
- CMD = `vehicle_torque_setpoint` (xyz = roll/pitch/yaw) + `vehicle_thrust_setpoint`
  (`thrust = thrust_sign * xyz[2]`, default -1 for FRD-down — **confirm live**).
- ARM = `vehicle_status.arming_state == ARMING_STATE_ARMED` (strict; the tiny does
  no failsafe reasoning — the FC flips arming_state and it propagates).

**Transport:** serial, FTDI USB↔**TELEM2** (= `/dev/ttyS2` on Pixhawk 6C) @921600.
Not the FMU-USB (that defaults to MAVLink; MAVLink and DDS can't share a CDC).

**Firmware prereqs (one rebuild):**
1. `uxrce_dds_client` module enabled (`make <target> boardconfig` → modules) — was
   off for flash on the alpha/beta build.
2. `/fmu/out/vehicle_torque_setpoint` + `/fmu/out/vehicle_thrust_setpoint` added to
   `dds_topics.yaml` publications (they ship `/fmu/in/` only). `vehicle_status`
   publishes by default.
3. Keep `/fmu/in/offboard_control_mode` in subscriptions (needed by the keep-alive).
4. Recommended: the 2-line **#25873** patch to `uxrce_dds_client.cpp` (gate
   connectivity on TX alone) so the keep-alive becomes optional.

**Agent (Pi) — version MUST match the client:** PX4 1.17 client is Micro-XRCE
**v2.x** → build agent **v2.4.3**:
```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && git describe --tags   # confirm v2.4.3 (no --version flag exists)
mkdir build && cd build && cmake .. && make -j2 && sudo make install && sudo ldconfig /usr/local/lib/
```
A v3.x agent creates the client then never converges timesync.

**FTDI latency — pin to 1 ms** (16 ms default breaks timesync convergence + adds jitter):
```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # verify: reads 1
# persist: /etc/udev/rules.d/99-ftdi-low-latency.rules
#   ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

**Bring-up order (order matters):**
```bash
# 1. agent (owns the serial port) -- foreground while debugging:
MicroXRCEAgent serial --dev /dev/serial/by-id/usb-...TELEM2-FTDI... -b 921600
# 2. bridge node -- the keep-alive only fires with the node running:
ros2 run afc_bridge bridge_node --ros-args -p serial_port:=/dev/serial/by-id/usb-...RP2350...
# 3. verify:
nsh> uxrce_dds_client status        # Running, connected; Payload rx != 0 (keep-alive)
ros2 topic hz /fmu/out/vehicle_torque_setpoint   # stable ~25 Hz, max ~0.07s (NOT 1.0s)
ros2 topic echo /afc/health         # cmd_fresh, src=PRIMARY (armed), arm_counter climbing
```

**PX4 #25873 (the 1 Hz hiccup) — symptom & fix:**
- Symptom: `/fmu/out/*` rate halves with periodic 1.0 s max-latency spikes; client
  cycle `max ~1013000us`; `Payload rx: 0`.
- Cause: one-way ROS (subscribe-only) → client runs a blocking 1 s ping every
  second. `UXRCE_DDS_RX_TO=-1` does NOT stop it.
- Fix: publish any inert `/fmu/in/` topic → rx != 0 → ping suppressed. Bridge
  publishes `offboard_control_mode` @5 Hz (in-node). Confirmed: 13 Hz/1.0 s →
  25 Hz/0.07 s. Permanent fix = the 2-line firmware patch above.

**DDS troubleshooting (symptom → cause → fix):**

| Symptom | Cause | Fix |
|---|---|---|
| `uxrce_dds_client: command not found` (nsh) | module not compiled in | enable in `boardconfig`, rebuild |
| agent `create_client` but client `disconnected`/`timesync false` | one-way serial (dead FTDI-TX→TELEM2-RX or no common GND) | recheck that pair + ground; pyserial listen + `mavlink status` rx to prove each direction |
| stuck in timesync request/reply loop | FTDI 16 ms latency jitter | `latency_timer=1`; or disable PX4 timesync to connect (bridge carries own timebase) |
| connected but `/fmu/out/*` stalls ~1 Hz | PX4 #25873 (rx==0 ping) | keep-alive publisher (above) |
| topics listed but `echo` empty | px4_msgs≠firmware, or reliable QoS | match px4_msgs; subscribe best-effort (SENSOR_DATA) |
| `vehicle_status` present, setpoints absent | `/fmu/out/` torque/thrust not in `dds_topics.yaml` | add them + rebuild firmware |
| setpoints all zeros | disarmed / not in a rate mode | arm in rate mode; zeros disarmed are normal |
| wrong TELEM2 port | 6C TELEM2 = ttyS2 (not ttyS1) | `mavlink start -d /dev/ttyS2`; `mavlink status` prints the device |
