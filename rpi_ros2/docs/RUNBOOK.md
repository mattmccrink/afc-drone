# AFC Drone — Cold-Start Runbook (Pi → rosbridge → PC dashboard)

Every-time operation, from a powered-off Pi to live data in the browser.
(First-time install/config is in BRINGUP.md; this assumes it's all built.)

Chain:
```
PX4 ──serial(TELEM2/FTDI)──> uXRCE agent ─┐
                                          ├─ ROS graph ─> rosbridge :9090 ─WiFi─> browser (PC)
tiny(RP2350) ──USB──> bridge node ────────┘
```

---

## 0. Power on — and check it
Power the Pi from a solid **5V/3A** supply (brownouts under load have bitten this
rig repeatedly). Once up and SSH'd in:
```bash
vcgencmd get_throttled        # want 0x0. nonzero = under-voltage; fix power first.
```

## 1. Get onto the Pi
Pick the link the PC will use to reach the Pi:
- **Flight-line / onboard (no router):** join the Pi's WiFi **hotspot**, then
  `ssh pi@10.42.0.1`  → dashboard URL later is `ws://10.42.0.1:9090`.
- **Bench over Ethernet/ICS:** `ssh pi@192.168.137.10`
  → dashboard URL later is `ws://192.168.137.10:9090`.

## 2. Confirm the two serial devices
FTDI (Pixhawk/TELEM2) enumerates as **ttyUSB\***, the tiny as **ttyACM\*** — they
don't collide, but use by-id names to be safe:
```bash
ls -l /dev/serial/by-id/
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer    # want 1 (udev rule = automatic)
```
If latency_timer isn't 1 (and you didn't add the udev rule), set it:
`echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`

## 3. Source the environment (every terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/afc-drone/rpi_ros2/install/setup.bash
```
(Put these two lines in ~/.bashrc so every new shell is ready.)

## 4. Start the uXRCE-DDS agent — Terminal A (owns the FTDI serial)
```bash
MicroXRCEAgent serial --dev /dev/serial/by-id/usb-...TELEM2-FTDI... -b 921600
```
Watch for `create_client` / `create_participant`. Leave it running.

## 5. Start the bridge node — Terminal B (opens the tiny; keep-alive fires here)
```bash
ros2 run afc_bridge bridge_node --ros-args \
     -p serial_port:=/dev/serial/by-id/usb-...RP2350...
```
The node auto-sends `tlm on\n` to the tiny and starts the #25873 keep-alive.
(Steps 4+5 in one: `ros2 launch afc_bridge afc_system.launch.py
serial_port:=<tiny> agent_dev:=<ftdi> start_agent:=true`.)

## 6. Start rosbridge — Terminal C (websocket :9090)
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Serves `ws://<pi-ip>:9090` on all interfaces. Can be started any time after the
graph is up (it just exposes existing topics).

## 7. Verify on the Pi (before touching the browser)
```bash
ros2 topic list                       # see /afc/* and /fmu/out/*
ros2 topic echo /afc/health           # serial up, ctrl/sensor ~25 Hz, crc 0
ros2 topic hz /fmu/out/vehicle_torque_setpoint   # steady, max ~0.07s (NOT 1.0s)
# on the FC (nsh): uxrce_dds_client status  -> connected, Payload rx != 0
```

## 8. Open the dashboard on the PC
- Open `afc_dashboard.html` in a browser (next to QGC).
- Set the URL box to the address from step 1 (`ws://10.42.0.1:9090` hotspot, or
  `ws://192.168.137.10:9090` ethernet) and click **Connect**.
- Link dot goes green; valve schematic, charts, and health populate.

---

## Shutdown
Ctrl-C terminals C → B → A (reverse order). The tiny reverts to SBUS/SAFE when
the bridge stops (by design).

## Quick troubleshooting
| Symptom | Check |
|---|---|
| Ethernet drops during heavy work | power/brownout — `vcgencmd get_throttled`; use `--parallel-workers 1` for builds |
| agent: no `create_client` | wrong `--dev`, baud, or MAVLink still on TELEM2 |
| `/fmu/out/*` stalls ~1 Hz | bridge node not running (keep-alive only fires with it up) |
| topics exist but dashboard empty | wrong ws URL/IP, or message-type strings in the HTML CFG don't match `ros2 topic type` |
| dashboard link dot stays red | rosbridge not running, port 9090 blocked, or PC not on the Pi's subnet |
| `crc errors` climbing in health | serial/baud/desync on the tiny link |
| valve fills look wrong-scale | set `valveAbsMax` / bar full-scales in the HTML CFG block |
