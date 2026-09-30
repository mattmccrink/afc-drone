# AFC bag recording — install (once, on the Pi)

Files go in the repo at `rpi_ros2/tools/`: `afc_record.sh`, `afc_rec`,
`afc-record@.service`.

```bash
cd ~/afc-drone/rpi_ros2/tools
chmod +x afc_record.sh afc_rec
sudo cp afc-record@.service /etc/systemd/system/
sudo systemctl daemon-reload
ln -sf ~/afc-drone/rpi_ros2/tools/afc_rec ~/.local/bin/afc_rec   # or add tools/ to PATH
mkdir -p ~/bags
```

If `~/.bashrc` sets `ROS_DOMAIN_ID` or `RMW_IMPLEMENTATION`, copy the same values
into the `Environment=` lines of the unit (then `daemon-reload`), or the recorder
won't see the graph.

Optional — no password prompt for start/stop (edit with `sudo visudo -f /etc/sudoers.d/afc-record`):

```
pi ALL=(root) NOPASSWD: /usr/bin/systemctl start afc-record@*, /usr/bin/systemctl stop afc-record@*
```

## Use

```bash
afc_rec start T-C5     # new bag ~/bags/afc_<date>_<time>_T-C5
afc_rec status         # recording? size, free disk
afc_rec stop           # clean close + `ros2 bag info`
afc_rec ls             # all bags with sizes
```

Recording survives the SSH session dropping (it runs under systemd, not your
shell). It never starts at boot. Start it before or after the stack — the
recorder picks the topics up when they appear.

Pull bags to the desktop: `scp -r pi@10.223.250.2:~/bags/afc_<name> .`

---

# Bridge + rosbridge as a boot service (S14)

The agent already runs as `microxrce-agent.service`. This adds the bridge and
rosbridge as a second, independent service, so the whole stack comes up at boot
and restarting the bridge can never cycle the agent.

```bash
cd ~/afc-drone/rpi_ros2/tools
chmod +x afc_bridge_run.sh
sudo cp afc-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now afc-bridge
journalctl -u afc-bridge -f          # expect "opened /dev/ttyAMA1 (exclusive)" and the 1 Hz [health] line
```

Same `Environment=` caveat as the recorder. To run the bridge by hand (debugging),
`sudo systemctl stop afc-bridge` first -- it holds the Tiny's UART exclusively.

Checks: reboot the Pi and confirm the dashboard connects with no manual steps;
`sudo systemctl restart afc-bridge` while armed recovers PRIMARY without an FC
reboot (repeat of T-C2); `systemctl status microxrce-agent` shows the agent's
uptime unchanged by bridge restarts.

---

# Pi configuration + preflight (2026-09-29)

`pi_setup.sh` makes the one-time Pi settings; `afc_preflight` is the go/no-go check.

```bash
cd ~/afc-drone/rpi_ros2/tools
chmod +x pi_setup.sh afc_preflight afc_preflight.py
ln -sf "$PWD/afc_preflight" ~/.local/bin/afc_preflight
sudo ln -sf "$PWD/pi_setup.sh" /usr/local/bin/pi_setup.sh   # /usr/local/bin: sudo does not search ~/.local/bin

pi_setup.sh status                    # read-only summary of what is set
sudo pi_setup.sh hostname              # 'phoenix' + mDNS -> ssh pi@phoenix.local
sudo pi_setup.sh wifi-off              # refuses unless eth0 is up and autoconnecting, and
                                       # (if you are on SSH) unless your session is on eth0; then reboot
```

**Read-only root (protects the SD card from hard power cuts).** With it on, every
write to the root filesystem goes to RAM and is discarded at power-off, so the
bags must live on a separate filesystem first; `ro-root on` refuses otherwise. It
uses the `overlayroot` package (install it once while the Pi has network:
`sudo apt install overlayroot`) with `recurse=0`, so the bag drive and `/boot/firmware`
stay real, writable filesystems. Not yet tried on this Pi: after the first reboot
check `findmnt -R /` shows `/` as `overlay` and the bag drive and `/boot/firmware` as
their own devices.

```bash
# 1. a USB drive for bags (ext4), mounted at ~/bags at every boot
sudo mkfs.ext4 -L PHXBAGS /dev/sda1          # CHECK the device name first: lsblk
echo 'LABEL=PHXBAGS /home/pi/bags ext4 defaults,noatime,nofail 0 2' | sudo tee -a /etc/fstab
sudo mount -a && sudo chown pi:pi /home/pi/bags
# 2. then
sudo pi_setup.sh ro-root on && sudo reboot
```
To update software later: `sudo pi_setup.sh ro-root off`, reboot, `git pull` /
`colcon build`, `ro-root on`, reboot.

**Clock.** The Pi has no real-time clock and no internet in the field, so its time
can be wrong, which scrambles bag names. Alignment with the PX4 log does not depend
on it (bags carry FC timestamps, and now `/fmu/out/vehicle_gps_position`, which maps
FC time to GPS UTC). To correct the Pi clock itself, with a GPS fix:

```bash
sudo systemctl stop afc-bridge; sudo pi_setup.sh time-from-gps; sudo systemctl start afc-bridge
```
It refuses while the bridge or a bag recording runs: stepping the clock under live
ROS nodes can stall timers that run on system time. The agent keeps running (it
carries the GPS topic).

**Preflight:** `afc_preflight` (add `--bench` on the bench, where sim builds, missing
venturis and no RC are WARN instead of FAIL). Exit status 0 = GO.
