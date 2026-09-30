#!/usr/bin/env bash
# pi_setup.sh -- one-time / occasional Pi configuration for the Phoenix onboard computer.
#
#   pi_setup.sh status               what is set right now (read-only; no sudo needed)
#   pi_setup.sh hostname [name]      hostname (default phoenix) + mDNS: ssh pi@phoenix.local
#   pi_setup.sh wifi-off | wifi-on   disable / re-enable the Wi-Fi radio (firmware overlay; reboot)
#   pi_setup.sh ro-root on|off       read-only root filesystem (overlayroot; reboot)
#   pi_setup.sh time-from-gps        set the Pi clock once from the FC's GPS (stack must be stopped)
#
# Run with sudo. Install the link in /usr/local/bin (sudo does not search ~/.local/bin).
#
# RECOVERY (locked out): every change here can be undone from the SD card's FAT boot
# partition on any laptop. Pull the card, open the boot partition, and
#   - Wi-Fi:      in config.txt delete the line  dtoverlay=disable-wifi
#   - read-only:  in cmdline.txt delete the word  overlayroot=tmpfs:recurse=0
#                 (cmdline.txt must stay ONE line)
# This script never write-protects the boot partition, so both files stay editable
# from the Pi too.
set -euo pipefail

BOOT=${PI_SETUP_BOOT:-/boot/firmware}      # override only for testing
[[ -d $BOOT ]] || BOOT=/boot
CONFIG=$BOOT/config.txt
CMDLINE=$BOOT/cmdline.txt
RUSER=${SUDO_USER:-pi}                     # the real user behind sudo
RHOME=$(getent passwd "$RUSER" | cut -d: -f6 || echo /home/pi)
BAG_DIR="${AFC_BAG_DIR:-$RHOME/bags}"
MARK="# phoenix: pi_setup.sh wifi-off"
OVL_TOKEN="overlayroot=tmpfs:recurse=0"    # recurse=0: separate mounts (bags, /boot) stay real

die()  { echo "pi_setup: $*" >&2; exit 1; }
need_root() { [[ $EUID -eq 0 ]] || die "run with sudo"; }
overlay_active() { [[ $(findmnt -n -o FSTYPE /) == overlay ]]; }
no_overlay() { overlay_active && die "root is a read-only overlay; changes would vanish at reboot. Run 'pi_setup.sh ro-root off', reboot, then retry." || true; }

# ---------------------------------------------------------------------------- #
eth_ip() {
  ip -4 -o addr show eth0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1
}

eth_ok() {
  # a wired IPv4 address, and NetworkManager set to bring that profile up at boot
  local ip con
  ip=$(eth_ip); [[ -n $ip ]] || return 1
  if command -v nmcli >/dev/null; then
    con=$(nmcli -g GENERAL.CONNECTION device show eth0 2>/dev/null | head -1)
    [[ -n $con && $con != "--" ]] || return 1
    [[ $(nmcli -g connection.autoconnect connection show "$con" | head -1) == yes ]] || return 1
  fi
  echo "$ip"
}

ssh_ifaces() {
  # interfaces carrying established inbound SSH sessions (works under sudo, which
  # drops $SSH_CONNECTION)
  local a
  for a in $(ss -Htn state established '( sport = :22 )' 2>/dev/null | awk '{print $3}' | sed 's/:22$//; s/^\[//; s/\]$//'); do
    ip -o addr show | awk -v a="$a" '{split($4,x,"/"); if (x[1]==a) print $2}'
  done | sort -u
}

# ---------------------------------------------------------------------------- #
cmd_status() {
  echo "hostname      : $(hostname)  (mDNS avahi-daemon: $(systemctl is-active avahi-daemon 2>/dev/null || true))"
  echo "eth0          : $(eth_ip)"
  if [[ -e /sys/class/net/wlan0 ]]; then echo "Wi-Fi         : present"; else echo "Wi-Fi         : off"; fi
  if grep -q '^dtoverlay=disable-wifi' "$CONFIG" 2>/dev/null; then echo "                (disable-wifi set in $CONFIG)"; fi
  if overlay_active; then echo "root fs       : read-only overlay"; else echo "root fs       : writable"; fi
  if grep -q 'overlayroot=' "$CMDLINE" 2>/dev/null; then echo "                ($(grep -o 'overlayroot=[^ ]*' "$CMDLINE") in cmdline.txt)"; fi
  echo "boot fs       : $(findmnt -n -o OPTIONS "$BOOT" 2>/dev/null | cut -d, -f1)"
  echo "bag dir       : $BAG_DIR -> $(findmnt -n -o SOURCE,FSTYPE --target "$BAG_DIR" 2>/dev/null || echo missing)"
  echo "clock (UTC)   : $(date -u '+%Y-%m-%d %H:%M:%S')  ntp-synced=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo ?)"
}

cmd_hostname() {
  need_root; no_overlay
  local name=${1:-phoenix}
  [[ $name =~ ^[a-z0-9-]+$ ]] || die "hostname must be lower-case letters, digits, '-'"
  # /etc/hosts first, so sudo never loses track of the host name
  if grep -q '^127\.0\.1\.1' /etc/hosts; then
    sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$name/" /etc/hosts
  else
    printf '127.0.1.1\t%s\n' "$name" >> /etc/hosts
  fi
  hostnamectl set-hostname "$name"
  command -v avahi-daemon >/dev/null || apt-get install -y avahi-daemon
  systemctl enable --now avahi-daemon
  systemctl restart avahi-daemon
  echo "hostname is now '$name'. From a laptop on the same network: ping $name.local / ssh pi@$name.local"
}

cmd_wifi_off() {
  need_root
  local ip ifs
  ip=$(eth_ok) || die "eth0 is not up with an autoconnecting NetworkManager profile -- fix Ethernet first, or you may be locked out"
  ifs=$(ssh_ifaces)
  if grep -qx wlan0 <<<"$ifs"; then die "an SSH session is over Wi-Fi; reconnect over Ethernet ($ip) and close the Wi-Fi session first"; fi
  if [[ -n $ifs ]] && ! grep -qx eth0 <<<"$ifs"; then die "no SSH session over eth0 -- prove Ethernet works (ssh pi@$ip) before turning Wi-Fi off"; fi
  if grep -q '^dtoverlay=disable-wifi' "$CONFIG"; then echo "already set in $CONFIG"; return; fi
  cp "$CONFIG" "$CONFIG.phoenix-bak"
  # A trailing [all] resets any model-specific [section] the file ends in, so the
  # overlay applies to every board.
  printf '\n%s\n[all]\ndtoverlay=disable-wifi\n' "$MARK" >> "$CONFIG"
  echo "Wi-Fi off from the next boot. Ethernet: $ip (also $(hostname).local)."
  echo "Recovery without network: delete 'dtoverlay=disable-wifi' from config.txt on the SD card."
  echo "Reboot when ready:  sudo reboot"
}

cmd_wifi_on() {
  need_root
  cp "$CONFIG" "$CONFIG.phoenix-bak"
  # drop our three-line block (marker, [all], overlay), then any other copy of the line
  sed -i "/^${MARK//\//\\/}\$/{N;N;d}" "$CONFIG"
  sed -i '/^dtoverlay=disable-wifi/d' "$CONFIG"
  echo "Wi-Fi back on from the next boot (sudo reboot)."
}

cmd_ro_root() {
  need_root
  case "${1:-}" in
    on)
      overlay_active && { echo "already active"; return; }
      dpkg -s overlayroot >/dev/null 2>&1 || die "package 'overlayroot' not installed (needs network once): sudo apt install overlayroot"
      grep -q '^auto_initramfs=1' "$CONFIG" || die "config.txt lacks auto_initramfs=1 (Bookworm default); overlayroot needs the initramfs"
      # Bags written to the overlay would live in RAM and vanish at power-off.
      local bagfs
      bagfs=$(findmnt -n -o TARGET --target "$BAG_DIR" 2>/dev/null || echo /)
      [[ $bagfs != / ]] || die "$BAG_DIR is on the root filesystem; with a read-only root every bag would be lost at power-off.
Mount a USB drive (or separate partition) at $BAG_DIR first (fstab entry with nofail; see INSTALL.md), then re-run."
      cp "$CMDLINE" "$CMDLINE.phoenix-bak"
      sed -i 's/ *overlayroot=[^ ]*//g; s/$/ '"$OVL_TOKEN"'/' "$CMDLINE"
      grep -q "$OVL_TOKEN" "$CMDLINE" || die "failed to write $CMDLINE (restored from .phoenix-bak)"
      [[ $(wc -l < "$CMDLINE") -le 1 ]] || { cp "$CMDLINE.phoenix-bak" "$CMDLINE"; die "cmdline.txt is not one line; left unchanged"; }
      echo "Read-only root from the next boot (sudo reboot). Bags stay on $bagfs; /boot stays writable."
      echo "After the reboot, check:  findmnt -R /   (/ = overlay, $bagfs and $BOOT = real devices)"
      echo "To update software: sudo pi_setup.sh ro-root off, reboot, update, ro-root on, reboot."
      ;;
    off)
      cp "$CMDLINE" "$CMDLINE.phoenix-bak"
      sed -i 's/ *overlayroot=[^ ]*//g' "$CMDLINE"
      echo "Read-only root off from the next boot (sudo reboot)."
      ;;
    *) die "usage: pi_setup.sh ro-root on|off" ;;
  esac
}

cmd_time_from_gps() {
  need_root
  # Stepping the clock under live ROS nodes can stall timers that run on system time,
  # and puts a jump in any bag being recorded. Refuse unless the stack is down.
  systemctl is-active --quiet afc-bridge && die "stop the bridge first: sudo systemctl stop afc-bridge"
  pgrep -f 'bridge_node|afc_system.launch' >/dev/null && die "a hand-started bridge is running; stop it first"
  [[ -z $(systemctl list-units --state=active --no-legend --plain 'afc-record@*' 2>/dev/null) ]] \
    || die "a bag is recording (afc_rec stop first)"
  # Read the GPS as the normal user, with their ROS environment (ROS_DOMAIN_ID etc.);
  # only the clock step itself runs as root.
  local utc
  utc=$(sudo -u "$RUSER" -H bash -ic '
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source "${AFC_WS:-$HOME/afc-drone/rpi_ros2}/install/setup.bash" >/dev/null 2>&1
    python3 - <<EOF
import sys, time, rclpy
from rclpy.qos import QoSPresetProfiles
from px4_msgs.msg import SensorGps
rclpy.init(); n = rclpy.create_node("afc_time_from_gps"); got = []
n.create_subscription(SensorGps, "/fmu/out/vehicle_gps_position",
                      lambda m: got.append((time.monotonic(), m)), QoSPresetProfiles.SENSOR_DATA.value)
t0 = time.monotonic()
while time.monotonic() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.1)
    if got and got[-1][1].fix_type >= 3 and got[-1][1].time_utc_usec > 0:
        break
rclpy.shutdown()
if not got or got[-1][1].fix_type < 3 or got[-1][1].time_utc_usec == 0:
    sys.exit(1)
t_rx, m = got[-1]
print(f"{m.time_utc_usec / 1e6 + (time.monotonic() - t_rx):.3f}")   # carried forward to now
EOF' 2>/dev/null | tail -1) || true
  [[ $utc =~ ^[0-9]+\.[0-9]+$ ]] || die "no 3D GPS fix with UTC time within 20 s (agent running? antenna outside?)"
  echo "Pi clock was $(python3 -c "import time;print(f'{time.time()-$utc:+.2f}')") s vs GPS (+-0.3 s transport lag); setting it"
  date -u -s "@$utc" >/dev/null
  echo "now $(date -u '+%Y-%m-%d %H:%M:%S UTC'). Restart the stack: sudo systemctl start afc-bridge"
}

case "${1:-status}" in
  status)        cmd_status ;;
  hostname)      cmd_hostname "${2:-}" ;;
  wifi-off)      cmd_wifi_off ;;
  wifi-on)       cmd_wifi_on ;;
  ro-root)       cmd_ro_root "${2:-}" ;;
  time-from-gps) cmd_time_from_gps ;;
  *) sed -n '2,19p' "$0"; exit 1 ;;
esac
