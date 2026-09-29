#!/usr/bin/env bash
# afc_bridge_run.sh -- start the AFC bridge + rosbridge (ExecStart of afc-bridge.service).
#
# The uXRCE-DDS agent is NOT started here: it runs as its own service
# (microxrce-agent.service), so this service can be restarted on its own (T-C2).
# Cycling the agent is also recoverable on firmware built with patch_px4.py
# steps 4-6 (reconnect fix + TELEM2 TX drain; D1/D2 2026-09-28). On older
# firmware an agent outage wedged the FC's DDS link until an FC reboot.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source "${AFC_WS:-/home/pi/afc-drone/rpi_ros2}/install/setup.bash"
set -u
exec ros2 launch afc_bridge afc_system.launch.py start_agent:=false
