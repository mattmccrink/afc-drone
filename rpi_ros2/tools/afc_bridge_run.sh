#!/usr/bin/env bash
# afc_bridge_run.sh -- start the AFC bridge + rosbridge (ExecStart of afc-bridge.service).
#
# The uXRCE-DDS agent is NOT started here: it runs as its own service
# (microxrce-agent.service) and must never be cycled -- an agent restart wedges
# the FC's DDS link until an FC reboot (S14). Restarting THIS service is safe
# (T-C2: a bridge-only restart recovers without an FC reboot).
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source "${AFC_WS:-/home/pi/afc-drone/rpi_ros2}/install/setup.bash"
set -u
exec ros2 launch afc_bridge afc_system.launch.py start_agent:=false
