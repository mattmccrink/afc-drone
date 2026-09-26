#!/usr/bin/env bash
# afc_record.sh -- record the AFC topic set to a timestamped rosbag.
#
# Normally started by systemd (afc-record@<tag>.service) via the `afc_rec`
# wrapper, NOT from an SSH shell: a recorder in an SSH terminal dies with the
# session (SIGHUP) if the link over the Doodle drops mid-test, and the bag can
# be left without metadata. Under systemd it keeps recording and is stopped
# with SIGINT so the bag is always closed cleanly.
#
#   $1 = optional tag appended to the bag name (e.g. T-C5, tunnel_run3)
set -euo pipefail

# ROS env (set -u is relaxed around the setup scripts, which reference unset vars)
set +u
source /opt/ros/humble/setup.bash
source "${AFC_WS:-/home/pi/afc-drone/rpi_ros2}/install/setup.bash"
set -u

BAG_DIR="${AFC_BAG_DIR:-/home/pi/bags}"
mkdir -p "$BAG_DIR"

TAG="${1:-}"
NAME="afc_$(date +%Y%m%d_%H%M%S)${TAG:+_$TAG}"

# Topic set: everything needed for post-test cause/effect.
#   FC side : command demand + arm state (as the bridge received them)
#   Tiny side: control, sensor, compressor telemetry + bridge health
TOPICS=(
  /fmu/out/vehicle_torque_setpoint
  /fmu/out/vehicle_thrust_setpoint
  /fmu/out/vehicle_status_v1
  /afc/ctrl_tlm
  /afc/sensor_tlm
  /afc/compressor
  /afc/health
)

echo "[afc_record] writing $BAG_DIR/$NAME"
# exec: the recorder becomes the service's main process, so systemd's
# SIGINT reaches it directly and it finalizes metadata.yaml on stop.
exec ros2 bag record -o "$BAG_DIR/$NAME" "${TOPICS[@]}"
