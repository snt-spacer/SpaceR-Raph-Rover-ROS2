#!/usr/bin/env bash
set -Eeo pipefail          # removed -u intentionally for ROS sourcing

die(){ echo "ERROR: $*" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash
source /opt/component_bumblebee_arm64/install/local_setup.bash

set -u                     # re-enable after ROS sources are loaded

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER=tcp/10.10.0.15:7447

echo "[Bumblebee] Waiting for Zenoh router TCP..."

until timeout 1 bash -c '</dev/tcp/10.10.0.15/7447' 2>/dev/null; do
  sleep 0.5
done

echo "[Bumblebee] Router reachable"

CAMERA_SERIAL="${CAMERA_SERIAL:-25050218}"
CAMERA_IP="${CAMERA_IP:-10.10.0.17}"
CAMERA_SUBNET="${CAMERA_SUBNET:-255.255.255.0}"
CAMERA_GATEWAY="${CAMERA_GATEWAY:-10.10.0.1}"
GIGE_INTERFACE="${GIGE_INTERFACE:-enP8p1s0}"

gev_nettweak "${GIGE_INTERFACE}" || true

# gev_nettweak sets jumbo frames (MTU 9000) for the camera, but the base
# controller on the same NIC cannot receive them: cap the path MTU to it at 1500.
# set_network.sh sets the same route persistently.
MCU_IP="${MCU_IP:-10.10.0.211}"
ip route replace "${MCU_IP}/32" dev "${GIGE_INTERFACE}" scope link mtu 1500 || true
sleep 2

GigEConfig -a || true
sleep 10

GigEConfig -s "${CAMERA_SERIAL}" -i "${CAMERA_IP}" -n "${CAMERA_SUBNET}" -g "${CAMERA_GATEWAY}" || true
sleep 10

ros2 daemon stop 2>/dev/null || true
ros2 daemon start

exec ros2 run bumblebee_publisher bumblebee_node \
  --ros-args \
  -p camera_serial:="'${CAMERA_SERIAL}'" \
  -p resolution:="${CAMERA_RESOLUTION:-Quarter}" \
  -p fps:="${CAMERA_FPS:-2.0}" \
  -p pixel_format:="${CAMERA_PIXEL_FORMAT:-RGB8Packed}" \
  -p frame_id:="${CAMERA_FRAME_ID:-navcam_optical_frame}" \
  -p debug_view:="${DEBUG_VIEW:-false}"
