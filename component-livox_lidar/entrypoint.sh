#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/livox_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER=tcp/10.10.0.15:7447

echo "[Livox] Waiting for Zenoh router TCP..."
until timeout 1 bash -c '</dev/tcp/10.10.0.15/7447' 2>/dev/null; do
  sleep 0.5
done
echo "[Livox] Router reachable"

# The LiDAR host endpoint (192.168.1.5/24) is configured persistently by the
# host netplan (see set_network.sh)
if ! ip -4 addr show dev enP8p1s0 | grep -q '192\.168\.1\.5/'; then
  echo "[Livox] ERROR: 192.168.1.5/24 is not on enP8p1s0." >&2
  echo "[Livox] Run set_network.sh on the host to configure the LiDAR endpoint." >&2
  exit 1
fi
echo "[Livox] LiDAR host endpoint 192.168.1.5/24 present"

exec ros2 launch livox_ros_driver2 msg_HAP_launch.py
