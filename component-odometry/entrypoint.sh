#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER=tcp/10.10.0.15:7447

echo "[Odometry] Waiting for Zenoh router TCP..."
until timeout 1 bash -c '</dev/tcp/10.10.0.15/7447' 2>/dev/null; do
  sleep 0.5
done
echo "[Odometry] Router reachable"

exec python3 /odometry_node.py
