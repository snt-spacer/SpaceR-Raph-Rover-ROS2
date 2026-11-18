#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /root/leo_base_ws/install/setup.bash

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_zenoh_cpp" && -n "${ZENOH_ROUTER:-}" ]]; then
  host_port="${ZENOH_ROUTER#tcp/}"
  echo "[wit_imu] Waiting for Zenoh router ${host_port}..."
  until timeout 1 bash -c "</dev/tcp/${host_port%%:*}/${host_port##*:}" 2>/dev/null; do
    sleep 0.5
  done
  echo "[wit_imu] Router reachable"
fi

DEVICE="/dev/ttyUSB0"

echo "Checking IMU device..."

# List the serial devices, for the log
ls -l /dev/ttyUSB*

# Wait up to 10 s for the IMU
for i in {1..10}; do
  if [ -e "$DEVICE" ]; then
    echo "IMU found!"
    break
  fi
  echo "Waiting for IMU..."
  sleep 1
done

if [ ! -e "$DEVICE" ]; then
  echo "ERROR: IMU not found at $DEVICE"
  exit 1
fi

echo "Starting IMU driver..."

# Group everything this driver emits under a single /imu_wit namespace. The node
# publishes relative topics (data_raw, mag, temperature, orientation), so
# __ns:=/imu_wit makes them /imu_wit/data_raw, /imu_wit/mag,
# /imu_wit/temperature and /imu_wit/orientation.
ros2 run wit_imu_driver wit_imu_node \
  --ros-args \
  -r __ns:=/imu_wit \
  -p device:=$DEVICE \
  -p baud:=115200
