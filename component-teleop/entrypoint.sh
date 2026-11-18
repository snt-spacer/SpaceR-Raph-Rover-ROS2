#!/usr/bin/env bash
set -Eeuo pipefail
die(){ echo "ERROR: $*" >&2; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
source /root/teleop_ws/install/local_setup.bash
set -u

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_zenoh_cpp" && -n "${ZENOH_ROUTER:-}" ]]; then
  host_port="${ZENOH_ROUTER#tcp/}"
  echo "[teleop] Waiting for Zenoh router ${host_port}..."
  until timeout 1 bash -c "</dev/tcp/${host_port%%:*}/${host_port##*:}" 2>/dev/null; do
    sleep 0.5
  done
  echo "[teleop] Router reachable"
fi

JOY_DEV="${JOY_DEV:-/dev/input/js0}"

echo "[teleop] Waiting for joystick device ${JOY_DEV}..."
for i in {1..10}; do
  if [ -e "${JOY_DEV}" ]; then
    echo "[teleop] Joystick found at ${JOY_DEV}"
    break
  fi
  sleep 1
done
if [ ! -e "${JOY_DEV}" ]; then
  echo "[teleop] WARNING: ${JOY_DEV} not present yet; joy_linux_node will keep retrying."
fi

exec ros2 launch rapha_teleop joy_teleop.launch.xml joy_dev:="${JOY_DEV}"
