#!/usr/bin/env bash
set -Eeuo pipefail
die(){ echo "ERROR: $*" >&2; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
source /root/leo_base_ws/install/local_setup.bash
set -u

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_zenoh_cpp" && -n "${ZENOH_ROUTER:-}" ]]; then
  host_port="${ZENOH_ROUTER#tcp/}"
  echo "[leo_base_system] Waiting for Zenoh router ${host_port}..."
  until timeout 1 bash -c "</dev/tcp/${host_port%%:*}/${host_port##*:}" 2>/dev/null; do
    sleep 0.5
  done
  echo "[leo_base_system] Router reachable"
fi

exec ros2 launch rapha_bringup rapha_bringup.launch.xml
