#!/usr/bin/env bash
set -Eeuo pipefail
die(){ echo "ERROR: $*" >&2; exit 1; }

[[ -f /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash"
[[ -f /root/microros_ws/install/local_setup.bash ]] || die "Missing /root/microros_ws/install/local_setup.bash"

set +u
source /opt/ros/jazzy/setup.bash
source /root/microros_ws/install/local_setup.bash
set -u

AGENT_PORT="${AGENT_PORT:-8888}"
DISCOVERY_PORT="${DISCOVERY_PORT:-7400}"
VERBOSITY="${VERBOSITY:--v4}"

echo "[micro-ROS Agent]"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"
echo "  UDP port: ${AGENT_PORT}"

if [[ "${DISCOVERY_PORT}" == "0" ]]; then
  echo "  Discovery Server: DISABLED"
  exec ros2 run micro_ros_agent micro_ros_agent udp4 -p "${AGENT_PORT}" ${VERBOSITY}
else
  echo "  Discovery Server: ${DISCOVERY_PORT}"
  echo "  Hint: export ROS_DISCOVERY_SERVER=<HOST_IP>:${DISCOVERY_PORT}"
  exec ros2 run micro_ros_agent micro_ros_agent udp4 -p "${AGENT_PORT}" -d "${DISCOVERY_PORT}" ${VERBOSITY}
fi
