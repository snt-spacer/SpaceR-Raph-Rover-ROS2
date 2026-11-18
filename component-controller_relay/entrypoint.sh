#!/usr/bin/env bash
set -Eeuo pipefail
die(){ echo "ERROR: $*" >&2; exit 1; }

[[ "${BRIDGE_SIDE:-}" == "dds" || "${BRIDGE_SIDE:-}" == "zenoh" ]] || \
  die "BRIDGE_SIDE must be set to 'dds' or 'zenoh'"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/controller_relay_ws/install/local_setup.bash
set -u

exec python3 /bridge_node.py
