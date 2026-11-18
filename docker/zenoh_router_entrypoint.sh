#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

SRC=/opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
CFG=/tmp/rmw_zenoh_router_config.json5
cp "$SRC" "$CFG"

# Transmit settings for multi-megabyte point clouds over a congested link. The
# defaults drop fragmented messages (anything over 64 KB) after 50 ms:
#   - queue size per priority 2 -> 16 (the maximum Zenoh accepts)
#   - max_wait_before_drop_fragments 50 ms -> 10 s
#   - wait_before_drop 1 ms -> 100 ms
sed -i -E 's/(control|real_time|interactive_high|interactive_low|data_high|data|data_low|background): 2,/\1: 16,/' "$CFG"
sed -i -E 's/max_wait_before_drop_fragments: [0-9]+/max_wait_before_drop_fragments: 10000000/' "$CFG"
sed -i -E 's/([^_]wait_before_drop): [0-9]+/\1: 100000/' "$CFG"

export ZENOH_ROUTER_CONFIG_URI="$CFG"
echo "[zenoh_router] starting rmw_zenohd (tuned: tx queue=16, drop_fragments=10s) listening on :7447"
exec ros2 run rmw_zenoh_cpp rmw_zenohd
