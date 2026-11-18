#!/usr/bin/env bash
# Build and start the OPERATOR stack (run on the remote computer, not the robot).
# Gamepad teleop + debug CLI / RViz, connected to the robot's Zenoh router. See
# docker-compose.remote.yml (and config/zenoh_session.json5 for the robot IP).
set -Eeuo pipefail
cd "$(dirname "$0")"

COMPOSE="docker-compose.remote.yml"

# component-teleop builds FROM leo_raph/base:1.0; debug_cli builds from
# docker/base.Dockerfile directly. Build the base first.
docker build -t leo_raph/base:1.0 -f docker/base.Dockerfile docker

docker compose -f "$COMPOSE" build
docker compose -f "$COMPOSE" up -d

echo
echo "Operator stack up."
echo "  Teleop: plug in the gamepad (/dev/input/js0)."
echo "  Topics / RViz:  docker compose -f $COMPOSE exec debug_cli bash"
echo "                  then inside:  ros2 topic list   or   rover-rviz"
