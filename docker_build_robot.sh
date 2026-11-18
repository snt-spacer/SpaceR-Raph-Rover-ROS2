#!/usr/bin/env bash
# Build and start the ROVER stack (run on the Jetson).
# All sensors, the base system, the microcontroller bridge, odometry, the voxel
# filter and the Zenoh router. See docker-compose.robot.yml.
set -Eeuo pipefail
cd "$(dirname "$0")"

COMPOSE="docker-compose.robot.yml"

# Base image every component-*/Dockerfile builds FROM.
docker build -t leo_raph/base:1.0 -f docker/base.Dockerfile docker

docker compose -f "$COMPOSE" build
docker compose -f "$COMPOSE" up -d

echo
echo "Rover stack up. Check with:  docker compose -f $COMPOSE ps"
