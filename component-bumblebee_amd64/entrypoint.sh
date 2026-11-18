#!/usr/bin/env bash
set -Eeuo pipefail
die(){ echo "ERROR: $*" >&2; exit 1; }

[[ -f /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash"
[[ -f /opt/component_bumblebee_amd64/install/local_setup.bash ]] || die "Missing /opt/component_bumblebee_amd64/install/local_setup.bash"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/component_bumblebee_amd64/install/local_setup.bash
set -u

CAMERA_SERIAL="${CAMERA_SERIAL:-25050218}"
CAMERA_IP="${CAMERA_IP:-10.10.0.17}"
CAMERA_SUBNET="${CAMERA_SUBNET:-255.255.255.0}"
CAMERA_GATEWAY="${CAMERA_GATEWAY:-10.10.0.1}"
GIGE_INTERFACE="${GIGE_INTERFACE:-eth0}"

# Spinnaker network tuning for the GigE interface
echo "[Bumblebee] Running gev_nettweak on ${GIGE_INTERFACE}..."
gev_nettweak "${GIGE_INTERFACE}" || echo "[Bumblebee] WARNING: gev_nettweak failed on ${GIGE_INTERFACE}"

echo "[Bumblebee] Forcing GigE camera IP (serial=${CAMERA_SERIAL} ip=${CAMERA_IP})..."
GigEConfig -s "${CAMERA_SERIAL}" -i "${CAMERA_IP}" -n "${CAMERA_SUBNET}" -g "${CAMERA_GATEWAY}" || \
    echo "[Bumblebee] WARNING: GigEConfig failed — camera may not be connected"

echo "[Bumblebee] Publishing static transform base_link → camera_link (z=0.40 m)..."
ros2 run tf2_ros static_transform_publisher 0 0 0.40 0 0 0 base_link camera_link &

echo "[Bumblebee] Launching stereo_image_publisher (serial=${CAMERA_SERIAL})..."
exec ros2 run bumblebee_ros stereo_image_publisher "${CAMERA_SERIAL}"
