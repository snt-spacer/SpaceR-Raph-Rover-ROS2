#!/usr/bin/env bash
set -Eeo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER=tcp/10.10.0.15:7447

until timeout 1 bash -c '</dev/tcp/10.10.0.15/7447' 2>/dev/null; do
  sleep 0.5
done
echo "[OAK Camera] Zenoh router reachable, launching..."

# RGB rectification (the driver does not undistort RGB on the device). Run in
# the /loccam/rgb namespace so image_rect and its compressed variants land there.
ros2 run image_proc rectify_node --ros-args \
    -r __ns:=/loccam/rgb \
    -r image:=image_raw \
    -r image_rect:=image_rect &

# XYZRGB cloud from the RGB-aligned depth, coloured by the raw RGB image (which
# is pixel-aligned with that depth) -> /loccam/points.
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
    -r rgb/image_rect_color:=/loccam/rgb/image_raw \
    -r rgb/camera_info:=/loccam/rgb/camera_info \
    -r depth_registered/image_rect:=/loccam/stereo/image_raw \
    -r points:=/loccam/points &

# Standalone camera_node (the composable launch fails to load in this image).
# The node name "loccam" selects the /loccam parameters and the loccam_* TF frames.
exec ros2 run depthai_ros_driver camera_node --ros-args \
    -r __node:=loccam \
    --params-file /opt/component-oak_camera/config/oak_d_pro_w.yaml
