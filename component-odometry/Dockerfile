FROM leo_raph/base:1.0

# base:1.0 already provides rclpy, nav_msgs, sensor_msgs, geometry_msgs, tf2_ros
# and rmw_zenoh_cpp. Do NOT apt-install them here: pulling ros-jazzy-* again
# upgrades libfastcdr and breaks the typesupport ABI (undefined fastcdr symbol).

COPY odometry_node.py /odometry_node.py
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
