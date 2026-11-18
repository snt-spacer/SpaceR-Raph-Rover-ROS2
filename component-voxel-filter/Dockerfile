FROM leo_raph/base:1.0

# base:1.0 already provides rclpy, sensor_msgs and numpy. Do NOT apt-install
# ros-jazzy-* here (it upgrades libfastcdr and breaks the typesupport ABI).

COPY voxel_filter_node.py /voxel_filter_node.py
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
