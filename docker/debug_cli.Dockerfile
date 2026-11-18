# debug_cli: the base tools (rviz2, rqt, ROS CLI) plus rapha_description, so RViz
# can load the robot's meshes (package://rapha_description/models/...).
FROM leo_raph/base:1.0

ENV DESC_WS=/root/desc_ws
RUN mkdir -p ${DESC_WS}/src
COPY component-leo_base_system/src/rapha_common/rapha_description ${DESC_WS}/src/rapha_description
RUN bash -lc "source /opt/ros/jazzy/setup.bash && cd ${DESC_WS} && colcon build"

# Overlay the description workspace in every interactive shell so package:// mesh
# URIs resolve for rviz2 (the rover-rviz alias is inherited from base:1.0).
RUN printf '\nsource /opt/ros/jazzy/setup.bash\nsource %s/install/setup.bash\n' "${DESC_WS}" >> /root/.bashrc

CMD ["bash"]
