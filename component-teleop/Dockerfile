FROM leo_raph/base:1.0

ENV TELEOP_WS=/root/teleop_ws
RUN mkdir -p ${TELEOP_WS}/src
WORKDIR ${TELEOP_WS}

# joy_linux, ackermann_msgs, sensor_msgs, std_srvs are all already in base:1.0.
# We only build the two rover-specific packages: rapha_interfaces (SteeringMode
# msg + SetSteeringMode srv used by the teleop node) and rapha_teleop itself.
COPY src ${TELEOP_WS}/src

RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
