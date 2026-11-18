FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Base tools & certs
RUN apt-get update && apt-get install -y \
    ca-certificates gnupg lsb-release curl \
    tmux git build-essential python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# (Optional) Add repo.ros2.org extra source (signed)
RUN install -m 0755 -d /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://repo.ros2.org/ubuntu/main $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros2-extra.list

# Install ROS 2 packages available via APT on Jazzy
# (micro-ROS Agent is built below with micro_ros_setup)
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joy-linux \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-depth-image-proc \
    ros-jazzy-rplidar-ros \
    ros-jazzy-ament-cmake-black \
    python3-pip \
    libspdlog-dev \
    libfmt-dev \
    htop \
    nano \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-image-transport-plugins \
 && rm -rf /var/lib/apt/lists/*

# micro-ROS agent, for the base controller
ENV UROS_WS=/root/microros_ws
RUN mkdir -p ${UROS_WS}/src
WORKDIR ${UROS_WS}

RUN git -C src clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
RUN git -C src clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git 
RUN apt-get update && rosdep fix-permissions \
    &&  rosdep update \
    &&  rosdep install --from-paths src --ignore-src -y
RUN bash -lc "source /opt/ros/jazzy/setup.bash && \
              colcon build \
                --cmake-args -DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_P2P_PROFILE=OFF -DUAGENT_USE_SYSTEM_LOGGER=OFF \
                --no-warn-unused-cli"

# Convenience for the debug_cli: `rover-rviz` launches the mounted rover debug
# profile (TF, robot model, /odom, all point clouds, one image per camera). The
# profile is bind-mounted at /root/rover_debug.rviz by docker-compose.
RUN echo "alias rover-rviz='rviz2 -d /root/rover_debug.rviz'" >> /root/.bashrc

CMD ["bash"]
