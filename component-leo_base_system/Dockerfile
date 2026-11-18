FROM leo_raph/base:1.0
ENV LEO_BASE_WS=/root/leo_base_ws
RUN mkdir -p ${LEO_BASE_WS}/src
WORKDIR ${LEO_BASE_WS}
RUN apt-get update && apt-get install -y \
    ros-jazzy-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*
COPY src ${LEO_BASE_WS}/src
RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
CMD ["bash"]
