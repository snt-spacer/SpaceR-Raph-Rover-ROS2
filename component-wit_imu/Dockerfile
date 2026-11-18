FROM leo_raph/base:1.0

ENV LEO_BASE_WS=/root/leo_base_ws
RUN mkdir -p ${LEO_BASE_WS}/src
WORKDIR ${LEO_BASE_WS}

# Install missing dependencies
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
 && rm -rf /var/lib/apt/lists/*

# Copy source
COPY src ${LEO_BASE_WS}/src

# Build
RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
