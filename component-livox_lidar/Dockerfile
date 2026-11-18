FROM leo_raph/base:1.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake build-essential git \
    iproute2 \
    ros-jazzy-rmw-zenoh-cpp \
    libpcl-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Build Livox-SDK2
WORKDIR /opt
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake -DCMAKE_CXX_FLAGS="-include cstdint" .. && make -j$(nproc) && make install && ldconfig

# Copy your livox_ros_driver2 workspace
ENV LIVOX_WS=/opt/livox_ws
RUN mkdir -p ${LIVOX_WS}/src
WORKDIR ${LIVOX_WS}
COPY src ${LIVOX_WS}/src

RUN bash -lc "source /opt/ros/jazzy/setup.bash && cd ${LIVOX_WS}/src/livox_ros_driver2 && bash build.sh jazzy"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
CMD ["bash", "/entrypoint.sh"]
