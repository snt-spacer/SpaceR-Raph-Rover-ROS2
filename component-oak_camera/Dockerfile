FROM leo_raph/base:1.0

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN apt-get update && apt-get install -y \
    usbutils \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-depthai-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-rmw-fastrtps-cpp ros-jazzy-fastcdr \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-vision-msgs \
    ros-jazzy-rviz-default-plugins \
 && rm -rf /var/lib/apt/lists/*

ENV  OPENBLAS_CORETYPE ARMV8
COPY /90-luxonis-usb.rules /etc/udev/rules.d/

ENV OAK_WS=/opt/component-oak_camera
RUN mkdir -p ${OAK_WS}/config
WORKDIR ${OAK_WS}
COPY config ${OAK_WS}/config

CMD ["bash", "/entrypoint.sh"]
