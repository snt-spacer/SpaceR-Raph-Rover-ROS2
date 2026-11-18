FROM leo_raph/base:1.0

ENV DEBIAN_FRONTEND=noninteractive

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Ubuntu 24.04 dependencies of the Spinnaker SDK
RUN apt-get update && apt-get install -y \
    libavcodec60 \
    libavformat60 \
    libswscale7 \
    libswresample4 \
    libavutil58 \
    libusb-1.0-0 \
    libpcre2-16-0 \
    libdouble-conversion3 \
    libxcb-xinput0 \
    libxcb-xinerama0 \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    ethtool \
    sudo \
    kmod \
    iproute2 \
    net-tools \
    debconf-utils \
    && rm -rf /var/lib/apt/lists/*

# Pre-seed FLIR EULA acceptance so the preinst scripts don't try to open an interactive dialog.
# The EULA key is registered under libspinnaker regardless of which package is being installed.
RUN echo "libspinnaker libspinnaker/accepted-flir-eula boolean true" | debconf-set-selections \
 && echo "libspinnaker libspinnaker/analytics-consent boolean false" | debconf-set-selections

WORKDIR /opt/spinnaker_setup

# Copy only the SDK files from the library subfolder
COPY library/ .

# Install the Spinnaker packages in the order install_spinnaker.sh uses.
# The interactive installer is skipped: it prompts for a user and sets up USB/udev,
# which a GigE camera in a privileged container does not need.
RUN dpkg -i libgentl_*.deb \
 && dpkg -i libspinnaker_*.deb \
 && dpkg -i libspinnaker-dev_*.deb \
 && dpkg -i libspinnaker-c_*.deb \
 && dpkg -i libspinnaker-c-dev_*.deb \
 && dpkg -i libspinvideo_*.deb \
 && dpkg -i libspinvideo-dev_*.deb \
 && dpkg -i libspinvideo-c_*.deb \
 && dpkg -i libspinvideo-c-dev_*.deb \
 && apt-get install -y ./spinview-qt_*.deb \
 && dpkg -i spinview-qt-dev_*.deb \
 && dpkg -i spinupdate_*.deb \
 && dpkg -i spinupdate-dev_*.deb \
 && dpkg -i spinnaker_*.deb \
 && dpkg -i spinnaker-doc_*.deb \
 && rm -rf /var/lib/apt/lists/*

# GenTL and Spinnaker paths
ENV GENICAM_GENTL64_PATH=/opt/spinnaker/lib/spinnaker-gentl
ENV PATH="${PATH}:/opt/spinnaker/bin"

WORKDIR /
RUN rm -rf /opt/spinnaker_setup

# ---- bumblebee_ros ROS 2 workspace ----
ENV BUMBLEBEE_WS=/opt/component_bumblebee_amd64
RUN mkdir -p ${BUMBLEBEE_WS}/src
WORKDIR ${BUMBLEBEE_WS}

RUN apt-get update && apt-get install -y \
    libopencv-dev \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    libpcl-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
 && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/Teledyne-MV/Stereo-BX-ROS2.git src/bumblebee_ros

COPY src ${BUMBLEBEE_WS}/src

RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --packages-select bumblebee_ros"

CMD ["bash", "/entrypoint.sh"]
