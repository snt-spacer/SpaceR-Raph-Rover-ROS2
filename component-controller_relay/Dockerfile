FROM leo_raph/base:1.0

ENV DEBIAN_FRONTEND=noninteractive

ENV RELAY_WS=/opt/controller_relay_ws
RUN mkdir -p ${RELAY_WS}/src
WORKDIR ${RELAY_WS}

# Only the message/service definitions are needed here, not the rest of leo_base_system.
COPY component-leo_base_system/src/rapha_common/rapha_interfaces src/rapha_interfaces

RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"

COPY component-controller_relay/bridge_node.py /bridge_node.py
COPY component-controller_relay/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
