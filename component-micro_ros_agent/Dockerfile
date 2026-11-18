FROM leo_raph/base:1.0

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Defaults (override with -e at runtime)
ENV AGENT_PORT=8888 \
    DISCOVERY_PORT=7400 \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    VERBOSITY=-v4

CMD ["bash", "/entrypoint.sh"]
