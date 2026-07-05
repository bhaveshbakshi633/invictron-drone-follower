# Reproducible ROS2 Humble + PX4 SITL + Gazebo image.
# The whole point: the reviewer's host distro does not matter -- "It Runs" is
# guaranteed inside this pinned image. The same image backs local runs and CI.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ARG PX4_VERSION=v1.15.0
ARG PX4_MSGS_BRANCH=release/1.15
ARG AGENT_VERSION=v2.4.3

RUN apt-get update && apt-get install -y --no-install-recommends \
      git wget curl build-essential cmake ninja-build sudo lsb-release gnupg \
      python3-pip python3-colcon-common-extensions python3-jinja2 \
      python3-numpy python3-matplotlib python3-empy \
 && rm -rf /var/lib/apt/lists/*

# --- micro-XRCE-DDS Agent (PX4 <-> ROS2 bridge) ------------------------------
# Superbuild: `cmake ..` fetches+builds Fast-DDS, then `make` builds the agent
# and `make install` installs it. Must build `all` before `install`.
RUN git clone -b ${AGENT_VERSION} --depth 1 \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent /tmp/agent \
 && mkdir -p /tmp/agent/build && cd /tmp/agent/build \
 && cmake .. \
 && make -j"$(nproc)" \
 && make install \
 && ldconfig /usr/local/lib \
 && cd / && rm -rf /tmp/agent

# --- PX4-Autopilot: deps + pre-built SITL (gz x500) --------------------------
ENV PX4_DIR=/opt/PX4-Autopilot
RUN git clone -b ${PX4_VERSION} --depth 1 --recursive \
      https://github.com/PX4/PX4-Autopilot ${PX4_DIR}
RUN bash ${PX4_DIR}/Tools/setup/ubuntu.sh --no-nuttx
# Build-only target: compiles the SITL firmware (incl. the gz bridge) and EXITS.
# NOTE: `DONT_RUN=1 make px4_sitl gz_x500` is NOT honored by the gz run-target --
# it launches the sim and hangs the build at the interactive pxh> prompt.
RUN cd ${PX4_DIR} && make px4_sitl_default

# --- ROS2 workspace: px4_msgs + our package ----------------------------------
ENV WS=/root/ros2_ws
RUN mkdir -p ${WS}/src \
 && git clone -b ${PX4_MSGS_BRANCH} --depth 1 \
      https://github.com/PX4/px4_msgs ${WS}/src/px4_msgs
COPY drone_system ${WS}/src/drone_system
RUN . /opt/ros/humble/setup.sh && cd ${WS} \
 && colcon build --symlink-install

# tools + scripts used by CI and by hand
COPY tools /root/tools
COPY scripts /root/scripts
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /root/scripts/*.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
