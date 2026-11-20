# ros2-doctor-mcp with ROS 2 Humble and the doctors it wraps (nav2_doctor, armdoc, slam_doctor, percept_doctor) built in.
#   docker build -t ros2-doctor-mcp .
#   docker run -i --rm --net=host -e ROS_DOMAIN_ID=0 ros2-doctor-mcp          # MCP server on stdio, sees robots on this machine
# Claude Code:  claude mcp add ros2-doctor -- docker run -i --rm --net=host -e ROS_DOMAIN_ID=0 ros2-doctor-mcp
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-pip python3-colcon-common-extensions python3-numpy \
        ros-humble-control-msgs ros-humble-controller-manager-msgs ros-humble-diagnostic-msgs \
        ros-humble-nav2-msgs ros-humble-tf2-msgs ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/darshmenon/nav2_doctor.git \
 && git clone --depth 1 https://github.com/darshmenon/armdoc.git \
 && git clone --depth 1 https://github.com/darshmenon/slam_doctor.git \
 && git clone --depth 1 https://github.com/darshmenon/percept_doctor.git
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select nav2_doctor armdoc slam_doctor percept_doctor

COPY . /opt/ros2_doctor_mcp
# the image's stock pip and setuptools cannot read pyproject.toml and would install an empty 'UNKNOWN' package
RUN pip3 install --no-cache-dir -U pip setuptools wheel && pip3 install --no-cache-dir /opt/ros2_doctor_mcp

# Fast DDS shared memory does not work between a container and the host (discovery succeeds, no data arrives),
# so the image talks UDP only
COPY docker/fastdds_udp.xml /etc/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_udp.xml

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
