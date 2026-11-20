#!/bin/bash
# stdout is the MCP channel: keep everything else on stderr
set -e
source /opt/ros/humble/setup.bash 1>&2
source /ws/install/setup.bash 1>&2
exec ros2-doctor-mcp "$@"
