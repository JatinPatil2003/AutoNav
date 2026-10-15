#!/bin/bash
set -e

# setup ros2 environment
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/fastdds.xml

source /opt/venv/bin/activate

source "/opt/ros/kilted/setup.bash"
source "/colcon_ws/install/setup.bash"

exec "$@"