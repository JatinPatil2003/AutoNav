#!/bin/bash
set -e

# setup ros2 environment
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "/opt/ros/humble/setup.bash"
source "/colcon_ws/install/setup.bash"
exec "$@"