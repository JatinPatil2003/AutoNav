# Navigation container: Nav2 (autonomous navigation on a saved map), or
# slam_toolbox / cartographer when mapping a new space. Pick the mode via
# NAV_MODE (navigation | slam | cartographer) — only one of these runs at
# a time, they are mutually exclusive drive modes, not separate services.
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinpatil2003/autonav:base
FROM ${BASE_IMAGE}

ENV NAV_MODE=navigation
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /colcon_ws

CMD ["bash", "-c", "ros2 launch autonav_navigation ${NAV_MODE}.launch.py"]
