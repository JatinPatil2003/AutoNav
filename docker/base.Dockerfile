# Shared ROS 2 workspace image for all robot-side containers.
#
# Build once and tag it, then every other Dockerfile in this directory
# builds FROM that tag instead of recompiling the workspace:
#
#   docker build -f docker/base.Dockerfile -t autonav-ros-base:latest .
#
# This keeps each runtime container (lidar/imu/controller/...) a thin
# wrapper around one shared, already-built colcon workspace instead of
# rebuilding every ROS package from source N times.
FROM ros:kilted

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kilted

# -----------------------------
# System + ROS dependencies
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-pip \
    python3-venv \
    git \
    build-essential \
    cmake \
    network-manager \
    libusb-1.0-0 \
    libmodbus-dev \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Python virtual environment (autonav_imu's sensor driver deps)
# -----------------------------
RUN python3 -m venv /opt/venv --system-site-packages \
    && /opt/venv/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/venv/bin/pip install --no-cache-dir \
        pyserial==3.4 \
        smbus \
        transforms3d

RUN python3 -m pip install --break-system-packages \
        smbus2 \
        pyserial==3.4

# -----------------------------
# colcon workspace: every ROS 2 package that ships on the robot
# -----------------------------
WORKDIR /colcon_ws

RUN chown -R 1000:1000 /colcon_ws

COPY autonav_bringup src/autonav_bringup
COPY autonav_controller src/autonav_controller
COPY autonav_description src/autonav_description
COPY autonav_firmware src/autonav_firmware
COPY autonav_imu src/autonav_imu
COPY autonav_localization src/autonav_localization
COPY autonav_navigation src/autonav_navigation
COPY autonav_lidar src/autonav_lidar

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

ENV PATH="/opt/venv/bin:$PATH"

COPY docker/entrypoint.bash /entrypoint.bash
COPY fastdds.xml /fastdds.xml
RUN chmod +x /entrypoint.bash

ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
