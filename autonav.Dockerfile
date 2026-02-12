FROM ros:kilted

SHELL ["/bin/bash", "-c"]

# -----------------------------
# 1. Environment + noninteractive
# -----------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kilted

# -----------------------------
# 2. System dependencies (single apt layer)
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS packages
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf-transformations \
    \
    # Build + system tools
    python3-pip \
    python3-venv \
    git \
    build-essential \
    cmake \
    network-manager \
    docker.io \
    \
    # Runtime deps
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 3. Build & install YDLidar SDK
# -----------------------------
RUN git clone --depth=1 https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/YDLidar-SDK \
    && cd /tmp/YDLidar-SDK \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/YDLidar-SDK

# -----------------------------
# 4. Python virtual environment
# -----------------------------
RUN python3 -m venv /opt/venv --system-site-packages \
    && /opt/venv/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/venv/bin/pip install --no-cache-dir \
        fastapi \
        pymongo \
        pyserial==3.4 \
        smbus \
        transforms3d \
        "uvicorn[standard]" \
        aiortc \
        websockets \
        opencv-python \
        av

RUN apt-get update && apt-get install -y python3-pip \
    && python3 -m pip install --break-system-packages \
       smbus2 \
       pyserial==3.4        

# -----------------------------
# 5. ROS 2 workspace
# -----------------------------
WORKDIR /colcon_ws

RUN chown -R 1000:1000 /colcon_ws

# Copy full sources
COPY autonav_bringup src/autonav_bringup
COPY autonav_controller src/autonav_controller
COPY autonav_description src/autonav_description
COPY autonav_firmware src/autonav_firmware
COPY autonav_localization src/autonav_localization
COPY autonav_navigation src/autonav_navigation
COPY bno055 src/bno055
COPY ydlidar_ros2_driver src/ydlidar_ros2_driver

# Build workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install 


ENV PATH="/opt/venv/bin:$PATH"

# -----------------------------
# 6. Runtime files
# -----------------------------
COPY autonav_entrypoint.bash /autonav_entrypoint.bash
COPY fastdds.xml /fastdds.xml
RUN chmod +x /autonav_entrypoint.bash

COPY WebServer /WebServer
WORKDIR /WebServer

# -----------------------------
# 7. Entrypoint & server
# -----------------------------
EXPOSE 8000

ENTRYPOINT ["/autonav_entrypoint.bash"]
CMD ["uvicorn", "index:app", "--host", "0.0.0.0", "--port", "8000"]
