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
FROM jatinpatil2003/autonav:lyrical-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=lyrical

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
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage \
    python3-pip \
    python3-venv \
    git \
    build-essential \
    cmake \
    network-manager \
    libusb-1.0-0 \
    libmodbus-dev \
    # No ros-${ROS_DISTRO}-cartographer{,-ros} binary exists yet for this
    # distro — these are cartographer's + cartographer_ros's own system deps,
    # built from source below instead.
    libboost-iostreams-dev \
    libeigen3-dev \
    libabsl-dev \
    libcairo2-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libgmock-dev \
    libgtest-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Ceres Solver, pinned + built from source
# -----------------------------
# The distro's libceres-dev floats with the OS and has already moved past
# 2.2, which deleted the LocalParameterization/AutoDiffLocalParameterization/
# QuaternionParameterization API that this pinned cartographer commit (see
# below) is written against. Rather than patch cartographer's Ceres call
# sites to track whatever API the OS happens to ship, build the last Ceres
# release that still has the old API and pin it here for good.
RUN git clone --branch 2.1.0 --depth 1 https://github.com/ceres-solver/ceres-solver.git /tmp/ceres-solver \
    && cmake -S /tmp/ceres-solver -B /tmp/ceres-solver/build \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DPROVIDE_UNINSTALL_TARGET=OFF \
        -DCXSPARSE=OFF \
        -DSUITESPARSE=OFF \
        -DCUDA=OFF \
    && cmake --build /tmp/ceres-solver/build --target install -j"$(nproc)" \
    && rm -rf /tmp/ceres-solver

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

# cartographer (plain-cmake core, no ament_cmake/rosdep binary for this
# distro yet) + the ros2/cartographer_ros fork (the actively maintained ROS 2
# port, tracked by OSRF against rolling/Kilted+) — dropped straight into the
# workspace so colcon builds them alongside everything else instead of a
# separate manual install step. Pinned to known-good commits, not a floating
# branch. cartographer_rviz (submap RViz plugin) isn't used by this repo's
# rviz configs, so it's excluded from the build.
RUN git clone https://github.com/cartographer-project/cartographer.git src/cartographer \
    && git -C src/cartographer checkout 877157a0d91788a7700221d87232d412cb3c1ef4 \
    && git clone --branch ros2 https://github.com/ros2/cartographer_ros.git src/cartographer_ros \
    && git -C src/cartographer_ros checkout 743345256b43a7fd95765d40c7332b6f31ab66bb \
    && touch src/cartographer_ros/cartographer_rviz/COLCON_IGNORE \
    # Pinned cartographer commit predates CMake dropping support for
    # policies below 3.5; the system CMake here is too new to build it as-is.
    && sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5...3.27)/' src/cartographer/CMakeLists.txt \
    # Ceres is built below without SuiteSparse (its legacy FindSuiteSparse.cmake
    # doesn't interoperate with the SuiteSparse version this OS ships) — Eigen's
    # sparse backend is all cartographer's solver needs, so drop the hard
    # SuiteSparse component requirement instead of chasing that CMake module.
    && sed -i 's/find_package(Ceres REQUIRED COMPONENTS SuiteSparse)/find_package(Ceres REQUIRED)/' src/cartographer/CMakeLists.txt \
    # GCC's newer -Wmaybe-uninitialized analysis false-positives on Eigen's
    # copy constructors (a known Eigen/GCC interaction) and cartographer
    # promotes it to an error via -Werror=uninitialized; silence just that
    # sub-diagnostic, keeping genuine -Werror=uninitialized cases fatal.
    && sed -i 's/google_add_flag(GOOG_CXX_FLAGS "-Werror=uninitialized")/google_add_flag(GOOG_CXX_FLAGS "-Werror=uninitialized")\n    google_add_flag(GOOG_CXX_FLAGS "-Wno-maybe-uninitialized")/' src/cartographer/cmake/functions.cmake \
    # The installed libabsl-dev no longer defines the bare thread-safety
    # annotation macros (GUARDED_BY, LOCKS_EXCLUDED, ...) that this pinned
    # cartographer + cartographer_ros commit use directly — only the
    # ABSL_-prefixed forms remain, so rewrite the source to use those instead.
    && grep -rlE '\b(GUARDED_BY|PT_GUARDED_BY|LOCKS_EXCLUDED|EXCLUSIVE_LOCKS_REQUIRED|SHARED_LOCKS_REQUIRED|LOCKABLE|SCOPED_LOCKABLE|EXCLUSIVE_LOCK_FUNCTION|SHARED_LOCK_FUNCTION|UNLOCK_FUNCTION|LOCK_RETURNED|NO_THREAD_SAFETY_ANALYSIS|ACQUIRED_AFTER|ACQUIRED_BEFORE|ASSERT_EXCLUSIVE_LOCK|ASSERT_SHARED_LOCK)\(' src/cartographer src/cartographer_ros --include='*.h' --include='*.hpp' --include='*.cc' --include='*.cpp' \
         | xargs sed -i -E \
             -e 's/\b(GUARDED_BY|PT_GUARDED_BY|LOCKS_EXCLUDED|EXCLUSIVE_LOCKS_REQUIRED|SHARED_LOCKS_REQUIRED|LOCKABLE|SCOPED_LOCKABLE|EXCLUSIVE_LOCK_FUNCTION|SHARED_LOCK_FUNCTION|UNLOCK_FUNCTION|LOCK_RETURNED|NO_THREAD_SAFETY_ANALYSIS|ACQUIRED_AFTER|ACQUIRED_BEFORE|ASSERT_EXCLUSIVE_LOCK|ASSERT_SHARED_LOCK)\(/ABSL_\1(/g'

COPY autonav_bringup src/autonav_bringup
COPY autonav_controller src/autonav_controller
COPY autonav_description src/autonav_description
COPY autonav_firmware src/autonav_firmware
COPY autonav_imu src/autonav_imu
COPY autonav_localization src/autonav_localization
COPY autonav_navigation src/autonav_navigation
COPY autonav_lidar src/autonav_lidar
COPY autonav_msgs src/autonav_msgs
COPY autonav_utils src/autonav_utils

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

ENV PATH="/opt/venv/bin:$PATH"

COPY docker/entrypoint.bash /entrypoint.bash
# COPY fastdds.xml /fastdds.xml
RUN chmod +x /entrypoint.bash

ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
