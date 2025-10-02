FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf-transformations \
    uvicorn 

RUN apt-get update && apt-get install -y python3-pip \
    && python3 -m pip install -U \
    smbus \
    pyserial==3.4 \
    pynput \
    fastapi \
    pymongo


RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install
    
RUN apt-get update && apt-get install -y \
    network-manager \
    docker.io

COPY /autonav_bringup /colcon_ws/src/autonav_bringup

COPY /autonav_controller /colcon_ws/src/autonav_controller

COPY /autonav_description /colcon_ws/src/autonav_description

COPY /autonav_firmware /colcon_ws/src/autonav_firmware

COPY /autonav_localization /colcon_ws/src/autonav_localization

COPY /autonav_navigation /colcon_ws/src/autonav_navigation

COPY /bno055 /colcon_ws/src/bno055

COPY /ydlidar_ros2_driver /colcon_ws/src/ydlidar_ros2_driver

COPY autonav_entrypoint.bash /autonav_entrypoint.bash

RUN chmod +x /autonav_entrypoint.bash

WORKDIR /colcon_ws

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install'

# RUN source /opt/ros/humble/setup.sh \
#     && colcon build \
#     && rm -rf log/ build/ src/ \
#     && apt-get autoremove -y \
#     && apt-get autoclean -y \
#     && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/autonav_entrypoint.bash"]

COPY /WebServer /WebServer

WORKDIR /WebServer

EXPOSE 8000

CMD ["uvicorn", "index:app", "--host", "0.0.0.0", "--port", "8000"]