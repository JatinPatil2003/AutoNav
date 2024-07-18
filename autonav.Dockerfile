FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-nav* \
    ros-humble-ros2-control* \
    ros-humble-slam-toolbox* \
    ros-humble-example-interfaces* \
    ros-humble-robot-localization* \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cartographer* \
    ros-humble-rviz2* \
    ros-humble-cv-bridge

RUN apt-get update && apt-get install -y \
    python3-pip \
    && python3 -m pip install -U \
    smbus \
    pyserial==3.4 \
    pynput

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

COPY /serial /serial

RUN source /opt/ros/humble/setup.sh \
    && cd /serial \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

COPY /autonav_controller /colcon_ws/src/autonav_controller

COPY /autonav_description /colcon_ws/src/autonav_description

COPY /autonav_firmware /colcon_ws/src/autonav_firmware

COPY /autonav_bringup /colcon_ws/src/autonav_bringup

COPY /autonav_localization /colcon_ws/src/autonav_localization

COPY /autonav_navigation /colcon_ws/src/autonav_navigation

COPY /autonav_perception /colcon_ws/src/autonav_perception

COPY /bno055 /colcon_ws/src/bno055

COPY /ydlidar_ros2_driver /colcon_ws/src/ydlidar_ros2_driver

COPY autonav_entrypoint.bash /autonav_entrypoint.bash

RUN chmod +x /autonav_entrypoint.bash

WORKDIR /colcon_ws

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install'

# RUN source /opt/ros/humble/setup.sh \
#     && colcon build --executor sequential \
#     && rm -rf log/ build/ src/ \
#     && apt-get autoremove -y \
#     && apt-get autoclean -y \
#     && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/autonav_entrypoint.bash"]

CMD ["bash"]