FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-nav* \
    ros-humble-ros2-control* \
    ros-humble-slam-toolbox* \
    ros-humble-example-interfaces* \
    ros-humble-robot-localization* \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cartographer* \
    ros-humble-rviz2*

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

COPY /autonav_controller /colcon_ws/src/autonav_controller

COPY /autonav_description /colcon_ws/src/autonav_description

COPY /autonav_firmware /colcon_ws/src/autonav_firmware

COPY /autonav_bringup /colcon_ws/src/autonav_bringup

COPY /autonav_localization /colcon_ws/src/autonav_localization

COPY /autonav_navigation /colcon_ws/src/autonav_navigation

COPY /bno055 /colcon_ws/src/bno055

COPY /ydlidar_ros2_driver /colcon_ws/src/ydlidar_ros2_driver

WORKDIR /colcon_ws

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install'

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

CMD bash