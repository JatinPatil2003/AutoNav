FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control*


RUN apt-get update && apt-get install -y \
    # ros-humble-nav* \
    # ros-humble-slam-toolbox* \
    ros-humble-robot-localization* \
    ros-humble-xacro 
    # ros-humble-rmw-cyclonedds-cpp \
    # ros-humble-demo-nodes-py* \
    # ros-humble-teleop* \
    # ros-humble-joy* \
    # ros-humble-cartographer* 

RUN apt-get update && apt-get install -y \
    python3-pip \
    && python3 -m pip install -U \
    smbus \
    pyserial \
    pynput

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN apt-get update && apt-get install -y \
    ros-humble-rviz2*

COPY /autonav_controller /colcon_ws/src/autonav_controller

COPY /autonav_description /colcon_ws/src/autonav_description

COPY /autonav_firmware /colcon_ws/src/autonav_firmware

COPY /autonav_bringup /colcon_ws/src/autonav_bringup

COPY /autonav_localization /colcon_ws/src/autonav_localization

WORKDIR /colcon_ws

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install'

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

CMD bash