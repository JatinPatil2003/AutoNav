# Minimal joystick teleop container. Deliberately standalone (ros:humble,
# builds only autonav_controller) rather than FROM the kilted base image —
# this runs on an operator's machine for manual driving, not on the robot
# alongside the rest of the modular stack.
FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY autonav_controller /colcon_ws/src/autonav_controller

WORKDIR /colcon_ws

RUN source /opt/ros/humble/setup.bash \
    && colcon build --executor sequential \
    && rm -rf log/ build/ src/

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /colcon_ws/install/setup.bash' >> /root/.bashrc

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /colcon_ws/install/setup.bash && ros2 launch autonav_controller joystick_teleop.launch.py"]
