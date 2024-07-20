FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp 

COPY /autonav_controller /colcon_ws/src/autonav_controller

COPY autonav_entrypoint.bash /autonav_entrypoint.bash

RUN chmod +x /autonav_entrypoint.bash

WORKDIR /colcon_ws

RUN source /opt/ros/humble/setup.sh \
    && colcon build --executor sequential \
    && rm -rf log/ build/ src/ \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/autonav_entrypoint.bash"]

CMD ["bash"]