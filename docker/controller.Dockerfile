# Controller container: robot_state_publisher + ros2_control_node, loading
# the autonav_firmware hardware_interface plugin that talks to the ESP32
# over Modbus/serial on /dev/esp32.
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinvpatil/autonav:base
FROM ${BASE_IMAGE}

CMD ["ros2", "launch", "autonav_bringup", "autonav_control.launch.py"]
