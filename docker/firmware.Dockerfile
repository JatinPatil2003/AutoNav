# Firmware bridge container: esp32_bridge_node, the serial middleware that
# talks framed commands/telemetry to the ESP32 over /dev/esp. Replaces the
# old micro-ROS agent container now that the ESP32 firmware speaks this
# custom protocol instead of micro-ROS.
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinpatil2003/autonav:base
FROM ${BASE_IMAGE}

CMD ["ros2", "launch", "autonav_firmware", "esp32_bridge.launch.py"]
