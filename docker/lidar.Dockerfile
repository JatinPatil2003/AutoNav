# Lidar container: runs the autonav_lidar driver against /dev/lidar
# (stable udev symlink set up by 99-autonav.rules / install_udev_rules.sh).
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinvpatil/autonav:base
FROM ${BASE_IMAGE}

ENV LIDAR_SERIAL_PORT=/dev/lidar
ENV LIDAR_SERIAL_BAUDRATE=1000000
ENV LIDAR_FRAME_ID=laser

CMD ["bash", "-c", "ros2 launch autonav_lidar rplidar_s2_launch.py serial_port:=${LIDAR_SERIAL_PORT} serial_baudrate:=${LIDAR_SERIAL_BAUDRATE} frame_id:=${LIDAR_FRAME_ID}"]
