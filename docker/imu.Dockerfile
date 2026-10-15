# IMU container: runs the autonav_imu driver against /dev/imu
# (stable udev symlink set up by 99-autonav.rules / install_udev_rules.sh).
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinvpatil/autonav:base
FROM ${BASE_IMAGE}

CMD ["ros2", "launch", "autonav_imu", "autonav_imu.launch.py"]
