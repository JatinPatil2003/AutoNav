# Localization container: robot_localization EKF, fusing IMU + wheel odometry.
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinpatil2003/autonav:base
FROM ${BASE_IMAGE}

CMD ["ros2", "launch", "autonav_localization", "ekf.launch.py"]
