# Always-on utils container: cmd_vel safety-timeout watchdog + system
# liveness heartbeat. Deliberately decoupled from the controller/navigation
# containers' own lifecycles — see autonav_utils/launch/autonav_utils.launch.py.
#
# Defaults to the published base image; override BASE_IMAGE to build
#   against a locally built one while iterating on docker/base.Dockerfile.
ARG BASE_IMAGE=jatinpatil2003/autonav:base
FROM ${BASE_IMAGE}

CMD ["ros2", "launch", "autonav_utils", "autonav_utils.launch.py"]
