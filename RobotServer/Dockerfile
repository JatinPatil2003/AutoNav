# Built with the repo root as context (not ./RobotServer) — see
# docker-compose.yml's `context: .` / `dockerfile: RobotServer/Dockerfile`
# and .circleci/config.yml's build invocation. Every path below is
# root-relative for that reason. RobotApp (the local HMI PWA) is built and
# served by its own container now — see RobotApp/Dockerfile — fronted by
# nginx, which reverse-proxies /api back to this process on localhost:8000.

# Deliberately its own lean image rather than FROM the heavy
# jatinpatil2003/autonav:base (which carries nav2-bringup, slam-toolbox,
# cartographer-ros, rviz2 — none of which RobotServer runs itself; it only
# talks to those nodes over topics/services/actions). RobotServer needs:
# Node.js (to run) + the ROS 2 Lyrical client libraries and message
# packages rclnodejs binds against at npm-install time.
FROM ros:lyrical AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=lyrical

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" > /etc/apt/sources.list.d/nodesource.list \
    && apt-get update && apt-get install -y -t nodistro nodejs \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Build stage: compiles TypeScript + rclnodejs's native bindings (needs
# build-essential/python3/cmake, which the runtime stage below doesn't).
# -----------------------------
FROM base AS build

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY RobotServer/package*.json ./
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && npm ci

COPY RobotServer/. .
RUN npm run build

# -----------------------------
# Runtime stage
# -----------------------------
FROM base

WORKDIR /app
ENV NODE_ENV=production

COPY --from=build /app/node_modules ./node_modules
COPY --from=build /app/dist ./dist
COPY RobotServer/package*.json ./

EXPOSE 8000

CMD ["bash", "-c", "source /opt/ros/lyrical/setup.bash && node dist/index.js"]
