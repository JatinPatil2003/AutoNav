
<div align="center">
<h1 align="center">

![AutoNavv85](https://github.com/JatinPatil2003/AutoNav/assets/89979346/352fa4d7-270b-43e4-bd51-bcee4377b07a)

<br>AUTONAV</h1>
<h4>AutoNav is an innovative autonomous mobile robot designed for efficient room navigation. Leveraging the Robot Operating System (ROS) along with SLAM (Simultaneous Localization and Mapping) and ROS Navstack.

It excels in autonomous pathfinding and obstacle avoidance. At its core, AutoNav employs IMU-based sensor fusion for precise movement control and utilizes a depth camera for accurate perception and 3D mapping. This integration of advanced technologies enables AutoNav to navigate complex environments with ease.</h4>

<p align="center">
<img src="https://img.shields.io/badge/YAML-CB171E.svg?style=apps&logo=YAML&logoColor=white" alt="YAML" />
<img src="https://img.shields.io/badge/Python-3776AB.svg?style=apps&logo=Python&logoColor=white" alt="Python" />
<img src="https://img.shields.io/badge/C%2B%2B-00599C.svg?style=apps&logo=c%2B%2B&logoColor=white" alt="C++" />
<img src="https://img.shields.io/badge/CMake-064F8C.svg?style=apps&logo=CMake&logoColor=white" alt="CMake" />

[![AutoNav_CI](https://github.com/JatinPatil2003/AutoNav/actions/workflows/AutoNav_CI.yml/badge.svg)](https://github.com/JatinPatil2003/AutoNav/actions/workflows/AutoNav_CI.yml)

</p>
<img src="https://img.shields.io/github/license/JatinPatil2003/AutoNav?style=apps&color=5D6D7E" alt="GitHub license" />
<img src="https://img.shields.io/github/last-commit/JatinPatil2003/AutoNav?style=apps&color=5D6D7E" alt="git-last-commit" />
<img src="https://img.shields.io/github/commit-activity/m/JatinPatil2003/AutoNav?style=apps&color=5D6D7E" alt="GitHub commit activity" />
<img src="https://img.shields.io/github/languages/top/JatinPatil2003/AutoNav?style=apps&color=5D6D7E" alt="GitHub top language" />
</div>

---

## 📖 Table of Contents
- [📖 Table of Contents](#-table-of-contents)
- [📍 Overview](#-overview)
- [📦 Features](#-features)
- [📂 Repository Structure](#-repository-structure)
- [🚀 Getting Started](#-getting-started)
  - [🔧 Installation](#-installation)
  - [🤖 Running AutoNav](#-running-autonav)
  - [🧪 Tests](#-tests)
- [🛣 Project Roadmap](#-project-roadmap)
- [🤝 Contributing](#-contributing)
    - [*Contributing Guidelines*](#contributing-guidelines)
- [📄 License](#-license)
- [👏 Acknowledgments](#-acknowledgments)

---


## 📍 Overview

<div align="center">
  
![9-removebg-preview](https://github.com/JatinPatil2003/AutoNav/assets/89979346/cc90858f-f488-4fa3-9c13-04c99cbd753a)

</div>

**AutoNav** is an advanced autonomous mobile robot designed to navigate through rooms independently. This project showcases the integration of cutting-edge technologies in robotics, including the **Robot Operating System (ROS)**, **Simultaneous Localization and Mapping (SLAM)**, **Sensor Fusion** and the **ROS Navigation Stack**.

**AutoNav** is a testament to the possibilities in autonomous robotic systems, aiming to pave the way for future innovations in this field. This project is suitable for enthusiasts and professionals alike, seeking to explore and expand in the realm of autonomous robotics.

---

## 📦 Features

 - **Autonomous Navigation:** 
   > Utilizes SLAM and ROS Navstack for intelligent pathfinding and obstacle avoidance in various environments.

 - **Sensor Fusion:** 
   >Integrates Inertial Measurement Unit (IMU) sensors for enhanced movement precision and stability.

 - **Hardware Integration:** 
   > Designed to work seamlessly with a range of robotic hardware, enabling easy integration of sensors, actuators, and other essential components for autonomous navigation.


---


## 📂 Repository Structure

```sh
└── AutoNav/
    ├── docker/                        # one Dockerfile per modular ROS robot container
    │   ├── base.Dockerfile            # shared colcon workspace + venv (build this first)
    │   ├── lidar.Dockerfile           # rplidar_ros
    │   ├── imu.Dockerfile             # autonav_imu
    │   ├── controller.Dockerfile      # ros2_control + autonav_firmware (ESP32)
    │   ├── localization.Dockerfile    # robot_localization EKF
    │   ├── navigation.Dockerfile      # Nav2 / slam_toolbox / cartographer
    │   ├── firmware.Dockerfile        # esp32_bridge_node serial middleware
    │   ├── utils.Dockerfile           # always-on cmd_vel watchdog + heartbeat
    │   ├── joystick.Dockerfile        # standalone teleop image (ros:humble)
    │   └── entrypoint.bash
    ├── docker-compose.yml             # always-on robot stack: autonav-mongodb, autonav-robotserver, autonav-lidar, autonav-imu, autonav-controller, autonav-localization, autonav-navigation, autonav-firmware, autonav-utils
    ├── docker-compose.dev.yml         # rviz viewers + interactive autonav-save_map, for a dev machine
    ├── autonav_bringup
    │   ├── CMakeLists.txt
    │   └── launch
    │       ├── autonav_bringup.launch.py   # bare-metal all-in-one dev launch (non-docker)
    │       ├── autonav_control.launch.py   # used by docker/controller.Dockerfile
    │       ├── view_rviz.launch.py
    │       ├── view_rviz_cartographer.launch.py
    │       ├── view_rviz_navigation.launch.py
    │       └── view_rviz_slam.launch.py
    ├── autonav_controller
    │   ├── CMakeLists.txt
    │   ├── autonav_controller
    │   │   ├── control.py
    │   │   ├── odom_logger.py
    │   │   ├── pid_controller.py
    │   │   └── tune_pid.py
    │   ├── config
    │   │   └── autonav_controllers.yaml
    │   └── launch
    │       ├── controller.launch.py
    │       └── joystick_teleop.launch.py
    ├── autonav_description
    │   ├── CMakeLists.txt
    │   ├── meshes
    │   ├── rviz
    │   │   └── display.rviz
    │   └── urdf
    │       ├── autonav.xacro
    │       ├── autonav_ros2_control.xacro
    │       └── materials.xacro
    ├── autonav_firmware
    │   ├── CMakeLists.txt
    │   ├── firmware                    # ESP32 Arduino/PlatformIO sketches
    │   ├── scripts                     # standalone Modbus debug scripts
    │   └── src
    │       ├── autonav_interface.cpp          # ros2_control hardware_interface (Modbus)
    │       ├── autonav_interface_serial.cpp   # ros2_control hardware_interface (raw serial)
    │       └── motor_controller.cpp
    ├── autonav_localization
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   └── ekf.yaml
    │   └── launch
    │       └── ekf.launch.py            # used by docker/localization.Dockerfile
    ├── autonav_navigation
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── cartographer.lua
    │   │   ├── navigation.yaml
    │   │   └── slam.yaml
    │   ├── launch
    │   │   ├── cartographer.launch.py
    │   │   ├── navigation.launch.py     # used by docker/navigation.Dockerfile
    │   │   ├── save_map.launch.py
    │   │   └── slam.launch.py
    │   └── rviz
    │       ├── cartographer.rviz
    │       ├── navigation.rviz
    │       └── slam.rviz
    ├── autonav_utils                   # always-on cmd_vel watchdog, heartbeat, SLAM-time tf_pose_bridge; used by docker/utils.Dockerfile
    ├── autonav_imu                     # IMU driver, used by docker/imu.Dockerfile
    ├── rplidar_ros                     # lidar driver, used by docker/lidar.Dockerfile
    └── RobotServer                     # Node/TypeScript ROS bridge (rclnodejs) — see RobotServer/README.md
```

---

## 🚀 Getting Started

***Dependencies***

Please ensure you have the following dependencies installed on your system:

`- ℹ️ Docker `

[Installation Link](https://docs.docker.com/engine/install/ubuntu/)


### 🔧 Installation

1. Clone the AutoNav repository:
```sh
mkdir ~/colcon_ws/src/ && cd ~/colcon_ws/src
git clone https://github.com/JatinPatil2003/AutoNav.git
```

2. Change to the project directory:
```sh
cd AutoNav
```

3. Pull the modular Docker images (one per subsystem, see `docker/`):
```sh
docker compose pull
```

### 🤖 Running AutoNav

Make sure you are in the project directory:

```sh
cd colcon_ws/src/AutoNav
```

 - For the always-on robot stack (autonav-mongodb, autonav-robotserver, autonav-lidar, autonav-imu, autonav-controller, autonav-localization, autonav-navigation, autonav-firmware, autonav-utils)

    ```bash
    docker compose up -d
    ```

 - For Mapping (swap the navigation container into slam/cartographer mode)

    ```bash
    NAV_MODE=slam docker compose up autonav-navigation
    # or
    NAV_MODE=cartographer docker compose up autonav-navigation
    ```

    https://github.com/JatinPatil2003/AutoNav/assets/89979346/1e70e5b4-4566-484d-8f96-549d4326ddfe

 - For Saving a Map

    ```bash
    docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm autonav-save_map
    ```

 - For Navigation on a saved map

    ```bash
    docker compose up autonav-navigation
    ```

    https://github.com/JatinPatil2003/AutoNav/assets/89979346/8b9d2ae8-db69-4070-b972-aa64177b11e3

 - For rviz/rviz_cartographer/rviz_navigation on the host computer

    ```bash
    xhost +local:
    docker compose -f docker-compose.yml -f docker-compose.dev.yml up {autonav-rviz/autonav-rviz_cartographer/autonav-rviz_navigation}
    ```

### 🧪 Tests
```sh
► Give goal position using rviz form rviz docker conatiner
```

---


## 🛣 Project Roadmap

> - [X] `ℹ️  Import urdf form fusion360`
> - [X] `ℹ️  Implement Mapping and Navigation in Simulation`
> - [X] `ℹ️  Implement Mapping and Naviagtion using Hardware`
> - [X] `ℹ️  Implement sensor fusion using IMU and Odometry`
> - [ ] `ℹ️  Implement 3D-Mapping using depth camera`

---

## 🤝 Contributing

Contributions are welcome! Here are several ways you can contribute:

- **[Submit Pull Requests](https://github.com/JatinPatil2003/AutoNav/blob/main/CONTRIBUTING.md)**:
  
  >  Review open PRs, and submit your own PRs.

- **[Join the Discussions](https://github.com/JatinPatil2003/AutoNav/discussions)**: 
 
  >  Share your insights, provide feedback, or ask questions.

- **[Report Issues](https://github.com/JatinPatil2003/AutoNav/issues)**: 
      
  > Submit bugs found or log feature requests for JATINPATIL2003.

#### *Contributing Guidelines*

<details closed>
<summary>Click to expand</summary>

1. **Fork the Repository**: Start by forking the project repository to your GitHub account.
2. **Clone Locally**: Clone the forked repository to your local machine using a Git client.
   ```sh
   git clone https://github.com/JatinPatil2003/AutoNav.git
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear and concise message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to GitHub**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.

Once your PR is reviewed and approved, it will be merged into the main branch.

</details>

---

## 📄 License


This project is protected under the **```MIT```** License. For more details, refer to the [LICENSE](https://github.com/JatinPatil2003/AutoNav/blob/ros2/LICENSE) file.

---

## 👏 Acknowledgments

- [Fusion360 to URDF Plugin](https://github.com/JatinPatil2003/Fusion2Urdf_plugin)

- [Bno055 IMU Package](https://github.com/flynneva/bno055)

- [RPLidar ROS Package](https://github.com/Slamtec/rplidar_ros)

[**Return**](#Top)

---

