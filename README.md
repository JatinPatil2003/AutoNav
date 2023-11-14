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
- [⚙️ Modules](#️-modules)
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

**AutoNav** is an advanced autonomous mobile robot designed to navigate through rooms independently. This project showcases the integration of cutting-edge technologies in robotics, including the **Robot Operating System (ROS)**, **Simultaneous Localization and Mapping (SLAM)**, and the **ROS Navigation Stack**.

**AutoNav** is a testament to the possibilities in autonomous robotic systems, aiming to pave the way for future innovations in this field. This project is suitable for enthusiasts and professionals alike, seeking to explore and expand in the realm of autonomous robotics.

---

## 📦 Features

 - **Autonomous Navigation:** 
   ```
   Utilizes SLAM and ROS Navstack for intelligent pathfinding and obstacle avoidance in various environments.
   ```

 - **Advanced Perception:** 
   ```
   Employs a depth camera for accurate environmental perception and dynamic 3D mapping.
   ```

 - **Sensor Fusion:** 
   ```
   Integrates Inertial Measurement Unit (IMU) sensors for enhanced movement precision and stability.
   ```

 - **ROS Integration:** 
   ```
   Built on ROS, providing a robust and flexible framework for robot software development.
   ```


---


## 📂 Repository Structure

```sh
└── AutoNav/
    ├── CMakeLists.txt
    ├── config/
    │   ├── common_costmap.yaml
    │   ├── dwa_planner.yaml
    │   ├── global_costmap.yaml
    │   ├── local_costmap.yaml
    │   └── move_base.yaml
    ├── launch/
    │   ├── amcl.launch
    │   ├── autonav.launch
    │   ├── controller.launch
    │   ├── controller.yaml
    │   ├── display.launch
    │   ├── encoderticks.launch
    │   ├── gazebo.launch
    │   ├── i2c.launch
    │   ├── laptop.launch
    │   ├── mapping.launch
    │   ├── move_base.launch
    │   ├── navigation.launch
    │   ├── robot.launch
    │   ├── trial.launch
    │   ├── urdf.rviz
    │   └── x4.launch
    ├── maps/
    │   ├── L412.pgm
    │   ├── L412.yaml
    │   ├── house_map.pgm
    │   ├── house_map.yaml
    │   ├── myworldmap.pgm
    │   └── myworldmap.yaml
    ├── meshes/
    │   ├── base_link.stl
    │   ├── lwheel_v1_1.stl
    │   ├── root.stl
    │   ├── rwheel_v1_1.stl
    │   └── ydlidar_x4_v2_1.stl
    ├── rviz/
    │   ├── autonav.rviz
    │   ├── hardwarerviz.rviz
    │   ├── laserrviz.rviz
    │   ├── navigation.rviz
    │   └── odomrviz.rviz
    ├── src/
    │   ├── Arduino_codes/
    │   │   ├── left/
    │   │   ├── leftwheelpub/
    │   │   ├── right/
    │   │   ├── rightwheelpub/
    │   │   └── trial/
    │   ├── i2c.py
    │   ├── left.py
    │   ├── left_cmdvel.py
    │   ├── leftwheelsub.py
    │   ├── odometrypub.py
    │   ├── right.py
    │   ├── right_cmdvel.py
    │   ├── rightwheelsub.py
    │   └── transformbroadcast.py
    ├── urdf/
    │   ├── autonav.gazebo
    │   ├── autonav.trans
    │   ├── autonav.xacro
    │   └── materials.xacro
    └── worlds/
        ├── house.world
        └── myworld.world

```

---


## ⚙️ Modules

<details closed><summary>Root</summary>

| File                                                                                 | Summary                   |
| ---                                                                                  | ---                       |
| [CMakeLists.txt](https://github.com/JatinPatil2003/AutoNav/blob/main/CMakeLists.txt) | This CMakeLists.txt file is for configuring the build environment of the "autonav_description" ROS package, focusing on dependencies, ROS message generation, dynamic reconfigure parameters, and basic build and installation settings. |

</details>

<details closed><summary>Urdf</summary>

| File                                                                                        | Summary                   |
| ---                                                                                         | ---                       |
| [autonav.gazebo](https://github.com/JatinPatil2003/AutoNav/blob/main/urdf/autonav.gazebo)   | This XML file configures a Gazebo simulation for the "autonav" robot, defining its physical properties, sensors (like the Hokuyo lidar), and plugins for ROS integration and control. |
| [autonav.xacro](https://github.com/JatinPatil2003/AutoNav/blob/main/urdf/autonav.xacro)     | This XML file defines the URDF (Unified Robot Description Format) for the "autonav" robot, detailing its physical components, links, joints, materials, and geometrical properties for simulation and visualization. |
| [materials.xacro](https://github.com/JatinPatil2003/AutoNav/blob/main/urdf/materials.xacro) | This XML snippet defines a set of custom materials with specified RGBA color values for the "autonav" robot, including silver, iron cast, nylon white, and ABS white materials for use in its visualization and simulation. |
| [autonav.trans](https://github.com/JatinPatil2003/AutoNav/blob/main/urdf/autonav.trans)     | This XML file defines the transmission configurations for the "autonav" robot, specifically for two joints named "Revolute 2" and "Revolute 3". Each transmission uses a simple interface for effort-based control, linking the joints to their respective actuators with a mechanical reduction ratio of 1. |

</details>

<details closed><summary>Worlds</summary>

| File                                                                                      | Summary                   |
| ---                                                                                       | ---                       |
| [myworld.world](https://github.com/JatinPatil2003/AutoNav/blob/main/worlds/myworld.world) | The XML-like content appears to be a simulation world description, possibly for a robotics simulation platform like Gazebo or a similar simulator. This format, known as SDF (Simulation Description Format), is used to describe objects, models, environments, physics properties, and other simulation parameters. |
| [house.world](https://github.com/JatinPatil2003/AutoNav/blob/main/worlds/house.world)     | This SDF (Simulation Description Format) file sets up a simulation environment in Gazebo. It includes a global light source (sun), a ground plane, and a turtlebot3_house model. The physics are configured with specific parameters like update rate, step size, and solver settings. Additionally, it defines the scene's ambient and background lighting and enables shadows. Finally, it configures a user camera with a specific pose and an orbit view controller for the GUI. |

</details>

<details closed><summary>Maps</summary>

| File                                                                                        | Summary                   |
| ---                                                                                         | ---                       |
| [myworldmap.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/maps/myworldmap.yaml) | Map based on Gazebo custom world|
| [L412.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/maps/L412.yaml)             | Map based on Hostel room |
| [house_map.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/maps/house_map.yaml)   | Map based on TurtleBot3 house world |

</details>

<details closed><summary>Rviz</summary>

| File                                                                                            | Summary                   |
| ---                                                                                             | ---                       |
| [autonav.rviz](https://github.com/JatinPatil2003/AutoNav/blob/main/rviz/autonav.rviz)           | Main rviz config file for autonav |
| [hardwarerviz.rviz](https://github.com/JatinPatil2003/AutoNav/blob/main/rviz/hardwarerviz.rviz) | rviz config file for hardware testing |
| [laserrviz.rviz](https://github.com/JatinPatil2003/AutoNav/blob/main/rviz/laserrviz.rviz)       | rviz config file for visualization of laser data |
| [navigation.rviz](https://github.com/JatinPatil2003/AutoNav/blob/main/rviz/navigation.rviz)     | rviz config file for navigation |
| [odomrviz.rviz](https://github.com/JatinPatil2003/AutoNav/blob/main/rviz/odomrviz.rviz)         | rviz config file for testing odometry |

</details>

<details closed><summary>Src</summary>

| File                                                                                                   | Summary                   |
| ---                                                                                                    | ---                       |
| [right_cmdvel.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/right_cmdvel.py)             | Python script for publishing right wheel velocity |
| [odometrypub.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/odometrypub.py)               | Python script for publishing Odometry |
| [left.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/left.py)                             | Python script for publishing left motor encoder counts |
| [right.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/right.py)                           | Python script for publishing right motor encoder counts |
| [leftwheelsub.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/leftwheelsub.py)             | Python script for testing left wheel counts |
| [i2c.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/i2c.py)                               | HTTPStatus Exception: 404 |
| [rightwheelsub.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/rightwheelsub.py)           | Python script for testing right wheel counts |
| [transformbroadcast.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/transformbroadcast.py) | Python script for publishing transform between odom from and base_link |
| [left_cmdvel.py](https://github.com/JatinPatil2003/AutoNav/blob/main/src/left_cmdvel.py)               | Python script for publishing left wheel velocity |

</details>

<details closed><summary>Trial</summary>

| File                                                                                               | Summary                   |
| ---                                                                                                | ---                       |
| [trial.ino](https://github.com/JatinPatil2003/AutoNav/blob/main/src/Arduino_codes/trial/trial.ino) | Arduino script for testin I2C communication |

</details>

<details closed><summary>Leftwheelpub</summary>

| File                                                                                                                    | Summary                   |
| ---                                                                                                                     | ---                       |
| [leftwheelpub.ino](https://github.com/JatinPatil2003/AutoNav/blob/main/src/Arduino_codes/leftwheelpub/leftwheelpub.ino) | Arduino script for handling left motor with rosserial |

</details>

<details closed><summary>Right</summary>

| File                                                                                               | Summary                   |
| ---                                                                                                | ---                       |
| [right.ino](https://github.com/JatinPatil2003/AutoNav/blob/main/src/Arduino_codes/right/right.ino) | Arduino script for handling right motor using I2C |

</details>

<details closed><summary>Rightwheelpub</summary>

| File                                                                                                                       | Summary                   |
| ---                                                                                                                        | ---                       |
| [rightwheelpub.ino](https://github.com/JatinPatil2003/AutoNav/blob/main/src/Arduino_codes/rightwheelpub/rightwheelpub.ino) | Arduino script for handling right motor with rosserial |

</details>

<details closed><summary>Left</summary>

| File                                                                                            | Summary                   |
| ---                                                                                             | ---                       |
| [left.ino](https://github.com/JatinPatil2003/AutoNav/blob/main/src/Arduino_codes/left/left.ino) | Arduino script for handling left motor using I2C |

</details>

<details closed><summary>Config</summary>

| File                                                                                                  | Summary                   |
| ---                                                                                                   | ---                       |
| [move_base.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/config/move_base.yaml)           | File contains configuration for move_base |
| [common_costmap.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/config/common_costmap.yaml) | File contains configuration for common costmap |
| [dwa_planner.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/config/dwa_planner.yaml)       | File contains configuration for dwa planner |
| [global_costmap.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/config/global_costmap.yaml) | File contains configuration for global costmap |
| [local_costmap.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/config/local_costmap.yaml)   | File contains configuration for local costmap |

</details>

<details closed><summary>Launch</summary>

| File                                                                                                  | Summary                   |
| ---                                                                                                   | ---                       |
| [x4.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/x4.launch)                     | Launch file for YDLidar X4 |
| [mapping.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/mapping.launch)           | Launch file to start mapping |
| [autonav.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/autonav.launch)           | Launch file to start navigation in simulation |
| [controller.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/controller.launch)     | Launch file to launch controllers |
| [robot.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/robot.launch)               | Launch file to bringup all necessary packages for Autonav in hardware on robot |
| [encoderticks.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/encoderticks.launch) | Launch file brings up odometry node and transform publishing node |
| [move_base.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/move_base.launch)       | Launch file to bringup move_base node with config files |
| [gazebo.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/gazebo.launch)             | Lauch file bringups Autonav urdf in gazebo |
| [trial.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/trial.launch)               | Launch file to bringup AMCL node |
| [i2c.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/i2c.launch)                   | Launch file bringups all necessary nodes to start communiaction between Arduino and Raspberry-pi |
| [controller.yaml](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/controller.yaml)         | Config file for controllers |
| [laptop.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/laptop.launch)             | Launch file to bringup all necessary packages for Autonav on Master computer |
| [amcl.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/amcl.launch)                 | launch file bringups amcl node with configs |
| [navigation.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/navigation.launch)     | Launch file to bringup all nodes for navigation |
| [display.launch](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/display.launch)           | Launch file to bringup AutoNav in RVIZ |
| [urdf.rviz](https://github.com/JatinPatil2003/AutoNav/blob/main/launch/urdf.rviz)                     | RVIZ config file for displaying autonav in RVIZ |

</details>

---

## 🚀 Getting Started

***Dependencies***

Please ensure you have the following dependencies installed on your system:

`- ℹ️ ROS-1 Noetic-Desktop-full [On Master Computer]`

`- ℹ️ ROS-1 Noetic-Desktop [On Robot Computer]`


### 🔧 Installation

1. Clone the AutoNav repository:
```sh
git clone https://github.com/JatinPatil2003/AutoNav.git
```

2. Change to the project directory:
```sh
cd AutoNav
```

3. Install the dependencies:
```sh
► sudo apt install ros-noetic-navigation* ros-noetic-slam-gmapping* 
► sudo apt install ros-noetic-teleop-twist-keyboard
► pip install smbus
```

### 🤖 Running AutoNav

 - On Robot Computer
  
    ```bash
    ► roslaunch autonav_description robot.launch
    ```

 - On Master Computer
  
    ```bash
    ► roslaunch autonav_description laptop.launch
    ```

### 🧪 Tests
```sh
► Give goal position using rviz form master computer
```

---


## 🛣 Project Roadmap

> - [X] `ℹ️  Import urdf form fusion360`
> - [X] `ℹ️  Implement Mapping and Navigation in Simulation`
> - [X] `ℹ️  Implement Mapping and Naviagtion using Hardware`
> - [ ] `ℹ️  Create python script to send goal pose`
> - [ ] `ℹ️  Implement sensor fusion using IMU and Odometry`
> - [ ] `ℹ️  Implement perception using depth camera and various algorithums`
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


This project is protected under the **```MIT```** License. For more details, refer to the [LICENSE](https://github.com/JatinPatil2003/AutoNav/blob/master/LICENSE) file.

---

## 👏 Acknowledgments

- [Fusion360 to URDF Plugin](https://github.com/JatinPatil2003/Fusion2Urdf_plugin)

[**Return**](#Top)

---

