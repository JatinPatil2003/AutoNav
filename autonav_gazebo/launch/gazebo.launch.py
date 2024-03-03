import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autonav_description_dir = get_package_share_directory("autonav_description")
    autonav_gazebo_dir = get_package_share_directory("autonav_gazebo")
    autonav_description_share = os.path.join(
        get_package_prefix("autonav_description"), "share"
    )
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", autonav_description_share)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("autonav_description"),
                    "urdf",
                    "autonav.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    world = os.path.join(
        autonav_gazebo_dir,
        'worlds',
        'gazebo_world.world'
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py"),
        ),
        launch_arguments={'world': world}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "autonav",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            env_var,
            start_gazebo_server,
            start_gazebo_client,
            robot_state_publisher_node,
            spawn_robot,
        ]
    )
