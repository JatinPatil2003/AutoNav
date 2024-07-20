import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    autonav_description_dir = get_package_share_directory("autonav_description")
    autonav_localization_dir = get_package_share_directory("autonav_localization")
    pkg_bno055 = get_package_share_directory('bno055')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')

    use_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
    )

    rviz = LaunchConfiguration("rviz")

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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(autonav_description_dir, "rviz", "display.rviz")],
        condition=IfCondition(rviz),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_bno055, 'launch', 'bno055.launch.py'),
        )
    ) 
  
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py'),
        )
    )

    controller_params_file = os.path.join(
        get_package_share_directory("autonav_controller"),
        "config",
        "autonav_controllers.yaml",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    autonav_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["autonav_controller", "--controller-manager", "/controller_manager"],
        remappings=[
            ("/autonav_controller/cmd_vel_unstamped", "/autonav_controller/cmd_vel"),
        ],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[autonav_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    pid_controller = Node(
        package="autonav_controller",
        executable="pid_controller.py",
        output="screen",
    )

    lidar_republisher = Node(
        package="autonav_controller",
        executable="lidar_republisher.py",
        output="screen",
    )

    velocity_republisher = Node(
        package="autonav_controller",
        executable="cmd_vel_republisher.py",
        output="screen",
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(autonav_localization_dir, 'launch', 'ekf.launch.py'),
        )
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            robot_state_publisher_node,
            rviz_node,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            # pid_controller,
            imu,
            lidar,
            ekf,
            lidar_republisher,
            velocity_republisher,
        ]
    )
