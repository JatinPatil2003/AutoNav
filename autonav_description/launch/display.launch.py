import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    autonav_description_dir = get_package_share_directory('autonav_description')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('autonav_description'),
                    'urdf',
                    'autonav.xacro',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(autonav_description_dir, 'rviz', 'display.rviz')],
    )

    controller_params_file = os.path.join(get_package_share_directory('autonav_controller'),'config','autonav_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description,
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=7.0, actions=[controller_manager])

    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            delayed_controller_manager,
        ]
    )
