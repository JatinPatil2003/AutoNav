from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonav_utils',
            executable='cmd_vel_republisher',
            name='cmd_vel_republisher',
            output='screen',
        ),

        Node(
            package='autonav_utils',
            executable='system_heartbeat',
            name='system_heartbeat',
            output='screen',
        ),
    ])
