import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonav_description_dir = get_package_share_directory('autonav_description')

    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
    )

    rviz = LaunchConfiguration('rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(autonav_description_dir, 'rviz', 'display.rviz')],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            rviz_node,
        ]
    )
