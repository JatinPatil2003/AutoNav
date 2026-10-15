import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('autonav_firmware'),
        'config',
        'esp32_bridge_params.yaml',
    )

    node = Node(
        package='autonav_firmware',
        executable='esp32_bridge_node',
        output='screen',
        parameters=[config],
        # arguments=[
        #     '--ros-args',
        #     '--log-level', 'DEBUG'
        # ],
    )

    return LaunchDescription([node])
