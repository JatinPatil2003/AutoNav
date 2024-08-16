import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_autonav_localization = get_package_share_directory('autonav_localization')
    robot_localization_file_path = os.path.join(pkg_autonav_localization, 'config/ekf.yaml')

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path])

    return LaunchDescription([
        start_robot_localization_cmd
    ])
