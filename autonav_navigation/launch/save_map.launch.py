from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

MAP_NAME = input('Enter map file name:')


def generate_launch_description():
    # Declare a launch argument for the map file name and directory
    map_file_name_arg = DeclareLaunchArgument(
        'map_file_name',
        default_value=f'{MAP_NAME}',
        description='Name of the map file to save.')

    map_directory_arg = DeclareLaunchArgument(
        'map_directory',
        default_value=f'/colcon_ws/src/autonav_navigation/maps/{MAP_NAME}',
        description='Directory to save the map file.')

    map_directory = LaunchConfiguration('map_directory')

    # Define the map_saver_cli node execution
    map_saver_cli = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f',
             map_directory],
        shell=True
    )

    # Return the launch description
    return LaunchDescription([
        map_file_name_arg,
        map_directory_arg,
        map_saver_cli
    ])
