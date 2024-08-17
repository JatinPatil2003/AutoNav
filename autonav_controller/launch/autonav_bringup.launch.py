import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition


def generate_launch_description():
    autonav_description_dir = get_package_share_directory('autonav_description')

    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
    )

    rviz = LaunchConfiguration('rviz')

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(autonav_description_dir, 'rviz', 'display.rviz')],
        condition=IfCondition(rviz),
    )

    controller_params_file = os.path.join(
        get_package_share_directory('autonav_controller'),
        'config',
        'autonav_controllers.yaml',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    autonav_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['autonav_controller', '--controller-manager', '/controller_manager'],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[autonav_spawner],
        )
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    pid_controller = Node(
        package='autonav_controller',
        executable='pid_controller.py',
        output='screen',
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            robot_state_publisher_node,
            rviz_node,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            pid_controller,
        ]
    )
