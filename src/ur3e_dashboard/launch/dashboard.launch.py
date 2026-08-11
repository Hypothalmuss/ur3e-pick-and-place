from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        # Loopback by default: the dashboard can command the robot, so it is
        # not exposed on the network without an explicit choice.
        DeclareLaunchArgument('bind', default_value='127.0.0.1'),
        Node(
            package='ur3e_dashboard',
            executable='dashboard_server',
            name='dashboard_server',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'bind': LaunchConfiguration('bind'),
            }],
        ),
    ])
