from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3e_motion',
            executable='motion_executor',
            output='screen',
        ),
        Node(
            package='ur3e_motion',
            executable='pick_place_bt_node',
            output='screen',
        ),
        Node(
            package='ur3e_motion',
            executable='pick_place_orchestrator',
            output='screen',
        ),
    ])
