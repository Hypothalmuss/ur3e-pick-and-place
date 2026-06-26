from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3e_perception',
            executable='perception_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
