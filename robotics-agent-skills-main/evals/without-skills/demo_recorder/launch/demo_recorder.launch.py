from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('demo_recorder'),
        'config',
        'demo_recorder.yaml'
    )

    return LaunchDescription([
        Node(
            package='demo_recorder',
            executable='demo_recorder_node',
            name='demo_recorder',
            parameters=[config],
            output='screen',
        ),
    ])
