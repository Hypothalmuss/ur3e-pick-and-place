from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('ur3e_sim_bringup'), 'urdf', 'ur3e_with_effector.urdf.xacro']),
        ' ur_type:=', ur_type,
        ' sim_gazebo:=true',
        ' simulation_controllers:=',
        PathJoinSubstitution([FindPackageShare('ur3e_sim_bringup'), 'config', 'ros2_controllers.yaml']),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py']),
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('ur3e_sim_bringup'), 'worlds', 'ur3e_workcell.world'
                ]),
                'verbose': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py']),
            ]),
            condition=IfCondition(gazebo_gui),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'ur3e',
            ],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        ),
    ])
