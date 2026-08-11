from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    robot_description = ParameterValue(Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('ur3e_sim_bringup'), 'urdf', 'ur3e_with_effector.urdf.xacro']),
        ' ur_type:=', ur_type,
        ' sim_gazebo:=true',
        ' simulation_controllers:=',
        PathJoinSubstitution([FindPackageShare('ur3e_sim_bringup'), 'config', 'ros2_controllers.yaml']),
        ' use_mimic_plugin:=', LaunchConfiguration('use_mimic_plugin'),
        ' use_grasp_fix:=', LaunchConfiguration('use_grasp_fix'),
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_mimic_plugin', default_value='true'),
        # On by default: the friction grasp cannot hold the cube. The knuckle is
        # position-commanded through gazebo_ros2_control, so it teleports shut
        # and cannot stall on contact - the pads bat the cube away (measured 57 mm
        # of displacement before the fingers finished closing, then a 1 m/s
        # launch). The attach-on-close plugin sidesteps that by fixing the cube
        # to the gripper. Pass use_grasp_fix:=false for the friction grasp.
        DeclareLaunchArgument('use_grasp_fix', default_value='true'),

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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.15', '--y', '0', '--z', '3.0',
                       '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
                       '--frame-id', 'world', '--child-frame-id', 'camera_link'],
        ),

        Node(
            package='ur3e_perception',
            executable='perception_node',
            output='screen',
        ),

        Node(
            package='ur3e_end_effectors',
            executable='gripper_state_node',
            output='screen',
        ),

    ])
