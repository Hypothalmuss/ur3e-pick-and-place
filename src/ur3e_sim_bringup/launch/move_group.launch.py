import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name: str, file_path: str) -> dict | None:
    absolute_file_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(absolute_file_path) as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    package = "ur3e_sim_bringup"

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(package), "urdf", "ur3e_with_effector.urdf.xacro"]),
        " ur_type:=", ur_type,
        " sim_gazebo:=true",
        " simulation_controllers:=",
        PathJoinSubstitution([FindPackageShare(package), "config", "ros2_controllers.yaml"]),
    ])

    robot_description_semantic = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(package), "config", "moveit", "ur3e.srdf"]),
    ])

    robot_description_planning = {
        "robot_description_planning": load_yaml(package, os.path.join("config", "moveit", "joint_limits.yaml"))
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(package, os.path.join("config", "moveit", "ompl_planning.yaml"))
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(package, os.path.join("config", "moveit", "moveit_controllers.yaml"))
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    kinematics_param = PathJoinSubstitution([
        FindPackageShare(package), "config", "moveit", "kinematics.yaml"
    ])

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "/tmp/warehouse_ros.sqlite",
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"robot_description_semantic": ParameterValue(robot_description_semantic, value_type=str)},
            {"publish_robot_description_semantic": True},
            kinematics_param,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package), "config", "moveit", "view_robot.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"robot_description_semantic": ParameterValue(robot_description_semantic, value_type=str)},
            {"use_sim_time": use_sim_time},
        ],
    )

    scene_initializer_node = Node(
        package=package,
        executable="scene_initializer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [move_group_node, rviz_node, scene_initializer_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e", description="UR robot series"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz2"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
