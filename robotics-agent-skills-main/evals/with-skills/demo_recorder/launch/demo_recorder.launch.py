"""
Launch file for the demo_recorder lifecycle node.

Supports launch arguments so operators can override any parameter from the
command line without editing YAML:

    ros2 launch demo_recorder demo_recorder.launch.py \\
        save_directory:=/data/demos \\
        camera_topic:=/cam/image_raw \\
        capture_rate_hz:=15.0 \\
        autostart:=true

The ``autostart`` argument triggers an automatic lifecycle transition:
  Unconfigured -> Inactive (configure) -> Active (activate)
so the node is immediately ready to accept start/stop recording service calls.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory("demo_recorder")
    default_config = os.path.join(pkg_dir, "config", "demo_recorder.yaml")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the YAML parameter file.",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="demo_recorder",
            description="Name of the lifecycle node.",
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="",
            description="Override camera topic (empty = use config file).",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="",
            description="Override joint states topic (empty = use config file).",
        ),
        DeclareLaunchArgument(
            "save_directory",
            default_value="",
            description="Override save directory (empty = use config file).",
        ),
        DeclareLaunchArgument(
            "capture_rate_hz",
            default_value="0.0",
            description="Override capture rate Hz (0 = use config file).",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate the node.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal).",
        ),
    ]

    # ------------------------------------------------------------------
    # Lifecycle node
    # ------------------------------------------------------------------
    recorder_node = LifecycleNode(
        package="demo_recorder",
        executable="demo_recorder_node",
        name=LaunchConfiguration("node_name"),
        namespace="",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    # ------------------------------------------------------------------
    # Autostart: configure -> activate on process start
    # ------------------------------------------------------------------
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration("autostart")),
    )

    # When configure succeeds (node enters 'inactive'), auto-activate
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=recorder_node,
            goal_state="inactive",
            entities=[
                LogInfo(msg="demo_recorder is configured. Activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: True,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("autostart")),
    )

    # Log when node reaches active state
    activate_log = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=recorder_node,
            goal_state="active",
            entities=[
                LogInfo(
                    msg="demo_recorder is ACTIVE and ready to record."
                ),
            ],
        ),
    )

    return LaunchDescription(
        declared_args
        + [
            recorder_node,
            configure_event,
            activate_on_configured,
            activate_log,
        ]
    )
