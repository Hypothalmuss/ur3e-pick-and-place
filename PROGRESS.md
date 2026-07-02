# PROGRESS — Bug Log & Integration History

## Feature 1: Workspace scaffold & ur3e_msgs

- 2024-06-25: Scaffold created. 6 packages (1 ament_cmake + 5 ament_python) with all message/service interfaces defined. Builds cleanly.

## Feature 2: Robot bringup (Gazebo, URDF, controllers, TF)

- 2024-06-25: Created custom xacro `ur3e_with_effector.urdf.xacro` wrapping `ur_description` (ur3e) + `robotiq_description` (2F-85 gripper).
  - Uses `gazebo_ros2_control/GazeboSystem` for both arm (6 joints) and gripper (1 joint + 5 mimic).
  - Overhead camera defined in world file with `libgazebo_ros_camera.so` plugin.
- 2024-06-25: Created `ur3e_workcell.world` with ground plane, table, pick/place zones, and overhead camera.
- 2024-06-25: Created `ros2_controllers.yaml` with `joint_state_broadcaster`, `joint_trajectory_controller`, and `gripper_controller` (GripperActionController).
- 2024-06-25: Created `bringup.launch.py` — robot_state_publisher + gzserver/gzclient + spawn + controller spawners.

## Feature 3: MoveIt2 config (SRDF, planners, move_group)

- 2026-06-26: Created `config/moveit/` with SRDF, kinematics, OMPL planners, joint limits, controllers, RViz config.
- 2026-06-26: Created `launch/move_group.launch.py` — move_group node + optional RViz2.
- MoveIt2 starts but spams errors about `robotiq_85_*_mimic` joints not found (non-fatal, comes from unconditional `<xacro:include>` in robotiq_2f_85_macro.urdf.xacro).

## Feature 4: Overhead camera + perception

- 2026-07-02: Replaced old camera with regular camera (type="camera", B8G8R8) at (0.15, 0, 3.0) with pitch=0 (looking down via OGRE -Z convention).
- 2026-07-02: Added `static_transform_publisher` for `camera_link` TF in bringup launch.
- 2026-07-02: Restored `perception_node.py` with orange cube detection via `/camera/image_raw`.
- 2026-07-02: Created `view_all.rviz` — standalone RViz config with Grid + MotionPlanning + Camera.

### ⚠ Camera Perception Issue (BLOCKER)
- Camera publishes at ~33 Hz, `/detected_objects` arrives with empty array.
- HSV mask finds **0 orange pixels** out of 307200.
- Saved frames show uniform gray (178, 178, 178) = ground plane ambient color.
- Cube at (0.3, 0, 0.03) should be within FOV, but doesn't appear in image.
- Possible causes: software rendering glitch, cube not rendered, wrong HSV, camera orientation still wrong.

## Feature 5: End effector nodes

- 2026-07-02: `gripper_state_node.py` — subscribes `/joint_states`, publishes `/gripper/position` (Float64) and `/gripper/state` (String: open/closed/moving).
- 2026-07-02: `vacuum_controller_node.py` — `/vacuum/control` service (SetBool), publishes `/vacuum/active` (Bool). Placeholder — no physical vacuum model.
- Wire into bringup.launch.py.

## Feature 6: Pick-and-place orchestrator

- 2026-07-02: `pick_place_orchestrator.py` — async state machine using MoveIt2 `/move_action` + gripper `/gripper_controller/gripper_cmd`.
  - States: WAITING → HOME → APPROACH → GRASP → CLOSE → RETRACT → PLACE → RELEASE → DONE → WAITING
  - Falls back to direct `FollowJointTrajectory` if MoveIt2 unavailable.
- `motion_executor.py` — `move_to_joints()` + `gripper_command()` using action clients.
- `pick_place_bt.py` / `pick_place_bt_node.py` — older behavior tree approach (unused, kept for reference).

## Known Issues

1. **MoveIt2 mimic joint errors**: `robotiq_gripper.ros2_control.xacro` is unconditionally included, defining `_mimic` joints not in URDF model. Non-fatal but noisy.
2. **Camera perception**: Cube not visible in camera feed. Under investigation.
3. **RViz2 crash**: MotionPlanning + Camera in same instance segfaults (Humble bug). Workaround: separate RViz instances.
