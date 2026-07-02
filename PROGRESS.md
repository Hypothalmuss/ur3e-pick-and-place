# PROGRESS — Bug Log & Integration History

## Feature 1: Workspace scaffold & ur3e_msgs

- 2024-06-25: Scaffold created. 6 packages (1 ament_cmake + 5 ament_python) with all message/service interfaces defined. Builds cleanly.

## Feature 2: Robot bringup (Gazebo, URDF, controllers, TF)

- 2024-06-25: Created custom xacro `ur3e_with_effector.urdf.xacro` wrapping `ur_description` (ur3e) + `robotiq_description` (2F-85 gripper).
  - Uses `gazebo_ros2_control/GazeboSystem` for both arm (6 joints) and gripper (1 joint + 5 mimic).
  - Overhead camera defined in world file with `libgazebo_ros_camera.so` plugin.
- 2024-06-25: Created `ur3e_workcell.world` with ground plane, table, pick/place zones, and overhead camera.
- 2024-06-25: Created `ros2_controllers.yaml` with `joint_state_broadcaster`, `joint_trajectory_controller`, and `gripper_controller` (GripperActionController).
- 2024-06-25: Created `bringup.launch.py` — robot_state_publisher + gzserver/gzclient + spawn + controller spawners. Builds cleanly.

## Feature 3: MoveIt2 config (SRDF, planners, move_group)

- 2026-06-26: Created `config/moveit/` with SRDF, kinematics, OMPL planners, joint limits, controllers, RViz config.
- 2026-06-26: Created `launch/move_group.launch.py` — move_group node + optional RViz2.
- Builds cleanly. Verified SRDF parses with xacro.

## Feature 4: Overhead camera + perception

- 2026-07-02: Replaced old camera with regular camera (type="camera", B8G8R8) at (0.15, 0, 3.0) looking down (roll=π).
- 2026-07-02: Added `static_transform_publisher` for `camera_link` TF in bringup launch.
- 2026-07-02: Restored `perception_node.py` with orange cube detection via `/camera/image_raw`.
- 2026-07-02: Created `view_all.rviz` — standalone RViz config with Grid + MotionPlanning + Camera.

### Known Limitation
- **RViz2 crash**: MotionPlanning + Camera displays in the same RViz2 instance causes segfault in ROS 2 Humble.
  - **Workaround**: Run two separate RViz2 instances. See MEMORY.md for details.
