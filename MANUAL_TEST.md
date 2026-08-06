# MANUAL TEST — UR3e Pick-and-Place Workspace

## Prerequisite (run in ALL terminals)

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
source /usr/share/gazebo/setup.sh
export PATH="/usr/bin:$PATH"
source /home/eagletn3/Downloads/new_ur3e/install/setup.bash
```

## Build

```bash
cd /home/eagletn3/Downloads/new_ur3e
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Rebuild specific packages:
```bash
colcon build --packages-select ur3e_msgs
colcon build --packages-select ur3e_perception
colcon build --packages-select ur3e_motion
```

## Launch Order

**ORDER MATTERS** — wait for each terminal to fully load before starting the next.

### Terminal 1 — Gazebo + robot + controllers + perception

```bash
ros2 launch ur3e_sim_bringup bringup.launch.py
```

Launches: `robot_state_publisher`, `gzserver` + `gzclient`, robot spawn, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controller`, camera TF, `perception_node`, `gripper_state_node`.

Wait ~10–15 seconds for Gazebo to load and robot to spawn.

### Terminal 2 — MoveIt2 + RViz

```bash
ros2 launch ur3e_sim_bringup move_group.launch.py
```

Launches: `move_group` node, `scene_initializer_node`, RViz2 with MotionPlanning display.

Headless mode (no RViz):
```bash
ros2 launch ur3e_sim_bringup move_group.launch.py launch_rviz:=false
```

### Terminal 3 — Pick-and-place orchestrator

```bash
ros2 launch ur3e_motion motion.launch.py
```

Launches: `motion_executor`, `pick_place_orchestrator`, `pick_place_bt_node`.

Orchestrator auto-starts when it receives a `/detected_objects` message with ≥1 object.

### Terminal 4+ — Diagnostics (any order)

```bash
ros2 topic echo /detected_objects              # perception output
ros2 topic echo /gripper/state                 # gripper state
ros2 topic echo /gripper/position              # gripper position
ros2 topic echo /vacuum/active                 # vacuum state
ros2 action list -t                            # active actions
ros2 action info /move_action                  # MoveIt2 action
ros2 action info /gripper_controller/gripper_cmd # gripper action
ros2 control list_controllers                  # controller status
ros2 run image_view image_view image:=/camera/image_raw  # camera feed
ros2 run tf2_tools view_frames.py              # TF tree dump
```

### Terminal 4+ — Separate RViz2 instances (avoid segfault)

```bash
# Robot visualization (already launched in Terminal 2, skip if running)
rviz2 -d /home/eagletn3/Downloads/new_ur3e/src/ur3e_sim_bringup/config/moveit/view_all.rviz

# Camera feed only (separate instance to avoid MotionPlanning+Camera crash)
rviz2
# Add: Grid, RobotModel, Camera (topic /camera/image_raw)
```

## Standalone Nodes (bypass launch files)

```bash
ros2 run ur3e_perception perception_node
ros2 run ur3e_end_effectors gripper_state_node
ros2 run ur3e_end_effectors vacuum_controller_node
ros2 run ur3e_motion motion_executor
ros2 run ur3e_motion pick_place_orchestrator
ros2 run ur3e_motion pick_place_bt_node
ros2 run ur3e_dashboard dashboard_server
```

## Service Calls

```bash
# Vacuum on/off
ros2 service call /vacuum/control std_srvs/srv/SetBool "{data: true}"
ros2 service call /vacuum/control std_srvs/srv/SetBool "{data: false}"
```

## Manually Trigger Orchestrator (bypass perception)

```bash
ros2 topic pub /detected_objects ur3e_msgs/msg/DetectedObjectArray '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"},
  objects: [{
    aruco_id: 0, class_name: "cube", confidence: 1.0,
    pose: {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  }]
}' --rate 0.5
```

## Cleanup

```bash
pkill -f "ros2|gzserver|gzclient|rviz2"
# or
ros2 daemon stop
pkill -f gzserver
pkill -f gzclient
```
