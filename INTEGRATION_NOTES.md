# INTEGRATION NOTES — Build & Launch Reference

## Prerequisites

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1   # required for Gazebo on this machine
source /usr/share/gazebo/setup.sh  # avoid RTShader crash
```

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Launch (Feature 2)

```bash
# Terminal 1: Gazebo + robot bringup
export LIBGL_ALWAYS_SOFTWARE=1
source /usr/share/gazebo/setup.sh
export PATH="/usr/bin:$PATH"  # if Conda active
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3e_sim_bringup bringup.launch.py
```

## Packages

| Package | Type | Entry Points |
|---|---|---|
| ur3e_msgs | ament_cmake | — |
| ur3e_end_effectors | ament_python | `gripper_state_node`, `vacuum_controller_node` |
| ur3e_sim_bringup | ament_python | `scene_initializer` |
| ur3e_perception | ament_python | `perception_node` |
| ur3e_motion | ament_python | `pick_place_bt_node`, `motion_executor` |
| ur3e_dashboard | ament_python | `dashboard_server` |

## Environment Variables

See `MASTER_PROMPT.md` Section 10 for the full reference.
