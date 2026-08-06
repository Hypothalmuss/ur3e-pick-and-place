# NEXT PROMPT — UR3e Pick-and-Place (Working Pipeline)

## Context

This is a ROS 2 Humble + Gazebo simulation of a UR3e robot with Robotiq 2F-85 gripper, overhead camera, and pick-and-place pipeline. The workspace is at `/home/eagletn3/Downloads/new_ur3e`. All Python nodes use `--symlink-install` (live edits, no rebuild needed).

**Status**: ✅ End-to-end pick-and-place pipeline is working. 2 full cycles verified.

## What's Working

1. **Simulation bringup** — Gazebo loads: UR3e, gripper, target orange cube at (0.3, 0, 0.03), overhead camera at (0.15, 0, 3.0) looking down.
2. **Perception** — Detects orange cube via HSV mask at actual position (~0.312, 0.008, 0.0).
3. **MoveIt2 joint planning** — HOME/RETRACT joint moves succeed via `move_group`.
4. **MoveIt2 pose planning** — APPROACH and PLACE (high z) succeed.
5. **IK fallback** — When pose planning fails (GRASP, RETRACT, PLACE_DOWN), `/compute_ik` provides joint solution, sent directly to `FollowJointTrajectory` controller — bypassing MoveIt2 start-state collision check.
6. **IK retry without avoidance** — If `avoid_collisions=True` IK fails, retries with `avoid_collisions=False` (needed for PLACE_DOWN near ground).
7. **Gripper control** — Opens/closes reliably via `/gripper_controller/gripper_cmd`.
8. **Continuous looping** — After DONE, returns to WAITING and starts next cycle automatically.

## Known Issues (No Blockers)

### Issue 1: Perception always reports z=0.0

**Location**: `src/ur3e_perception/ur3e_perception/perception_node.py` line 143

Ray-plane intersection uses `self._table_z = 0.0`. All detected objects get z=0.0 regardless of actual height. Worked around via `GRASP_Z_OFFSET=0.12` in orchestrator.

**Possible fix**: Report actual intersection height or include object dimensions.

### Issue 2: Perception loses cube when arm moves overhead

Overhead camera at (0.15, 0, 3.0) can't see cube when arm passes between. Orchestrator buffers last detection so this is accepted.

**Possible fix**: Move camera to offset position (e.g., x=-0.3, y=0.6) for unobstructed view.

### Issue 3: MoveIt2 mimic joint errors (noisy)

Robotiq's `robotiq_gripper.ros2_control.xacro` is unconditionally included, defining `_mimic` joints not in URDF model. Non-fatal.

### Issue 4: RViz2 crash with dual displays

MotionPlanning + Camera in same RViz2 instance segfaults (Humble bug). Workaround: separate RViz2 instances.

## Key Architecture Decisions

### IK fallback flow
```
_move_pose(target) → MoveIt2 pose plan
  fails (error_code -2/-4/99999) →
    _pose_ik_fallback(target, avoid_collisions=True)
      /compute_ik → joint solution
        fails (error -31 NO_IK_SOLUTION) →
          _pose_ik_fallback(target, avoid_collisions=False)
            /compute_ik → joint solution →
              _send_joint_goal(joints)
                → /joint_trajectory_controller/follow_joint_trajectory
```

### Why direct joint control instead of MoveIt2 for IK fallback
The IK solution places the arm correctly at the target pose, but when trying to plan FROM the current arm state (which is very close to the ground/collision boundary), MoveIt2's `START_STATE_COLLISION` check rejects it immediately. Sending joints directly to the controller bypasses this validation entirely.

## Launch Order

```bash
# Terminal 1 — Gazebo + robot + controllers + perception
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
source /usr/share/gazebo/setup.sh
export PATH="/usr/bin:$PATH"
source /home/eagletn3/Downloads/new_ur3e/install/setup.bash
ros2 launch ur3e_sim_bringup bringup.launch.py

# Terminal 2 — MoveIt2 (no RViz to avoid crash)
source /opt/ros/humble/setup.bash
source /home/eagletn3/Downloads/new_ur3e/install/setup.bash
ros2 launch ur3e_sim_bringup move_group.launch.py launch_rviz:=false

# Terminal 3 — Orchestrator + motion executor
source /opt/ros/humble/setup.bash
source /home/eagletn3/Downloads/new_ur3e/install/setup.bash
ros2 launch ur3e_motion motion.launch.py
```

## Key Files

| File | Purpose |
|------|---------|
| `src/ur3e_motion/ur3e_motion/pick_place_orchestrator.py` | State machine orchestrator with IK fallback + direct joint control |
| `src/ur3e_perception/ur3e_perception/perception_node.py` | HSV-based orange cube detection + TF-based ground-plane projection |
| `src/ur3e_end_effectors/ur3e_end_effectors/nodes/gripper_state_node.py` | Gripper position/state publisher |
| `src/ur3e_motion/ur3e_motion/motion_executor.py` | Low-level joint/gripper action clients (unused — orchestrator uses internal action clients) |
| `src/ur3e_sim_bringup/launch/bringup.launch.py` | Main bringup launch (Gazebo, controllers, perception) |
| `src/ur3e_sim_bringup/launch/move_group.launch.py` | MoveIt2 move_group + RViz |
| `src/ur3e_motion/launch/motion.launch.py` | Orchestrator + motion executor |
| `src/ur3e_sim_bringup/worlds/ur3e_workcell.world` | Gazebo world |
| `src/ur3e_sim_bringup/config/moveit/` | MoveIt2 SRDF, kinematics, OMPL, controllers |
| `src/ur3e_sim_bringup/urdf/robotiq_2f_85_macro.urdf.xacro` | Custom Robotiq macro with `include_ros2_control="false"` |
| `FIX_EXPLAINED.md` | Detailed explanation of the IK fallback fix |

## Key Constants (pick_place_orchestrator.py)

| Constant | Value | Purpose |
|----------|-------|---------|
| `GRIPPER_OPEN` | 0.0 | Gripper fully open position |
| `GRIPPER_CLOSED` | 0.7929 | Gripper fully closed position |
| `APPROACH_HEIGHT` | 0.22 | Z offset above cube for approach pose |
| `LIFT_HEIGHT` | 0.30 | Z offset for retract/lift after grasp |
| `GRASP_Z_OFFSET` | 0.10 | Z offset for grasp and place_down poses |
| `PLACE_X` | 0.0 | Place target X coordinate |
| `PLACE_Y` | -0.3 | Place target Y coordinate |
| `PLACE_Z` | 0.0 | Place target Z (base height) |
| `REACH_MAX` | 0.55 | Conservative UR3e reach limit (m) |
| `SAFE_VELOCITY` | 0.4 rad/s | Max joint velocity for direct joint control duration calc |

## Safety Architecture

### Joint velocity limiting (two layers)

1. **MoveIt2 planning layer**: `joint_limits.yaml` limits (0.5 rad/s arm, 0.8 rad/s wrist) enforced by MoveIt2 during trajectory planning.
2. **Direct control layer**: `_compute_safe_duration()` computes trajectory time as `max_displacement / SAFE_VELOCITY` (min 1.0s), used by `_send_joint_goal()` when sending joints directly to `FollowJointTrajectory`.

### Config files

| File | Changes |
|------|---------|
| `config/moveit/joint_limits.yaml` | Added `has_velocity_limits` (arm: 0.5, wrist: 0.8), reduced `max_acceleration` (arm: 2.0, wrist: 3.0) |
| `config/ros2_controllers.yaml` | Added `constraints` with goal/trajectory tolerances (0.05/0.1 rad) |

## Verifying the Pipeline

```bash
# Watch state transitions
tmux capture-pane -t ur3e_test:2 -p -S -500 | grep -E "State ->|Pick-and-place|complete|Ready|ERROR|IK fallback"

# Check gripper
ros2 topic echo /gripper/state

# Check cube detection
ros2 topic echo /detected_objects --once

# Direct gripper test
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 10.0}}"
```

## Git Log

```
833a9fe chore: update PROGRESS.md and MEMORY.md
328ba8c feat: end effector nodes, pick-and-place orchestrator, camera orientation fix
c35d822 feat: perception node with orange detection, table URDF spawn fix, motion restructure
7a4b51a chore: add frames and MEMORY.md to gitignore
d861361 feat: sim bringup with scene initializer, ground collision, gripper fixes
57dc556 feat: robot bringup with UR3e, gripper, and overhead camera
f001618 feat: workspace scaffold and ur3e_msgs interfaces
```
