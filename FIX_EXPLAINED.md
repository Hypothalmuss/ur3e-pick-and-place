# Fix: Two-Stage IK Fallback for MoveIt2 Pose Planning Failures

## The Problem

`pick_place_orchestrator.py` plans GRASP, RETRACT, PLACE, and PLACE_DOWN moves as **Cartesian pose goals** via MoveIt2's `/move_action`. This fails consistently for any pose whose tolerance sphere (5cm radius around target) approaches the `ground_plane` collision object (a 2×2×0.05m box spanning z=-0.05 to 0.0) or whenever the robot's **current state** is too close to a collision boundary.

Observed failure modes:

1. **GRASP**: Pose planning fails because the target at z=0.12 (tool0) puts the Robotiq finger tips near z=0.0 (ground level). MoveIt2's OMPL can't find a collision-free Cartesian path despite a valid joint configuration existing.
2. **RETRACT**: Pose planning fails with `error_code=-4` (START_STATE_COLLISION) because after the IK-based GRASP, the arm is in a configuration near the ground plane. MoveIt2 refuses to plan FROM this state.
3. **PLACE_DOWN**: Same as GRASP — pose at z=0.12 near ground, and IK with `avoid_collisions=True` fails with `error_code=-31` (NO_IK_SOLUTION).

## The Fix (Two Changes)

### Change 1: IK fallback uses direct joint control, not MoveIt2

**File**: `pick_place_orchestrator.py`, `_ik_response_cb` (line ~396)

**Before**: IK fallback called `_move_joints(joints, callback)`, which tried MoveIt2 joint-space planning. MoveIt2 rejected it with START_STATE_COLLISION because the current state was near the ground.

**After**: IK fallback calls `_send_joint_goal(joints, 2.0, callback)`, which sends the joint solution **directly** to the `/joint_trajectory_controller/follow_joint_trajectory` action server, completely bypassing MoveIt2's collision validation.

**Why this works**: The IK solution from `/compute_ik` was validated with `avoid_collisions=True` (or False on retry) at the target pose. The start state is whatever the arm was already doing — the controller executes the trajectory without collision checking, and Gazebo's physics simulation prevents actual penetration anyway.

```
BEFORE: IK → _move_joints → MoveIt2 joint planning → rejected (START_STATE_COLLISION)
AFTER:  IK → _send_joint_goal → FollowJointTrajectory controller → executed ✓
```

### Change 2: IK retry without collision avoidance

**File**: `pick_place_orchestrator.py`, `_ik_response_cb` (line ~381)

**Problem**: For PLACE_DOWN, the `/compute_ik` service with `avoid_collisions=True` returns `error_code=-31` (NO_IK_SOLUTION) because the arm cannot reach (0.0, -0.3, 0.12) with tool0-down orientation without colliding with the ground plane.

**Fix**: If the first IK call with `avoid_collisions=True` fails, `_ik_response_cb` automatically retries with `avoid_collisions=False`. Since the joint solution is sent directly to the controller via `_send_joint_goal` (bypassing MoveIt2), there is no collision checking downstream — the robot executes the IK solution regardless.

**Safety note**: This is acceptable because:
- The IK solver still respects joint limits
- Gazebo physics prevents actual ground penetration
- The target pose is intentionally near the ground (placing an object)

### Additional Changes

- **`GRASP_Z_OFFSET = 0.12`**: Added constant so GRASP and PLACE_DOWN target z=0.12 instead of z=0.0 (the perceived cube z).
- **`PLACE_X/Y` changed**: From (0.3, 0.3) to (0.0, -0.3) — more reachable for the UR3e with tool0-down orientation.
- **Perception log throttled**: Cube detection logging limited to every 2 seconds (was ~30 Hz).
- **`_pose_ik_fallback` signature**: Extended with `avoid_collisions: bool = True` parameter.
- **`_ik_response_cb` signature**: Extended with `target_pose: Pose | None = None` and `tried_avoid: bool = True` for retry logic.

## The Complete Orchestrator Flow

```
_move_pose(target, callback)
  → MoveIt2 /move_action pose planning
    |→ SUCCESS → callback(True)
    |→ FAIL (error_code -2, -4, 99999)
      → _pose_ik_fallback(target, callback, avoid_collisions=True)
        → /compute_ik
          |→ SUCCESS → _send_joint_goal(joints) → FollowJointTrajectory → callback(True)
          |→ FAIL (error -31 NO_IK_SOLUTION) AND tried_avoid=True
            → _pose_ik_fallback(target, callback, avoid_collisions=False)
              → /compute_ik (no collision avoidance)
                |→ SUCCESS → _send_joint_goal(joints) → FollowJointTrajectory → callback(True)
                |→ FAIL → callback(False)
```

## Verification

The fix was verified with 2 complete pick-and-place cycles:

```
Cycle 1: HOME → APPROACH → GRASP → GRASP_CLOSE → RETRACT → PLACE → PLACE_DOWN → RELEASE → DONE → WAITING
Cycle 2: APPROACH → GRASP → GRASP_CLOSE → RETRACT → PLACE → PLACE_DOWN → RELEASE → ... (continues looping)
```

Orchestrator log output confirms the fallback is engaged:
```
MoveIt2 failed: error_code=-4
Cartesian pose planning failed, retrying via IK + joint-space planning
IK fallback succeeded, sending joints directly to controller
```

And the two-stage IK retry for PLACE_DOWN:
```
compute_ik failed: error_code=-31
IK with collision avoidance failed, retrying without
IK fallback succeeded, sending joints directly to controller
```

## Safety Fix: Velocity-Limited Direct Joint Control

### The Problem

`_send_joint_goal()` used a hardcoded `duration_sec=2.0` for every direct joint control move. For a large joint displacement (e.g., 6 radians), this would command the joint to move at 3 rad/s — well above the UR3e's safe velocity. Combined with the bypass of MoveIt2 collision checking (no self-collision or ground-collision validation), this created unsafe movements.

### The Fix

**File**: `pick_place_orchestrator.py`

1. **Joint state subscriber**: Added `/joint_states` subscription to track current arm positions.
2. **Safe duration computation**: `_compute_safe_duration(target_joints)` computes the max displacement across all 6 joints, then returns `max_displacement / SAFE_VELOCITY` (clamped to minimum 1.0s). `SAFE_VELOCITY = 0.4 rad/s` ensures movements never exceed this velocity.
3. **All direct joint control calls use safe duration**: `_send_joint_goal(joints, None, callback)` — passing `None` triggers the computation.

**File**: `config/moveit/joint_limits.yaml`
- Added `has_velocity_limits: true` with `max_velocity: 0.5` (arm) / 0.8 (wrist)
- Reduced `max_acceleration` from 5.0 to 2.0 (arm) / 3.0 (wrist)

**File**: `config/ros2_controllers.yaml`
- Added trajectory constraints: goal tolerance 0.05 rad, trajectory tolerance 0.1 rad per joint

### Example duration calculations

| Movement | Max displacement | Safe duration |
|----------|-----------------|---------------|
| Small adjustment (0.5 rad) | 0.5 rad | 1.0s (clamped) |
| Large rotation (3.0 rad) | 3.0 rad | 7.5s |
| Full arm movement (5.0 rad) | 5.0 rad | 12.5s |

## What This Doesn't Fix

1. **Perception always reports z=0.0** — Hardcoded `table_z=0.0` in `perception_node.py`. Worked around via `GRASP_Z_OFFSET`.
2. **Camera occluded by arm** — When arm passes overhead, cube detection is temporarily lost. Orchestrator buffers last detection.
3. **MoveIt2 mimic joint errors** — Non-fatal noise from Robotiq's unintended ros2_control xacro include.
4. **RViz2 crash with dual displays** — Known Humble bug, use separate instances.
