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
- Camera perception now working reliably. Detects orange cube at (0.312, 0.008, 0.0) via HSV mask.

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

## Feature 7: Orchestrator fixes (2026-07-07)

- **GRASP_Z_OFFSET = 0.12**: Added to keep tool0 above the ground_plane collision object during GRASP and PLACE_DOWN poses.
- **IK fallback → direct joint control**: When MoveIt2 pose planning fails (error_code=-2, -4, 99999), the orchestrator computes IK via `/compute_ik` and sends the joint solution directly to `FollowJointTrajectory` controller, bypassing MoveIt2's start-state collision validation.
- **IK retry without collision avoidance**: If IK with `avoid_collisions=True` fails (error=-31 NO_IK_SOLUTION), retries with `avoid_collisions=False` — needed for PLACE_DOWN where the gripper intentionally nears the ground.
- **Perception log throttling**: Cube detection logging throttled to every 2 seconds instead of every message (~30 Hz).
- **PLACE target changed**: From (0.3, 0.3, 0.0) to (0.0, -0.3, 0.0) for better reachability.
- Verified: 2 full pick-and-place cycles completed successfully (continuous looping).

## Known Issues

1. **MoveIt2 mimic joint errors**: `robotiq_gripper.ros2_control.xacro` is unconditionally included, defining `_mimic` joints not in URDF model. Non-fatal but noisy.
2. **Perception always reports z=0.0**: Hardcoded table_z=0.0 in ray-plane intersection. Worked around via GRASP_Z_OFFSET.
3. **Camera occluded by robot arm**: When arm passes overhead, cube detection is temporarily lost. Orchestrator buffers last detection.
4. **RViz2 crash**: MotionPlanning + Camera in same instance segfaults (Humble bug). Workaround: separate RViz instances.

## Safety Enhancements (2026-07-07)

- **Joint velocity limits**: Added `has_velocity_limits: true` with `max_velocity: 0.5 rad/s` (arm) / `0.8 rad/s` (wrist) in `joint_limits.yaml` for MoveIt2 planning.
- **Safe duration computation**: `_send_joint_goal` now computes trajectory duration based on max joint displacement / `SAFE_VELOCITY` (0.4 rad/s), preventing unsafe speeds during direct joint control.
- **Joint state tracking**: Orchestrator subscribes to `/joint_states` to know current arm configuration before computing safe durations.
- **Controller constraints**: Added position constraints (goal: 0.05 rad, trajectory: 0.1 rad) to `joint_trajectory_controller` config.
- **GRASP_Z_OFFSET**: Set to 0.10 to allow gripper fingers to reach cube height (z=0.03). Ground collision avoided via IK fallback with `avoid_collisions=False` + direct joint control (MoveIt2 collision check bypassed).
- **Acceleration limits**: Reduced from 5.0 to 2.0 rad/s² (arm) / 3.0 rad/s² (wrist) for smoother motion.

## Feature 8: Working physical grasp — cube actually picked & placed (2026-08-04)

End-to-end verified: cube detected at (0.4,0), grasped, lifted, carried, and set
down at the place target (0.30,0.30). Confirmed by perception (0.312,0.312) and
overhead camera. Fixes made this session:

- **New package `ur3e_gazebo_plugins`** (ament_cmake, Gazebo Classic model plugins):
  - `libur3e_mimic_joint_plugin.so` — couples the Robotiq passive finger joints to
    the driver knuckle via `SetPosition` (kinematic follow). **SetForce coupling
    broke gazebo_ros2_control joint control** (arm/gripper froze); SetPosition works.
  - `libur3e_gripper_grasp_plugin.so` — attach-on-close grasp. The target cube is
    held KINEMATIC and plugin-managed: held at rest when open, carried at the
    finger-tip midpoint when the knuckle closes past 0.35 rad, set down on the
    ground on release. A kinematic body cannot be knocked by contact, which is the
    only reliable way to grasp in Gazebo Classic here.
  - Also builds JenniferBuehler `GazeboGraspFix` (`libgazebo_grasp_fix.so`) but it
    **segfaults gzserver under motion** — not used; the custom plugin replaces it.
- **Gripper mimic fixed**: added URDF `<mimic>` tags to the 5 passive finger joints
  and removed them from `ros2_control` (gazebo_ros2_control 0.4.10 has no mimic
  support and was freezing them). Both fingers now close symmetrically.
- **Cube dynamics**: made `target_cube` non-static with mass 0.15 kg and softer
  contact (kp 1e5, kd 1, max_vel 0.01); mass 0.03 + kp 1e6 caused spawn explosions.
  `collide_bitmask` 0x01 on the cube and 0xfffe on the finger links so the fingers
  never physically collide with it.
- **Orchestrator grasp fixes**: `GRIPPER_CLOSED` 0.45→0.70 (grips the 6 cm cube),
  `APPROACH_HEIGHT` 0.22→0.30 (finger tips clear the cube before the vertical
  descent — 0.22 clipped/knocked it), `GRASP_Z_OFFSET`→0.18, tighter goal pose
  tolerance (position sphere 0.05→0.015 m, orientation 0.2→0.08 rad), ERROR-state
  recovery (reopen gripper + re-home + retry instead of dead-ending), throttled
  reach-limit warning, `PLACE` moved to (0.30,0.30) (reachable; (0,-0.3) was behind
  the arm and caused wild IK swings).
- **Controller tolerances**: `joint_trajectory_controller` `trajectory` tolerance
  0.1→0.0 (disabled) — the 0.1 rad mid-trajectory check was aborting the direct
  IK-fallback moves.
- **Launch fix**: `robot_description` wrapped in `ParameterValue(..., value_type=str)`;
  and **XML comments in the URDF must not contain ": " (colon-space)** — it makes
  rclpy/rcl try to YAML-parse the URDF and fail, so gazebo_ros2_control never
  starts the controller_manager.

### Launch (3 terminals, headless gzserver ok)
    ros2 launch ur3e_sim_bringup bringup.launch.py            # + GAZEBO_PLUGIN_PATH to install/ur3e_gazebo_plugins/lib
    ros2 launch ur3e_sim_bringup move_group.launch.py launch_rviz:=false
    ros2 launch ur3e_motion motion.launch.py

## Feature 9: Collision safety, gripper closure, straight-line motion (2026-08-06)

Three observed defects fixed: the arm clipped the cube on the way in, the fingers
closed straight through it, and the arm made unsafe self-colliding / wrapped-around
moves. Verified over 4 continuous cycles: 0 self-collisions, 0 invalid plans,
0 IK fallbacks, 17 straight-line segments solved at 100%.

- **Gripper closed too far.** `GRIPPER_CLOSED` was a hard-coded 0.70 rad, which on
  the 2F-85 stroke (0 rad = 85 mm gap, 0.8 rad = shut) asks for a **10.6 mm** pad
  gap around a **60 mm** cube — 24.7 mm of penetration per finger. It is now
  derived from the object width via `gripper_angle_for_width()`, giving **0.273 rad**
  (~2 mm squeeze per pad). The grasp plugin's `attach_threshold` had to drop
  0.35→0.20 (and `detach_threshold` 0.15→0.08) or it would never latch.
- **Gripper action hung the state machine.** With the knuckle now stopping at a
  partly-closed angle it jitters ±0.02 rad under the finger linkage load, which is
  wider than `GripperActionController`'s default 0.01 rad `goal_tolerance` — the
  action reported neither success nor stall and the cycle sat in `GRASP_CLOSE` for
  320 s. Fixed with `goal_tolerance: 0.05`, `allow_stalling: true`, `max_effort: 50.0`,
  plus a `GRIPPER_TIMEOUT` watchdog in the orchestrator that judges the move by the
  actual joint position so a wedged gripper can never stall the cycle again.
- **Cube was invisible to MoveIt.** `scene_initializer_node.py` only ever added the
  ground plane, so every plan went straight through the cube. The orchestrator now
  publishes it as a `CollisionObject` on detection, attaches it to `tool0` (with
  `touch_links`) while carried, and removes it on release.
- **Cube must leave the scene before the gripper opens.** Putting it back into the
  world at release left the fingers wrapped around it, so the next plan started in
  collision (`target_cube` vs `robotiq_85_right_finger_tip_link`) and was discarded
  as invalid. Release now removes it entirely, departs straight up, then re-adds it.
- **Descents/lifts are Cartesian.** APPROACH→GRASP, GRASP→RETRACT, PLACE→PLACE_DOWN
  and the depart are `/compute_cartesian_path` straight lines (`avoid_collisions=False`
  by design — they deliberately close on the object). A free joint-space plan between
  the same poses arcs sideways, which is what knocked the cube over.
- **SRDF blinded MoveIt to self-collision.** 66 pairs were wrongly disabled: every
  gripper link vs `forearm_link` was tagged `Adjacent` (it is 4 joints away) and vs
  `shoulder_link`/`upper_arm_link`/`base*` tagged `Never`. Re-enabled — this is what
  let the arm drive into itself.
- **No joint position limits.** `joint_limits.yaml` set velocity/acceleration only, so
  all six joints kept the URDF's ±2π range and the planner picked wrapped-around and
  elbow-flipped branches. Now ±π (shoulder_lift capped at 0, so the upper arm never
  swings below horizontal).
- **IK fallback bypassed all safety.** It retried with `avoid_collisions=False` and
  streamed the raw solution to the controller as a *single* trajectory point — no
  path checking whatsoever. Now: collision avoidance always on, seeded from the
  current joint state (KDL is random-restart and was returning arbitrary branches),
  rejected if it reconfigures the arm by more than `MAX_IK_JOINT_JUMP` (2.0 rad), and
  routed through MoveIt joint-space planning so the path is checked. Observed
  rejecting a real 2.17 rad swing and recovering cleanly.
- **Controller runtime guard restored.** `trajectory` tolerance 0.0 (disabled) → 0.5 rad,
  safe now that no single-point trajectories are sent.
- **KDL IK budget** raised (timeout 0.005→0.05 s, attempts 3→5).

## Feature 10: The cube is a real physics object (2026-08-06)

The cube used to be a puppet. `libur3e_gripper_grasp_plugin.so` called
`SetKinematic(true)` on it and then `SetWorldPose()` **every world tick** — held at
a stored rest pose when open, snapped to the finger-tip midpoint when closed. So
it ignored gravity and every contact force, could not be dragged in the Gazebo GUI
(the plugin teleported it back within one 4 ms step), and "glitched to the floor"
on release because it was assigned `rest_z`, a **hardcoded 0.03** the xacro never
overrode. Its `<mass>` and inertia were dead config. The fingers could not even
touch it: all 8 finger links had `collide_bitmask 0x00` and the cube `0x01`.

It is now genuinely dynamic — gravity, real contact, pushable, GUI-draggable, and
it falls if the grip fails.

- **Grasp plugin off by default** (`use_grasp_fix` now defaults to `false`, in both
  the xacro and `bringup.launch.py`). Pass `use_grasp_fix:=true` to restore the old
  deterministic-but-fake hold.
- **Finger pads get real contact**: the two `*_finger_tip_link` pads no longer use
  `collide_bitmask 0x00`; they get friction and contact parameters via a new
  `finger_pad_contact` macro. The 6 intermediate linkage links stay non-colliding
  (the open-tree 4-bar still ejects itself otherwise).
- **Pads are force controlled.** `gazebo_ros2_control`'s position interface is a
  kinematic `SetPosition` teleport — it reaches the commanded angle regardless of
  what is in the way, so a gripper driven by it cannot stall on an object and just
  squeezes it out. The two pads hang off the *mimic* joints, so those four
  (`*_inner_knuckle`, `*_finger_tip`) now run `usePID` force coupling and can stall.
  `maxEffort` 2.0 N·m — at 20 N·m the pads launched the cube 3.5 m across the world.
- **Physics timestep 4 ms → 1 ms**, 150 solver iterations, `contact_surface_layer`
  0.001 → 0.0002. A 4 ms step cannot resolve a friction grasp.
- **Contact dead-band removed.** `min_depth` was 0.001 and the surface layer 0.001,
  so the first ~1 mm of penetration produced *no* force at all — the pads touched
  the cube geometrically and pushed with nothing.
- **Gripper stroke is now measured, not modelled.** `GRIPPER_CALIBRATION` in the
  orchestrator is a real pad-gap-vs-angle table taken off the running model. The
  linear `0.085*(1 - angle/0.8)` approximation is ~3 mm out mid-range, which is the
  difference between gripping a 60 mm cube and closing on air.
- **Perception centring fix.** The overhead camera sees the cube's *top* face, and
  intersecting that sightline with the table plane put the estimate ~11 mm off, far
  enough for one pad to catch the cube and shove it. It now intersects at the
  object's top (`object_height` param), which is the correct x/y of the centre; the
  reported z still sits on the table so the grasp offsets are unchanged.
- **Cube is lighter and grippier**: 0.15 kg → 0.08 kg (inertia updated to 4.8e-5),
  friction 1.0 → 3.0, contact `kp` 1e5 → 1e6 (at 1 ms, `kp` 1e5 gives ODE CFM ~0.01,
  soft enough for the cube to creep out).
- **Carry is gentler**: `CARRY_VEL_SCALE` 0.15 / `CARRY_ACC_SCALE` 0.10, and arm
  velocity limits 0.5 → 0.35 rad/s (Cartesian segments have no scaling field in
  Humble, so they are paced by `joint_limits.yaml`).
- **`/gazebo/model_states` added** (`libgazebo_ros_state.so`) so the cube's true pose
  can be measured without polling `gz model`, which crashes gzserver.

### ⚠ Status: the friction grasp is real but NOT yet reliable

Cycles run without error and the cube is genuinely picked up and carried, but it
creeps out of the pads before reaching the place target, and the result varies
between runs. Measured peak lift above the 0.030 m rest height:

| configuration | peak cube z | outcome |
|---|---|---|
| rigid pads, 3 mm squeeze | 0.057 | slips almost immediately |
| force pads, 20 N·m | 1.20 | cube launched to (-3.4, 3.6) |
| force pads 0.8 N·m, kp 1e5 | 0.071 | slips |
| force pads 2.0 N·m, kp 1e6 | **0.198** | lifted and carried ~20 cm, dropped mid-transfer |
| same, repeat run | 0.050 | marginal — run-to-run variance is large |

Force is not the limit: holding 0.08 kg at μ=3.0 needs ~0.01 N·m and the pads
supply ~0.6 N·m. The likely remaining cause is contact **quality** — the pads use a
mesh collision (`left/right_finger_tip.stl`) and ODE gives mesh-vs-box only a
couple of unstable contact points. Replacing them with a flat box pad was tried
(0.005 x 0.022 x 0.045 at x=±0.0228) and made it worse — the pads missed the cube
entirely — so it was reverted; the offset needs to be derived properly rather than
inferred from the STL bounding box.

Next things to try: correctly-placed box or cylinder pad collisions, a wider pad,
or a contact sensor to confirm where the pads actually touch.

### Residual, non-blocking
- A straight-line grasp descent occasionally solves only ~40% when the cube sits near
  the reach limit (radius ~0.44 m); it falls back to a planned motion and succeeds.
- Rarely the depart trajectory is rejected by `allowed_start_tolerance` (0.1 rad) when
  the arm settles after the gripper opens; the cycle still completes.

### Known limitations
- Grasp hold is plugin-assisted (kinematic carry on contact), not raw friction —
  raw friction grasping of a 6 cm cube with the 2F-85 is unstable in Gazebo Classic.
- MoveIt pose planning fails (START_STATE_COLLISION) near the ground and falls back
  to `/compute_ik` + direct joint control every pick; functional but the transfer
  arc can swing. Perception still reports z=0 (ground-plane projection).
