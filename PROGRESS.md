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

## 2026-08-11: Working pick-and-place, demo recorded

The friction grasp was abandoned in favour of the attach-on-close plugin
(`use_grasp_fix`, now the default in `bringup.launch.py`). Cube is picked at
(0.405, 0.008) and placed at (0.308, 0.306) — see `demo/pick_and_place.mp4`.

### The cube was being batted away, not gripped
Measured with `/gazebo/model_states` sampled at 4 Hz against `/joint_states`:

| squeeze | peak cube z | eject speed | final x |
|---|---|---|---|
| 10 mm | 0.090 | 0.91 m/s | 0.565 |
| 4 mm  | 0.091 | 0.99 m/s | 0.581 |
| 7 mm  | 0.095 | 1.13 m/s | 0.560 |

Identical across a 2.5x range, because squeeze is not the controlling variable.
The knuckle goes from -0.008 to 0.327 rad in under 0.3 s — `gazebo_ros2_control`
drives it by `SetPosition`, so it cannot stall on contact and the pads sweep
through the cube. It was displaced 57 mm before the fingers finished closing.

Ramping the close (10 increments over ~1 s, fire-and-forget goals) is much worse,
not better: each goal preempts the last, the mimic PID chases a jittering target
and goes unstable, and the cube left at 4.7 m/s and travelled 2.5 m. Reverted. Any
retry must rate-limit inside the controller or the mimic plugin.

### Contact stiffness had to drop on both surfaces
Cube `kp` 1e6 -> 1e5 *and* finger-pad `kp` 1e6 -> 1e5. ODE combines the two
surfaces, so softening only the cube leaves the contact hard (series combination
is dominated by the softer one: 9.1e4 vs 5e4). With cube mass 0.08 -> 0.15 kg this
took the throw from ~260 mm (which parked the cube under the robot, out of reach,
and deadlocked the state machine at 258 failed cycles) down to ~50 mm.
0.5 kg measured calmer still (0.70 m/s peak, 12 mm hop) but 0.15 kg was chosen.

### Grasp height was 41 mm off
`GRASP_Z_OFFSET` assumed the gripper reaches 0.16 m below tool0. Measured from TF
it is 0.119 m, so at 0.18 the finger tips bottomed out at z=0.061 — flat on the lid
of the 0.06 m cube. Now 0.15, putting the pads at ~0.031, level with mid-height.

### Planning scene blocked its own descents
At mid-cube height the gripper links genuinely overlap the cube's collision box, so
every collision-checked fallback was rejected (`compute_ik` -> NO_IK_SOLUTION) and
the descent never ran. The cube is now dropped from the scene before the grasp
descent and re-attached (carrying its own geometry) once closed; the same is done
before the place-down descent, where the attached box otherwise rests on the ground
plane. All four Cartesian segments now solve 100% with no fallbacks.

### Residual
- PLACE_DOWN still fails intermittently (~1 cycle in 2 in one run): straight-line
  path solves ~39%, falls back, and `compute_ik` returns -31. Cause not identified.
- `planning scene remove cube rejected` during error recovery — `_scene_remove_cube`
  is called on an already-removed object. Harmless but the recovery path is untidy.
- Grasp is kinematic (plugin-held): during carry the cube ignores gravity and cannot
  be dragged in the GUI.

## 2026-08-11 (later): dashboard control, and three real bugs it exposed

`ur3e_dashboard` is now a working web UI (`http://127.0.0.1:8080`, launched by
`run_sim.sh`) driving the cell over `ur3e_msgs/srv/RunTask`. Tasks: `home`,
`retract`, `pick_place`, `pick_to` (x,y from the UI), `open_gripper`,
`close_gripper`, `stop`. The orchestrator no longer loops on its own - it idles
until commanded; `auto:=true` restores the old behaviour.

Plain stdlib `http.server` + a ROS node, no FastAPI/rosbridge, so it runs on a
bare Humble install. `GET /api/state` (status + camera/orchestrator liveness),
`POST /api/task`. UI polls at 400 ms.

Acceptance: 10/10 twice in a row (20/20), across 6 pick cycles - 0 ERROR states,
0 NO_IK_SOLUTION, 0 stale callbacks. Place accuracy 7-10 mm.

### Stop did not stop
An aborted cycle's callback chain kept running: after a stop the robot executed
a whole phantom pick-and-place in the background, interleaved with the next
command (observed: ERROR -> recovering -> GRASP -> HOME -> GRASP_CLOSE ->
RETRACT -> PLACE -> ...). Fixed with a generation counter - `_guard` binds every
motion callback to the generation current when the motion was requested, and
`_abandon()` bumps it on task start, stop, and entry to recovery. Stale
callbacks are now dropped with a log line instead of driving the arm.

### PLACE_DOWN raced its own planning scene
`_apply_scene` was fire-and-forget, so the descent could plan while the cube was
still attached to tool0 - where at place height its underside rests on the
ground plane, so collision-checked IK rejected everything (-31) and the cycle
bailed into recovery, which then *dropped* the cube from 0.32 m rather than
placing it. `_apply_scene`/`_scene_remove_cube` take an optional `then`, and
both descents are now sequenced behind the diff landing.

### A partial straight line is worth executing
Even sequenced, the place descent intermittently solved only ~32-39%: the free
plan into PLACE lands in whichever IK branch it likes, and from some of them the
rest of the vertical line is unreachable. Falling back to a pose goal then failed
collision-checked IK and aborted the cycle. Now a partial path above
`CARTESIAN_RETRY_MIN_FRACTION` is executed and the remainder re-requested from
the new configuration (up to `CARTESIAN_MAX_ATTEMPTS`). It fired twice in the
acceptance runs and both recovered to a clean 100% descent.

### Also
- `RETRACT_JOINTS` was byte-identical to `HOME_JOINTS`, so the dashboard's Home
  and Retract buttons did exactly the same thing. Retract is now a real park
  pose (pan +90 deg, arm folded) clear of the workspace and the camera.
- A deliberate stop reported `last_result: failed`; it now reports `stopped`.

## 2026-08-12: zones, faster motion, multi-cube perception, tidy task

Dashboard gained a `tidy` task: sweep every cube outside a red square on the
ground into slots inside it. Zones, multi-cube tracking and a large speed-up
came with it. Partly working - see "Not finished" at the end.

### Speed: 5-10x
`joint_limits.yaml` velocities were 0.35/0.5 rad/s and `CARRY_VEL_SCALE` 0.15,
set for a friction grasp that could be shaken loose. The grasp is a rigid plugin
attach now, so none of that applies. Now 1.8/2.5 rad/s and 0.6/0.4 scaling.
Measured: home 30 s -> 1.5 s, retract 74 s -> 3.5 s, pick_place ~150 s -> 12 s.
This was not cosmetic - cycles were slow enough that test harnesses timed out
and commands came back "busy", which is what made a full run look broken.

### Reach zones
`zone_of()` classifies a table position as too_close (<0.24 m), perfect, or
too_far (>0.50 m) by radius from base_link. Bounds come from measurement: every
good pick has been between r=0.405 and r=0.474. Cubes spawn in the perfect
annulus, picks outside it are refused, and the dashboard shows each cube's zone.

### Drop zone and multi-cube
Three cubes plus a red square outline (`drop_zone_*` in the world, centre
(0.30, 0.22), 160 mm side, all four corners inside the perfect annulus).
Perception detects all cubes and gives them ids that survive occlusion, and the
orchestrator publishes every non-target cube to MoveIt as an obstacle - without
that the arm planned straight through them and shoved two out of the workspace.

### The camera was the limiting factor
At 640x480 from 3 m, one pixel covered 5.7 mm of table and a cube spanned ~10 px,
giving a ~10 mm position error. The open gripper leaves only 12.4 mm of clearance
per side around a 60 mm cube, so that error was enough for a pad to clip a corner
on the descent and drag the cube 57 mm instead of grasping it (caught by tracing
Gazebo poses against the state machine, not guessable from the endpoints).
1600x1200 brings the error to 3-6 mm.

### Descent length, not descent retries
The straight-line descent solved only ~50% in a scattered scene, and the retry
made it *worse*: it raised the hover height, which lengthens the line, so every
retry was harder than the last (99 re-approaches and 17 hard failures in one
sweep). `APPROACH_HEIGHT` 0.30 -> 0.22 makes it a 70 mm line and retries now
lower the hover. Result: 0 partial solves, 0 re-approaches, 0 failures.

### Two loops that could not terminate
- Blacklisting an unpickable cube by perception id is useless: ids are reassigned
  when a cube is occluded or shoved, so each retry looked like a new cube. One
  sweep reached id #59 and ran 10 minutes. Blacklist is keyed on a 50 mm position
  grid now, plus a hard `TIDY_MAX_ATTEMPTS` cap.
- A cycle that shoved a cube instead of carrying it still reported success, so
  the sweep claimed 2 placed when 1 had arrived. `_verify_last_place()` now
  checks the slot on the next scan and only counts confirmed deliveries.

### Also
- FastDDS forced to UDP-only (`config/fastdds_udp_only.xml`). Killed processes
  leave locked /dev/shm segments and the *next* launch comes up with no TF at
  all - the arm just sits at home. That cost two sessions to diagnose twice.
- The orchestrator wipes move_group's planning scene at startup. move_group
  outlives the node, so a restart inherited a cube still attached to tool0 and
  every plan returned INVALID_MOTION_PLAN.
- Orange HSV band narrowed to H 10-25 so the red drop-zone marker is not
  detected as a cube.

### Not finished
The tidy sweep reliably delivers about 1 cube in 3. The descent and planning
problems are fixed (0 failures), and reporting is now honest, but a pick still
sometimes displaces its cube rather than grasping it - last run left one cube
pushed to (0.165, 0.080), inside the too_close zone. Perception error (3-6 mm)
is well inside the 12.4 mm clearance, so the remaining cause is not resolution;
the next thing to instrument is where in APPROACH/GRASP the contact happens, per
cube, with the Gazebo pose trace used above.

### Tidy working: 3/3 (update to the section above)
Three more faults, all found by asking "is use_grasp_fix actually doing what we
think" rather than tuning further:

1. **The grasp plugin managed exactly one object.** `FindObject()` took the
   first non-static model in the world and bound to it forever, making only that
   cube kinematic and grabbable. Invisible with a single cube; with three, the
   other two were ordinary dynamic bodies the gripper could only shove. The
   signature was unmistakable in hindsight: cube_1 was the one that always
   worked. It now manages every graspable model and attaches the *nearest* one
   inside `grasp_radius`.
2. **The end-of-cycle pose moved across the workspace.** `_finish` parked at
   `RETRACT_JOINTS`, which was byte-identical to `HOME_JOINTS` until Retract
   became a real park pose (pan +90 deg). After that, every approach following a
   place needed a ~2.1 rad reconfiguration and was rejected by the IK jump
   limit, so every pick after the first failed. Parks at HOME now.
3. **compute_ik returns wrapped solutions.** It works from the URDF's +/-2*pi
   limits, not the +/-pi in joint_limits.yaml, so it returned a pose a whole
   revolution out - physically identical, but the jump check saw 6.07 rad and
   refused it. IK solutions are unwrapped to the equivalent nearest the current
   joint angle before the check.

Verified against Gazebo ground truth, not perception: cube_1 carried 400 mm,
cube_2 78 mm, cube_3 457 mm, 3/3 inside the square in 38 s, with 0 ERROR states,
0 IK rejections and 0 descent retries.

### Tidy: works, but only about half of picks plan successfully
Across four runs after the grasp-plugin fix: 3/3, 2/3, 1/3, 3/3. The failures are
now *clean* - a pick that fails leaves its cube untouched at its spawn position
and the sweep skips it, rather than shoving it out of the workspace. That is the
qualitative change from the plugin fix; the cubes are kinematic and pinned, so
nothing can knock them any more.

What still fails is planning the APPROACH to certain cube positions:
`MoveIt2 failed: error_code=-2` followed by an IK solution rejected as too large
a reconfiguration (2.6 rad, and some above the raised 3.0 limit). The IK seed is
now complete (arm + gripper joint, is_diff=False) and wrapped solutions are
unwrapped, which removed one whole class of these, but a genuine branch flip
remains for cubes behind the robot.

Next thing to try: constrain the approach plan rather than filter its result -
either a joint-space goal derived from a known-good seed per workspace sector,
or set the OMPL planner's start state explicitly and let it plan the whole
transfer instead of pose-goal-then-IK-fallback.

## 2026-08-12 (later): emergency stop, reset, and cube selection

### Emergency stop is a halt, not a recovery
`stop` was the only interrupt and it is not an emergency control: it drops into
ERROR recovery, which *opens the gripper and drives the arm back to HOME*. It
makes the robot move more, and drops whatever it is holding.

`estop` halts where it stands, latches, and refuses every command except
`reset`. `reset` clears the latch, wipes the planning scene and homes the arm
ready for the next command. Both are on the dashboard; estop is always accepted,
even mid-cycle and even while already latched.

Cancelling the action goals turned out to be necessary but nowhere near
sufficient - measured 2.54 rad of further travel after the cancel, because once
a trajectory is with joint_trajectory_controller it keeps executing. The halt
also publishes a single-point JointTrajectory at the current joint positions,
which supersedes the running goal. Measured after that change: 0.0001 rad of
drift over the 4 s following the stop, against 0.155 rad of deceleration during
the stop itself.

### Pick only the cubes you choose
`RunTask` gains `int32[] ids` and a `pick_selected` task, which runs the tidy
sweep restricted to those perception ids. The dashboard lists every detected
cube with its zone and a checkbox; cubes outside the perfect zone or already in
the drop zone are shown but not selectable. Ticks survive the 400 ms poll, which
they would not if the list were simply re-rendered.

Verified: selecting one of three cubes moved that cube into the square and left
the other two untouched at their spawn positions; an empty selection and an
unknown id are both refused.

### Known cosmetic issue
A `pick_selected` run that succeeds can still report "1 placed, 1 skipped" - the
skip count picks up an entry it should not when the selection is satisfied. The
physical outcome is correct; only the summary line is wrong.

## 2026-08-12: RViz zone markers and cube spawning

`/ur3e/zone_markers` (MarkerArray, transient-local so a late RViz still gets it)
draws the reach zones as stacked translucent discs - green at ZONE_FAR with the
red too_close disc on top, so the middle reads as an annulus without building
one - plus an orange outline at the far limit, the red drop-zone square and the
four slot footprints. `view_robot.rviz` has the display; launch with
`./run_sim.sh gui rviz`.

`spawn_cube` drops a new cube at a free spot in the perfect zone, clear of the
drop zone and 140 mm from any existing cube. Its SDF matches the pre-placed
cubes exactly (same mass, inertia and kp=1e5 contact) - a spawned cube that
behaved differently would make every later test ambiguous. Button on the
dashboard; the task returns immediately rather than occupying the state machine.

Verified: 8 markers on the topic (3 zones, drop zone, 4 slots); a spawned cube
appears in Gazebo as cube_spawned_1 and is detected as a new tracked id in the
perfect zone.

## 2026-08-12: full system acceptance 32/32

`full_system_test.py` exercises the whole cell - startup, rejections, simple
motions, spawn-then-pick-that-cube, estop mid-motion, reset, pick_to, tidy-all -
and checks every physical claim against /gazebo/model_states rather than
perception, since perception is one of the things under test.

The last two failures were both the IK reconfiguration guard. MAX_IK_JOINT_JUMP
was rejecting *legitimate* moves: with joints capped at +/-pi by
joint_limits.yaml, travelling from +3.0 to -3.0 is a real 6.0 rad journey
because the planner cannot wrap through pi. The guard dated from when IK
solutions were streamed straight to the trajectory controller as a single point
with no path checking, where a large reconfiguration genuinely could drive the
arm through itself. That path no longer exists - solutions are planned through
MoveIt, which collision-checks them and enforces the joint limits - so the guard
was pure downside. Raised to 6.5 rad, and the rejection message now names the
offending joint and its current/target angles.

Result: 32/32, including pick_to landing a cube within 0 mm of the requested
point and tidy putting 4/4 cubes (three placed plus one spawned mid-run) inside
the red square.

### The real cause: wrist_3 was winding up
The 32/32 above was luck - the next run was 30/32 again. The detailed rejection
message (which now names the offending joint) gave it away:

    worst joint wrist_3_joint  -8.32 -> -2.10

wrist_3 was at **-8.32 rad**, two and a half turns outside the +/-pi that
joint_limits.yaml allows. compute_ik works from the URDF's +/-2*pi limits, the
solution was executed exactly as returned, and every cycle added another
fraction of a turn. Once the joint is outside the planner's range MoveIt cannot
plan *from* that state at all, which is where the INVALID_MOTION_PLAN failures
came from too - and the arm looked fine, because -6.28 rad is physically the
same orientation as 0.

`_nearest_equivalent` had a `return best if abs(best) <= math.pi else q` escape
hatch that handed the raw out-of-range value straight back; that was the leak.
It now always returns `atan2(sin q, cos q)`, and `_move_joints` normalises its
targets the same way so a wound joint is commanded back into range instead of
being left there. MAX_IK_JOINT_JUMP raised to 6.5 so the unwinding move itself
is permitted.

(The earlier attempt to raise that constant silently failed to apply - the log
still read "limit 3.0" - which is why the first fix appeared not to work.)

Clean run afterwards: 32/32, wrist_3 back at -0.01, and 0 ERROR states and 0 IK
rejections underneath, where the lucky pass had 1 of each.

### Where the wind actually came from, and the deadlock it caused
Normalising IK solutions and joint goals was not enough - the next run wound
wrist_3 to -12.59 rad, four turns. The remaining source is the Cartesian
segments: a straight-down tool orientation leaves wrist_3 unconstrained, and
compute_cartesian_path's solution is executed exactly as returned, so it creeps
a little every cycle.

That created a deadlock. Once a joint is outside the +/-pi in joint_limits.yaml,
MoveIt refuses to plan *from* that state, so the arm cannot be commanded back
into range - including to HOME. Every motion fails from then on and the only way
out was restarting the stack.

`_move_joints` now checks whether any joint has wound out of range and, if so,
drives to the normalised target through the trajectory controller directly,
which has no such limit. It is the one place bypassing MoveIt is correct: the
target is the same physical pose, and the alternative is an arm that cannot move
at all.

Three consecutive full runs afterwards: 32/32, 32/32, 32/32, wrist_3 ending at
-0.01 / -0.03 / 0.02, 0 ERROR states in all three. The third logged one unwind,
so the safety net is demonstrably firing and recovering rather than the problem
merely not recurring.

## 2026-08-12: direct transfer between picks - implemented, NOT yet effective

Skipping the HOME trip between picks of a multi-cube sweep is in
`_finish` behind a `direct_transfer` parameter (default true), guarded by
`_tidy_active` and more than one cube still to fetch.

It does not fire. Three full runs logged `skips=0`. The branch is present in the
built copy and the parameter is declared, so the condition
`len(self._pickable_cubes()) > 1` is simply not true at `_finish` time and I have
not yet established why - `self._cubes` is the pre-cycle snapshot there, so it
should still list every cube outside the drop zone. Needs one instrumented run
logging the pickable count at `_finish` to settle it.

The half of the change that *is* working is the winding guarantee moving to the
approach. It used to be a side effect of visiting HOME; `_approach` now checks
for an out-of-range joint itself and unwinds first. That fired in two of the
three runs, so the property no longer depends on a pose the sweep may skip -
which is what made the optimisation safe to attempt at all.

No regression: runs 2 and 3 were 32/32; run 1 was 30/32 with the same
intermittent approach-planning failures seen before this change.

## 2026-08-12: "pick all" stacked every cube in slot 1

Reported from use: `tidy` put all cubes on top of each other at slot 1, while
picking them one at a time with `pick_selected` spread them correctly. Measured
from Gazebo: two cubes at 0 mm separation.

`_free_slot()` judged occupancy purely from perception, and at HOME the arm
reaches out over the workspace and can shadow the drop zone from the overhead
camera. A cube just delivered was therefore invisible when the next slot was
chosen, so slot 1 read as free every time. One-at-a-time picking hid it because
the arm ends up elsewhere between commands and the view is clear.

The sweep now reserves a slot when it commits to one, independent of whether it
can see anything. First attempt at that still stacked, because the reservation
was released whenever `_verify_last_place()` reported nothing had arrived - and
that verdict comes from the same blind perception, so it fired exactly when the
zone was occluded and handed the next cube the same slot. The reservation now
survives a failed verify: losing a slot to a false negative is much cheaper than
putting two cubes in one place.

Three runs after the fix: 1/1, 3/3 and 3/3 cubes placed, all in distinct slots,
closest pair 89 mm against a 60 mm cube on a 90 mm pitch. (The 1/1 run is the
separate intermittent approach-planning failure, not stacking.)

### The suite gave a false green
`full_system_test.py` checked that cubes ended up *inside* the square but never
that they were in *different* slots, so three cubes stacked at slot 1 passed
32/32. A closest-pair check is now part of the suite.
