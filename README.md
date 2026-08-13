# UR3e Pick-and-Place Cell

A simulated robotic work cell: a Universal Robots UR3e with a Robotiq 2F-85
gripper, an overhead camera, and a web dashboard to drive it. The robot finds
cubes on the table, decides which are reachable, and sorts them into a marked
drop zone — either all of them, or only the ones you select.

ROS 2 Humble · Gazebo Classic 11 · MoveIt 2

![demo](demo/full_demo.webm)

**Demos:** [`demo/full_demo.webm`](demo/full_demo.webm) — the dashboard driving a
full multi-cube sweep. [`demo/pick_and_place.mp4`](demo/pick_and_place.mp4) — an
earlier single-cube cycle from the overhead camera.

---

## What it does

**Sees.** An overhead camera feeds a colour-segmentation pipeline that finds
every cube, projects it onto the table plane through the camera transform, and
assigns each one an ID that survives the arm passing overhead. Stable IDs matter
because the orchestrator commits to "cube 4" and cannot have that renamed
mid-cycle.

**Decides.** The table is split into three reach zones by radius from the base:
too close, workable, too far. Cubes are classified on sight and anything outside
the workable band is refused rather than attempted. The drop zone is a red
square with four slots on a fixed pitch, so placed cubes never land on each
other.

**Acts.** A state machine runs each cycle: approach, straight-line descent,
grasp, lift, transfer, straight-line place, release, depart. Descents and lifts
are Cartesian so the tool travels vertically instead of arcing sideways into the
object. Every cube not currently being picked is published to MoveIt as a
collision object, so the arm plans around them rather than through them.

## Tasks

Driven from the dashboard, or over HTTP.

| Task | What it does |
| --- | --- |
| `tidy` | Sweep every reachable cube into the drop zone, one at a time |
| `pick_selected` | Same, but only the cube IDs you tick |
| `pick_place` | One cube to the default target |
| `pick_to` | One cube to coordinates you type in |
| `spawn_cube` | Drop a new cube into a free spot in the workable zone |
| `home` / `retract` | Move to the home pose, or park clear of the workspace |
| `open_gripper` / `close_gripper` | Direct gripper control |
| `stop` | End the cycle tidily: reopen the gripper, return home |
| `estop` | Halt where it stands and latch |
| `reset` | Clear the latch, wipe the planning scene, return home |

## Dashboard

`http://127.0.0.1:8080` — a ROS node plus Python's standard-library HTTP server.
No FastAPI, no rosbridge, nothing to `pip install`; it runs from a bare Humble
install. The browser polls a single JSON endpoint.

Live state and task result, every detected cube with its zone and a checkbox to
select it, joint positions, a drop-zone target readout, an activity feed, and
liveness indicators for the orchestrator and camera. Task buttons disable while
the robot is busy; the emergency stop never does.

```
GET  /api/state    orchestrator status, cubes, zones, joints, activity log
POST /api/task     {"task": "tidy"}  ·  {"task": "pick_selected", "ids": [2,4]}
```

## Emergency stop

Worth calling out because the obvious implementation does not work. Cancelling
the in-flight action goals is necessary but not sufficient — a trajectory
already handed to the controller keeps executing, and the arm carries on for
several radians. The stop also commands the controller to hold its current
position, which is what actually brings it to rest. Measured drift after a stop:
0.0001 rad.

It then latches: every command except `reset` is refused until you clear it.
This is deliberately distinct from `stop`, which is a tidy recovery — that one
opens the gripper and drives home, which is the wrong response to an emergency.

## Grasping

Gazebo Classic's contact solver does not hold a light box in a parallel gripper
reliably. The gripper closes on the cube physically, and a plugin fixes the
nearest object to the tool once the driver knuckle passes a threshold, releasing
it when the gripper reopens. The motion looks and behaves like a real grasp;
only the hold is plugin-assisted.

Contact stiffness is tuned on both surfaces — cube and finger pads — because ODE
combines them and softening only one still leaves a hard contact.

## Visualisation

RViz displays the three reach zones as translucent discs, the drop-zone square,
and the four slot footprints, published as a latched `MarkerArray` so a
late-starting RViz still receives them.

## Layout

```
src/
  ur3e_msgs/           interfaces, including the RunTask service
  ur3e_sim_bringup/    URDF/xacro, world, controllers, MoveIt config, launch
  ur3e_perception/     camera pipeline, cube detection and tracking
  ur3e_motion/         orchestrator state machine, task interface
  ur3e_end_effectors/  gripper state reporting
  ur3e_dashboard/      web UI and HTTP server
  ur3e_gazebo_plugins/ grasp and mimic-joint plugins
demo/                  recorded runs
full_system_test.py    end-to-end acceptance suite
```

## Running it

```bash
colcon build --symlink-install
./run_sim.sh gui          # Gazebo + robot + MoveIt + dashboard
./run_sim.sh gui rviz     # and RViz with the zone overlays
./stop_sim.sh
```

Then open `http://127.0.0.1:8080`.

The orchestrator idles until told what to do. Pass `auto:=true` to
`motion.launch.py` if you want it to loop pick-and-place on its own.

## Testing

```bash
python3 full_system_test.py
```

Around thirty checks covering startup, input validation, every task, emergency
stop and reset. Every physical claim is verified against Gazebo's own model
state rather than against the perception pipeline, since that pipeline is one of
the things under test — including that placed cubes end up in *different* slots,
not merely inside the square.

`PROGRESS.md` carries the engineering history: the non-obvious constraints that
make this work in Gazebo Classic, and why several settings are what they are.
