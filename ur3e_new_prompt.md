# UR3e Pick-and-Place — Master Project Prompt

## 0. How to Read This Document

This is a **cold-start system prompt**. It contains everything a coding AI needs to rebuild this project from scratch with zero additional context. Read it linearly. Every rule is binding. If a section says "must" or "must not", follow it literally. If a section says "prefer", use judgment.

The document is organized so that **prime directives override everything else**. Architecture decisions come before feature details. Session protocol determines *how* you work; coding standards determine *what you produce*.

When you begin a new feature, load this document first, then load the **Feature Backlog** table to confirm which feature is active, then load the **Session Protocol** section and follow it step by step.

---

## 1. Prime Directives — Non-Negotiable Rules

These override all other instructions in this document.

1. **Always ask before coding if there is ambiguity, a blocker, or a decision to make.** Never guess. Never assume.
2. **Never push to Git without explicit user permission.** Flag the feature as done and wait.
3. **When something fails: diagnose, debug, retry once automatically. If it fails again, explain what happened and ask for instructions.** This includes colcon build failures, launch failures, MoveIt2 planning failures, missing data, etc.
4. **If you discover something that should have been done in an earlier feature, flag it, say whether it blocks current work, and ask whether to fix it now or defer.**
5. **Prefer the simplest solution.** No over-engineering. No extra abstractions. No premature optimization.
6. **Never hardcode paths, IPs, or ports.** Use environment variables with defaults.
7. **Every node must use `rclpy` logging — no bare `print()` or `logging` module.** All timing must use `rclpy.timer`; no `time.sleep()`.
8. **Every node must honor `use_sim_time`** — read the parameter and pass it to the node constructor.
9. **All workspace paths are relative to the workspace root.** No absolute paths in code.
10. **Each feature lives in its own Git branch** named `feature/N-description` matching the Feature Backlog. Branches are created by the user, not the AI.

---

## 2. Project Overview

### What It Is
A ROS 2 Humble + Gazebo Classic 11 simulation of a **UR3e robotic arm** performing autonomous pick-and-place. The arm detects objects on shelf zones via an overhead camera (ArUco markers for pose + YOLOv8 for class labels), plans motions with MoveIt2, executes with ros2_control, and is orchestrated by a py_trees_ros behavior tree. A web dashboard (FastAPI + rosbridge) provides real-time status and manual controls.

### Scope
- Simulation only (no real hardware).
- Single-arm, single-camera, two shelf zones, one drop zone.
- Gripper end effector (vacuum as stretch goal).
- 11 features built sequentially, each on its own branch.

### What Success Looks Like
- The arm completes a full pick-and-place cycle autonomously.
- **At least 8 out of 10 consecutive picks succeed** (object is picked from shelf and placed in drop zone).
- The dashboard shows live status of the BT, detected objects, and robot state.
- A viewer (RViz or Gazebo GUI) can observe the entire cycle.
- The workspace builds cleanly with `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.

### End User / Judge
- **You** — this is a portfolio piece. No external client or competition.

---

## 3. Architecture

### 3.1 Layered Diagram

```
┌──────────────────────────────────────────────┐
│              DASHBOARD LAYER                  │
│  dashboard_server (FastAPI + WebSocket)      │
│  └─ roslibpy → rosbridge_server              │
│     Static HTML served on :8080              │
├──────────────────────────────────────────────┤
│            ORCHESTRATION LAYER                │
│  pick_place_bt_node  (py_trees_ros wrapper)  │
│  └─ pick_place_bt.py  (behavior tree def)    │
├──────────────────────────────────────────────┤
│              MOTION LAYER                     │
│  motion_executor  (MoveIt2 action clients)   │
│  scene_initializer (planning scene objects)  │
├──────────────────────────────────────────────┤
│           PERCEPTION LAYER                    │
│  perception_node (ArUco + YOLOv8 + fusion)   │
│  └─ /camera/image_raw → detected_objects     │
├──────────────────────────────────────────────┤
│          END EFFECTOR LAYER                   │
│  gripper_state_node                          │
│  vacuum_controller_node                      │
├──────────────────────────────────────────────┤
│            SIMULATION LAYER                   │
│  Gazebo Classic 11  │  ros2_control          │
│  robot_state_publisher │  joint_state_broad. │
│  /camera/image_raw   │  /joint_states        │
│  /clock (use_sim_time)                       │
├──────────────────────────────────────────────┤
│            INTERFACES (ur3e_msgs)            │
│  Topics, Services, Actions, Msgs, Srvs      │
└──────────────────────────────────────────────┘
```

### 3.2 Communication Flows

| Flow | From | To | Transport |
|---|---|---|---|
| Camera feed | Gazebo camera | `/camera/image_raw` | ROS 2 topic |
| Detections | `perception_node` | `/perception/detected_objects` | ROS 2 topic (msg) |
| Pick goal | `pick_place_bt_node` | `/perception/assign_pick_goal` | ROS 2 service |
| Motion plan | `motion_executor` | MoveIt2 `/plan_kinematic_path` | ROS 2 action |
| Execute trajectory | `motion_executor` | `/joint_trajectory_controller/follow_joint_trajectory` | ROS 2 action |
| Gripper command | `motion_executor` | `/gripper_controller/gripper_cmd` | ROS 2 action |
| BT status | `pick_place_bt_node` | `/bt/status` | ROS 2 topic (msg) |
| BT start/stop | `dashboard_server` | `/bt/start`, `/bt/stop` | ROS 2 service |
| Dashboard data | `dashboard_server` | WebSocket → browser | rosbridge JSON |
| Planning scene | `scene_initializer` | `/apply_planning_scene` | ROS 2 service |
| TF | `robot_state_publisher` | `/tf`, `/tf_static` | ROS 2 topic |

### 3.3 Decisions Table

| Decision | Choice | Rationale |
|---|---|---|
| Orchestration | py_trees_ros | Visualizable, deterministic, strong ROS 2 integration; impressive portfolio piece |
| Motion API | Raw MoveIt2 action/service clients | moveit_commander / moveit_py unavailable in Humble |
| Perception | 1 merged node (ArUco + YOLO + fusion) | Reduced IPC, simpler launch, one less package |
| Safety layer | Dropped | Over-engineered for a portfolio sim demo |
| Dashboard | FastAPI + roslibpy | Cross-browser, no extra ROS bridge deps |
| End effector start | Gripper, vacuum as stretch | Gripper is simpler; vacuum adds contact sensor complexity |
| Message package | ament_cmake (C++) | Required for ROS 2 msg/service generation |
| All other packages | ament_python (Python) | Minimizes CMake burden |

### 3.4 Failure & Retry Policy

> This is the canonical retry policy. Every node and every operation follows it.

| Scenario | Action |
|---|---|
| `colcon build` fails | Read the error, fix the root cause, retry once. If it fails again, explain the error and ask for instructions. |
| Gazebo launch crash (segfault, RTShader, missing model) | Check setup sourcing (gazebo_setup.sh, LIBGL_ALWAYS_SOFTWARE), retry once. If repeatable, flag and ask. |
| MoveIt2 IK failure or planning timeout | Log the failed pose, retry with relaxed tolerances once. If still failing, log and return failure to BT (do not halt the node). |
| Missing camera image (empty /camera/image_raw) | Log warning, publish empty detection array, return success (no crash). |
| ArUco not detected / YOLO not detected | Publish partial detections if one system succeeds. If both fail, publish empty array. Never crash. |
| Service call timeout | Retry once with longer timeout. If still failing, log error and return failure. |
| Action goal rejected (e.g., gripper busy) | Wait 0.5 s (via timer, not sleep) and retry once. |
| Any unhandled exception | Catch, log full traceback, set node state to ERROR, publish on diagnostics topic (if exists) or log. Do not crash the process. |

---

## 4. Repository Structure

```
ur3e/
├── src/
│   ├── ur3e_msgs/                    # [ament_cmake] Custom ROS 2 interfaces
│   │   ├── msg/
│   │   │   ├── ArucoDetection.msg
│   │   │   ├── ArucoDetectionArray.msg
│   │   │   ├── YoloDetection.msg
│   │   │   ├── YoloDetectionArray.msg
│   │   │   ├── DetectedObject.msg
│   │   │   ├── DetectedObjectArray.msg
│   │   │   ├── BTStatus.msg
│   │   │   └── SafetyCommand.msg
│   │   ├── srv/
│   │   │   ├── AssignPickGoal.srv
│   │   │   └── GetBTStatus.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── ur3e_end_effectors/           # [ament_python]
│   │   ├── ur3e_end_effectors/
│   │   │   ├── __init__.py
│   │   │   └── nodes/
│   │   │       ├── __init__.py
│   │   │       ├── gripper_state_node.py
│   │   │       └── vacuum_controller_node.py
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── ur3e_sim_bringup/             # [ament_python]
│   │   ├── ur3e_sim_bringup/
│   │   │   ├── __init__.py
│   │   │   └── scene_initializer_node.py
│   │   ├── launch/
│   │   │   ├── bringup.launch.py
│   │   │   └── ur3e_pickplace.launch.py
│   │   ├── urdf/
│   │   │   └── ur3e_with_effector.urdf.xacro
│   │   ├── worlds/
│   │   │   └── ur3e_workcell.world
│   │   ├── config/
│   │   │   └── moveit/
│   │   │       ├── ur3e.srdf
│   │   │       ├── ur3e.srdf.xacro
│   │   │       ├── kinematics.yaml
│   │   │       ├── ompl_planning.yaml
│   │   │       ├── pilz_industrial_motion_planner.yaml
│   │   │       ├── joint_limits.yaml
│   │   │       └── moveit_controllers.yaml
│   │   ├── robots/
│   │   │   └── ur3e.urdf.xacro (or ros2_control config)
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── ur3e_perception/              # [ament_python]
│   │   ├── ur3e_perception/
│   │   │   ├── __init__.py
│   │   │   └── perception_node.py
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── ur3e_motion/                  # [ament_python]
│   │   ├── ur3e_motion/
│   │   │   ├── __init__.py
│   │   │   ├── pick_place_bt.py
│   │   │   ├── pick_place_bt_node.py
│   │   │   └── motion_executor.py
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── ur3e_dashboard/               # [ament_python]
│       ├── ur3e_dashboard/
│       │   ├── __init__.py
│       │   └── dashboard_server.py
│       ├── static/
│       │   └── index.html
│       ├── package.xml
│       └── setup.py
│
├── yolov8n.pt               # YOLOv8 model (download if missing at runtime)
├── PROGRESS.md               # Bug log and integration history
├── INTEGRATION_NOTES.md      # Build/launch reference
├── MASTER_PROMPT.md          # This file
└── AGENTS.md                 # Reusable lessons for the AI
```

---

## 5. Skill / Module Contract Template

Each node in the system must document itself using this template in its file header comment (or its README if standalone):

```
# =============================================================================
# Module:    <node_name>
# Purpose:   <one-line description>
# Publishes: <topic> [<msg_type>] — <what it contains>
# Subscribes: <topic> [<msg_type>] — <what it expects>
# Services:  <service_name> [<srv_type>] — <what it does>
# Actions:   <action_name> [<action_type>] — <client or server>
# Parameters (declared):
#   ~param_name (type, default) — description
# Failure mode: <what happens if X fails>
# =============================================================================
```

---

## 6. Feature Backlog & Branch Map

| # | Feature | Branch | Depends On | Status |
|---|---|---|---|---|
| 1 | Workspace scaffold & ur3e_msgs | `feature/1-workspace-scaffold` | — | ▢ |
| 2 | Robot bringup (Gazebo, URDF, controllers, TF) | `feature/2-robot-bringup` | 1 | ▢ |
| 3 | MoveIt2 config (SRDF, planners, move_group) | `feature/3-moveit-config` | 2 | ▢ |
| 4 | End effectors (gripper + vacuum nodes) | `feature/4-end-effectors` | 1, 2 | ▢ |
| 5 | Perception (merged ArUco + YOLO + fusion) | `feature/5-perception` | 1 | ▢ |
| 6 | Scene initializer (planning scene objects) | `feature/6-scene-initializer` | 3 | ▢ |
| 7 | Motion executor (MoveIt2 action clients) | `feature/7-motion-executor` | 3, 4 | ▢ |
| 8 | Behavior tree (BT definition + node wrapper) | `feature/8-behavior-tree` | 7, 5 | ▢ |
| 9 | Integration launch (single launch file) | `feature/9-integration-launch` | 2, 5, 7, 8 | ▢ |
| 10 | Dashboard (FastAPI + rosbridge) | `feature/10-dashboard` | 8, 9 | ▢ |
| 11 | Polish & tuning (8/10 success) | `feature/11-polish-tuning` | 9, 10 | ▢ |

**Status key:** ▢ = pending, ▣ = in progress, ✓ = completed, ✗ = cancelled

After each feature is done, the AI marks it ✓ and asks the user "Ready for feature N+1?" The user decides whether to push, review, or continue.

---

## 7. Session Protocol

The AI follows these exact steps **every time it starts work on a feature**, in order.

### Step 1 — Identify
- Read this MASTER_PROMPT.md.
- Read the Feature Backlog table. Confirm which feature is active.
- If a FEATURE_BRIEF.md exists for this feature, read it. Otherwise, confirm with the user before starting.

### Step 2 — QA
- Ask the user any questions about the feature before writing code.
- Questions must be specific and answerable with a short sentence or choice.
- Do not ask "how should I implement X?" — instead ask "For X, option A does Y, option B does Z — which do you prefer?"

### Step 3 — Plan
- Explain in 2-4 sentences what you will build and in what order.
- Ask the user to confirm the plan before writing code.

### Step 4 — Implement
- Build the feature. Follow the Coding Standards.
- Prefer existing patterns in the codebase over new patterns.
- After each file is written, do a quick self-review for correctness.
- Run `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` at the end.
- If the build fails, apply the **Failure & Retry Policy** (Section 3.4).

### Step 5 — Verify
- Confirm the build passes cleanly.
- If the feature can be launched (e.g., a standalone node), do a quick smoke test with `ros2 run`.
- Log any issues to PROGRESS.md.

### Step 6 — Handoff
- Report to the user:
  - What was built (list files created/modified).
  - Build result (pass/fail).
  - Any blockers, decisions deferred, or issues discovered.
  - Any environment variables added or changed.
- Ask: "Ready for feature N+1?"
- **Do not push.** Wait for the user's instruction.

---

## 8. Coding Standards

### 8.1 Python

| Rule | Standard |
|---|---|
| Formatter | `black` (line length 88) |
| Linter | `ruff` |
| Naming — files | `snake_case.py` |
| Naming — functions/vars | `snake_case` |
| Naming — classes | `PascalCase` |
| Naming — constants | `UPPER_SNAKE` |
| Type hints | Required on all function signatures |
| Docstrings | Google style on public functions only |
| Logging | `self.get_logger().info/warn/error` (rclpy) |
| No-nos | `print()`, `time.sleep()`, bare `except:`, mutable default args |
| Imports order | stdlib → third-party → rclpy → local |
| Line length | Max 88 characters |

### 8.2 C++ (ur3e_msgs only)

| Rule | Standard |
|---|---|
| Formatting | `clang-format` with Google style |
| Naming | `snake_case` for files, `CamelCase` for types |
| Includes | Use `#pragma once` |
| Comment style | `//` for inline, `/* */` for blocks |

### 8.3 ROS 2

| Rule | Standard |
|---|---|
| Node naming | `snake_case_node` (file) = `SnakeCaseNode` (class) |
| Parameter naming | `~snake_case` with underscores |
| Topic naming | `/snake_case/with/forward/slashes` |
| Service naming | `/snake_case/service_name` |
| Launch files | Python (`.launch.py`), ComposableNodeContainer pattern |

### 8.4 Git

| Rule | Standard |
|---|---|
| Branch naming | `feature/N-description-with-dashes` |
| Commit messages | Conventional commits: `feat:`, `fix:`, `chore:`, `docs:`, `refactor:` |
| Commit body | Explain *why*, not *what* (the diff shows what) |
| Commits per feature | As many as needed, but each must build |

### 8.5 File Headers

No license or copyright block required. Each file starts directly with code (or a `# =====` module contract if it's a node).

---

## 9. Active Feature Brief

When a feature is active, a `FEATURE_BRIEF.md` file may exist in the workspace root with specific instructions for that feature. If it exists, load it and follow it in addition to this master prompt. If it does not exist, proceed with the master prompt alone.

The FEATURE_BRIEF.md is created and removed by the user as needed. Do not create it yourself.

No feature-specific instructions are hardcoded in this document. All features are defined solely by the Feature Backlog table and the Architecture sections above.

---

## 10. Environment Variables Reference

```dotenv
# ROS 2
ROS_DOMAIN_ID=0

# Simulation
USE_SIM_TIME=true
GAZEBO_GUI=true

# Dashboard
HTTP_PORT=8080
ROSBRIDGE_PORT=9090

# Perception
YOLO_MODEL=yolov8n.pt
YOLO_CONFIDENCE=0.5
ARUCO_DICT=4x4_100

# Hardware / End Effector
END_EFFECTOR=gripper

# Motion Planning
MOVEIT_PLANNER=ompl
```

All variables have sensible defaults. Nodes must use `os.getenv(key, default)` — never require explicit export.

---

## 11. KPIs / Acceptance Criteria

A feature is **done** when:

1. **Build passes** — `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` exits 0 with no errors.
2. **The node(s) can be launched** — `ros2 run` or `ros2 launch` starts without crashing.
3. **Topics/services/actions are advertised** — `ros2 topic list`, `ros2 service list`, `ros2 action list` show the expected interfaces.
4. **No `time.sleep()` calls** exist in any node.
5. **`use_sim_time` parameter is wired** through every node.

The overall project is **done** when:

1. A full autonomous pick-and-place cycle completes in simulation.
2. **At least 8 out of 10 consecutive picks place the object in the drop zone.**
3. The dashboard shows BT status, detected objects, and robot state in real time.
4. The entire system starts with a single `ros2 launch` command.

---

## 12. Project-Specific Notes

### Simulation Quirks
- **GPU OpenGL crashes** on this machine. Always set `export LIBGL_ALWAYS_SOFTWARE=1` before launching Gazebo.
- **Gazebo RTShaderSystem crash** — source `/usr/share/gazebo/setup.sh` before `ros2 launch` to avoid `boost::shared_ptr<Scene> px != 0` assertion.
- **Conda conflict** — If Conda is active, `export PATH="/usr/bin:$PATH"` before any ROS 2 command. Conda's Python 3.13 shadows the system Python 3.10 needed by colcon and ament.
- Always use `--symlink-install` for colcon to allow live Python edits.
- YOLOv8 model (`yolov8n.pt`) is provided in the workspace root. If missing, download at runtime with `ultralytics` (or ask user).

### External Dependencies (system packages)
- `ros-humble-desktop` (full desktop install)
- `ros-humble-gazebo-ros2-control` (ros2_control + Gazebo integration)
- `ros-humble-moveit2` (move_group, planners)
- `ros-humble-py-trees-ros` (behavior tree)
- `ros-humble-rosbridge-server` (dashboard WebSocket)
- `python3-opencv` (ArUco detection)
- `python3-ultralytics` or `pip install ultralytics` (YOLOv8)
- `ros-humble-ur` (ur_description, ur_controllers, ur_moveit_config) — from packages, not from source

### Things That Must Never Appear in the Codebase
- `moveit_commander` or `moveit_py` (do not exist in Humble)
- `time.sleep()` in any node
- `print()` for logging
- Hardcoded absolute paths
- Dockerfiles or docker-compose (no Docker in this project)
- Credentials, tokens, or secrets of any kind
- `# TODO:` comments — either do it or file it in PROGRESS.md

### Non-Goals
- Real robot hardware (never attempt to connect to a physical UR3e)
- Safety layer / human detection (dropped)
- Multi-arm coordination
- ROS 2 Humble → Iron/Jazzy migration
- CI/CD pipeline
- Formal verification or testing framework
- Containerization
