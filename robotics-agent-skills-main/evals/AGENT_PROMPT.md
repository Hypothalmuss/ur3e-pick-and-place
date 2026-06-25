# Agent Prompt — With Skills

This is the exact prompt given to the Claude Code sub-agent for the **with-skills** eval run.
This prompt includes **no explicit design instructions** — only raw skill summaries as
context and the same task prompt from PROMPT.md.

---

You have the following reference skills loaded as context. Use them however you see fit.

--- SKILL FILE: skills/ros2/SKILL.md ---
(Full ROS2 development skill covering: node design patterns with rclpy/rclcpp, lifecycle managed nodes with on_configure/on_activate/on_deactivate/on_cleanup/on_shutdown, QoS profiles and compatibility matrix — BEST_EFFORT for sensors depth=1, RELIABLE for commands depth=10, TRANSIENT_LOCAL for maps — QoS debugging, Python launch files with arguments/conditions/composable nodes/lifecycle orchestration, components for zero-copy, actions with goal/cancel/feedback, DDS configuration with CycloneDDS tuning, build system with colcon flags and ament_cmake vs ament_python, package.xml dependency declarations, CMakeLists.txt template with rosidl_generate_interfaces for custom msgs/srvs/actions, setup.py/setup.cfg for Python packages, custom message/service/action definitions, workspace overlays, build troubleshooting, package structure conventions, debugging toolkit commands, production deployment checklist.)

--- SKILL FILE: skills/robot-perception/SKILL.md ---
(Full robot perception skill covering: sensor landscape table, hardware/SDK/driver mapping, pinhole camera model, intrinsic calibration with IntrinsicCalibrator class, extrinsic calibration, hand-eye calibration, calibration quality checklist, CameraStream class with threaded bounded buffer and diagnostics, SyncedMultiSensor for software timestamp synchronization, HardwareSyncConfig for RealSense inter-cam sync and PTP, streaming anti-patterns — blocking capture, unbounded buffers, wrong timestamps — ImageUndistorter, RobotObjectDetector with 3D backprojection, FiducialDetector for AprilTag/ArUco, WorkspaceSegmenter, DepthProcessor with range/flying pixel/hole filling/bilateral filter, DepthToPointCloud vectorized backprojection, RGBDAligner, PointCloudProcessor with Open3D crop/outlier/voxel/normals/RANSAC/DBSCAN, PointCloudRegistration with ICP and global FPFH+RANSAC, CameraLiDARFusion, PerceptionTracker with Hungarian assignment, latency budget table, 6 perception anti-patterns, production checklist.)

--- SKILL FILE: skills/robotics-design-patterns/SKILL.md ---
(Full robotics design patterns skill covering: robot software stack layers, behavior trees with py_trees implementation and blackboard pattern, FSMs with smach, perception pipeline with sensor fusion architecture and timing rules, hardware abstraction layer with factory pattern, safety systems with watchdog/heartbeat/workspace limits, sim-to-real architecture with HAL pattern, data recording architecture with EpisodeRecorder class supporting streams/metadata/quality assessment, 8 anti-patterns — god node, magic numbers, polling, no error recovery, sim-only code, no timestamps, blocking control loop, no data logging — architecture decision checklist.)

--- SKILL FILE: skills/robotics-software-principles/SKILL.md ---
(Full robotics software principles covering 12 principles: single responsibility, dependency inversion, open-closed, interface segregation, Liskov substitution, separation of rates, fail-safe defaults, configuration over code, idempotent operations, observe everything, composability, graceful degradation. Each with robotics-specific rationale and code examples.)

--- SKILL FILE: skills/robotics-testing/SKILL.md ---
(Full robotics testing skill covering: testing pyramid for robotics, pytest fixtures for ROS2 nodes with ros_context/perception_node/test_image fixtures, pure function testing with parametrize, property-based testing with hypothesis, launch_testing for integration tests, mock hardware patterns — MockCamera and MockJointStatePublisher, golden file testing for trajectory regression, simulation testing with MuJoCo harness, CI/CD pipeline YAML for robotics, testing anti-patterns — sleep-based tests, no failure cases, non-deterministic tests.)

--- YOUR TASK ---

Create all files in: /home/agog13/mini_projects/robotics-agent-skills/evals/with-skills-fair/

Build a ROS2 Python package called `demo_recorder` for recording robot demonstration episodes during teleoperation.

Requirements:
- Subscribe to camera images from a wrist-mounted RealSense and joint states from a 7-DOF Franka arm
- Record episodes as structured data (observations + actions per timestep)
- Support starting/stopping recording via a ROS2 service call
- Save episodes to disk with metadata (task name, operator, timestamp, success/failure)
- Publish recording status so a UI can show whether recording is active

Include the full package: node implementation, message/service definitions, launch file, config, and a README.

Start by creating directories, then write all files.
