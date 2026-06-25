Build a ROS2 Python package called `demo_recorder` for recording robot demonstration
episodes during teleoperation.

Requirements:
- Subscribe to camera images from a wrist-mounted RealSense and joint states from a
  7-DOF Franka arm
- Record episodes as structured data (observations + actions per timestep)
- Support starting/stopping recording via a ROS2 service call
- Save episodes to disk with metadata (task name, operator, timestamp, success/failure)
- Publish recording status so a UI can show whether recording is active

Include the full package: node implementation, message/service definitions,
launch file, config, and a README.
