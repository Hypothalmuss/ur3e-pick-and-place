# AGENTS — Reusable Lessons for the AI

## ROS 2 Humble Notes

- `moveit_commander` and `moveit_py` do NOT exist in Humble. Use raw MoveIt2 action/service clients.
- Use `ament_cmake` for message/service packages (required by ROS 2).
- Use `ament_python` for all other packages to minimize CMake burden.

## Build Notes

- Always use `--symlink-install` for colcon to allow live Python edits.
- Python node files under `src/*/package/*/nodes/*.py` can be edited without rebuild.
- Drop `COLCON_IGNORE` in non-workspace subdirectories (e.g., skills/evals/) to prevent duplicate package errors.
- After `colcon build`, verify with `ros2 interface list | grep ur3e` for msgs, `install/*/bin/*` for executables.

## Gazebo Quirks

- Set `LIBGL_ALWAYS_SOFTWARE=1` before launching Gazebo to avoid GPU OpenGL crashes.
- Source `/usr/share/gazebo/setup.sh` before `ros2 launch` to avoid RTShader assertion.
- If Conda is active, run `export PATH="/usr/bin:$PATH"` before any ROS 2 command.

## Coding Conventions

- No `print()` — use `self.get_logger().info()`
- No `time.sleep()` — use `rclpy.timer`
- Every file: snake_case for files/functions, PascalCase for classes, UPPER_SNAKE for constants
- Type hints required on all function signatures
- Google-style docstrings on public functions only
