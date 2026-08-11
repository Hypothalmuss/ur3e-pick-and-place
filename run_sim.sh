#!/usr/bin/env bash
# Reliable launcher for the UR3e pick-and-place sim.
# Cleans up any leftovers first (the #1 cause of "robot won't spawn" / "sim
# breaks" is orphaned processes + stale DDS shared memory), then launches the
# three stages in order.
#
# Usage:  ./run_sim.sh          # headless gzserver (most stable)
#         ./run_sim.sh gui      # also open the Gazebo GUI window
# NOTE: no 'set -u' - ROS setup.bash references unbound vars and would abort.
WS=/home/eagletn3/Downloads/new_ur3e
GUI="${1:-headless}"

echo "[run_sim] cleaning up any previous run..."
for p in bringup.launch move_group.launch motion.launch gzserver gzclient \
         robot_state_publisher controller_manager scene_initializer \
         motion_executor pick_place_orchestrator static_transform \
         perception_node gripper_state "ros2 launch" moveit_ros_move_group; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 3
ros2 daemon stop >/dev/null 2>&1
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* 2>/dev/null
sleep 1

# Force system Python (3.10) ahead of any conda env - ROS Humble's rclpy C
# extension is built for 3.10, and conda's python (3.13) makes spawn_entity.py
# crash with "No module named 'rclpy._rclpy_pybind11'".
conda deactivate 2>/dev/null || true
export PATH="/usr/bin:$PATH"
unset PYTHONHOME PYTHONPATH 2>/dev/null

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null
source "$WS/install/setup.bash"
export GAZEBO_PLUGIN_PATH="$WS/install/ur3e_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"
cd "$WS"
echo "[run_sim] using python: $(which python3)"

GZGUI=false; [ "$GUI" = "gui" ] && GZGUI=true

echo "[run_sim] launching Gazebo + robot (gui=$GZGUI)..."
setsid nohup ros2 launch ur3e_sim_bringup bringup.launch.py gazebo_gui:=$GZGUI \
  > /tmp/ur3e_bringup.log 2>&1 </dev/null & disown
for i in $(seq 1 45); do grep -q "Configured and activated .*gripper_controller" /tmp/ur3e_bringup.log 2>/dev/null && break; sleep 2; done
grep -q "Successfully spawned" /tmp/ur3e_bringup.log 2>/dev/null \
  && echo "[run_sim]   robot spawned, controllers active" \
  || { echo "[run_sim]   ERROR: robot did not spawn - see /tmp/ur3e_bringup.log"; exit 1; }

echo "[run_sim] launching MoveIt2..."
setsid nohup ros2 launch ur3e_sim_bringup move_group.launch.py launch_rviz:=false \
  > /tmp/ur3e_movegroup.log 2>&1 </dev/null & disown
for i in $(seq 1 45); do grep -q "You can start planning now" /tmp/ur3e_movegroup.log 2>/dev/null && break; sleep 2; done
echo "[run_sim]   move_group ready"

echo "[run_sim] launching orchestrator..."
# The orchestrator idles until told what to do (auto:=true restores the old
# self-looping behaviour). Drive it from the dashboard below.
setsid nohup ros2 launch ur3e_motion motion.launch.py \
  > /tmp/ur3e_motion.log 2>&1 </dev/null & disown

echo "[run_sim] launching dashboard..."
setsid nohup ros2 launch ur3e_dashboard dashboard.launch.py \
  > /tmp/ur3e_dash.log 2>&1 </dev/null & disown
for i in $(seq 1 20); do
  curl -s --max-time 1 http://127.0.0.1:8080/api/state >/dev/null 2>&1 && break
  sleep 1
done

echo "[run_sim] DONE. Cell is up and idle."
echo "  DASHBOARD    : http://127.0.0.1:8080"
echo "  watch states : grep 'State ->' /tmp/ur3e_motion.log"
echo "  cube position: ros2 topic echo /detected_objects --once"
echo "  stop         : $WS/stop_sim.sh"
echo "  NOTE: do NOT run 'gz model'/'gz topic' in a loop - it can crash gzserver."
