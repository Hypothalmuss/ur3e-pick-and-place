#!/usr/bin/env bash
# Cleanly stop the UR3e sim and clear DDS shared memory so the next launch is
# clean. Run this whenever things get weird.
for p in bringup.launch move_group.launch motion.launch gzserver gzclient \
         robot_state_publisher controller_manager scene_initializer \
         motion_executor pick_place_orchestrator static_transform \
         perception_node gripper_state dashboard_server \
         "ros2 launch" moveit_ros_move_group; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 2
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* 2>/dev/null
echo "[stop_sim] stopped and cleaned."
