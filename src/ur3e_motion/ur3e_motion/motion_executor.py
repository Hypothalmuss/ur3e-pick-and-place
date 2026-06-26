import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

HOME_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
APPROACH_JOINTS = [0.0, -1.2, 0.5, -1.8, 0.0, 0.0]
RETRACT_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]


class MotionExecutor(Node):
    def __init__(self, node_name: str = 'motion_executor'):
        super().__init__(node_name)
        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, GripperCommand,
            '/gripper_controller/gripper_cmd')

    def move_to_joints(self, joint_positions: list[float],
                       duration_sec: float = 2.0) -> bool:
        if not self._traj_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Joint trajectory controller not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration_sec),
                                         nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points = [point]

        self.get_logger().info(f'Moving to {joint_positions}...')
        future = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Joint trajectory goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.error_code == 0
        self.get_logger().info('Move OK' if success else 'Move failed')
        return success

    def gripper_command(self, position: float, max_effort: float = 10.0) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Gripper controller not available')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        self.get_logger().info(f'Gripper to {position}...')
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.reached_goal
        self.get_logger().info('Gripper OK' if success else 'Gripper failed')
        return success


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutor()
    node.get_logger().info('Motion executor ready')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
