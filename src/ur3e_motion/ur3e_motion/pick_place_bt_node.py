import rclpy

from ur3e_motion.motion_executor import MotionExecutor
from ur3e_motion.pick_place_bt import PickPlaceBT


class PickPlaceBTNode(MotionExecutor):
    def __init__(self):
        super().__init__('pick_place_bt_node')
        self._bt = PickPlaceBT()

    def run_sequence(self):
        while True:
            action = self._bt.tick()
            if action == 'done':
                self.get_logger().info('Pick-and-place complete')
                break
            if action is None:
                break

            self.get_logger().info(f'State: {self._bt.state.name} -> {action}')

            if action == 'move':
                joints = self._bt.target_joints()
                if joints and not self.move_to_joints(joints):
                    self.get_logger().error('Move failed')
                    break
            elif action == 'open_gripper':
                if not self.gripper_command(0.0):
                    self.get_logger().error('Open gripper failed')
                    break
            elif action == 'close_gripper':
                if not self.gripper_command(0.5):
                    self.get_logger().error('Close gripper failed')
                    break

            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Sequence finished')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceBTNode()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()
