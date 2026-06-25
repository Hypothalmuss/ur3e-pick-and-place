import rclpy
from rclpy.node import Node


class GripperStateNode(Node):
    def __init__(self):
        super().__init__('gripper_state_node')


def main(args=None):
    rclpy.init(args=args)
    node = GripperStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
