import rclpy
from rclpy.node import Node


class MotionExecutor(Node):
    def __init__(self):
        super().__init__('motion_executor')


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
