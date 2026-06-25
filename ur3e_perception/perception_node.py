import rclpy
from rclpy.node import Node


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
