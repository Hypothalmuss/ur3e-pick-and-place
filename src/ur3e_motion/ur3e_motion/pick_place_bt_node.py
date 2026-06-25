import rclpy
from rclpy.node import Node


class PickPlaceBTNode(Node):
    def __init__(self):
        super().__init__('pick_place_bt_node')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceBTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
