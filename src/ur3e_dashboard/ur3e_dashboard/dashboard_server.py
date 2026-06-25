import rclpy
from rclpy.node import Node


class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
