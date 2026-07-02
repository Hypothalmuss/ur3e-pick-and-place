import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class VacuumControllerNode(Node):
    def __init__(self):
        super().__init__('vacuum_controller_node')
        self._active = False
        self._pub = self.create_publisher(Bool, '/vacuum/active', 10)
        self._srv = self.create_service(SetBool, '/vacuum/control', self._cb)

    def _cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._active = request.data
        self._pub.publish(Bool(data=self._active))
        self.get_logger().info(f'Vacuum {"activated" if self._active else "deactivated"}')
        response.success = True
        response.message = f'Vacuum {"on" if self._active else "off"}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VacuumControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
