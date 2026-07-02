import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'
POSITION_CLOSED = 0.8
POSITION_OPEN = 0.05


class GripperStateNode(Node):
    def __init__(self):
        super().__init__('gripper_state_node')
        self._pos_pub = self.create_publisher(Float64, '/gripper/position', 10)
        self._state_pub = self.create_publisher(String, '/gripper/state', 10)
        self._sub = self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self._last_state = 'unknown'

    def _cb(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if name == GRIPPER_JOINT:
                pos = msg.position[i]
                self._pos_pub.publish(Float64(data=pos))

                if pos > POSITION_CLOSED - 0.05:
                    state = 'closed'
                elif pos < POSITION_OPEN + 0.05:
                    state = 'open'
                else:
                    state = 'moving'

                if state != self._last_state:
                    self._state_pub.publish(String(data=state))
                    self._last_state = state
                break


def main(args=None):
    rclpy.init(args=args)
    node = GripperStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
