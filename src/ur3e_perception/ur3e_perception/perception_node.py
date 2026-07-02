import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from ur3e_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Pose, Point, Quaternion


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None

        self._info_reported = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._img_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos)
        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self._info_cb, qos)

        self._pub = self.create_publisher(DetectedObjectArray, '/detected_objects', 10)

        self._cx = 320.0
        self._cy = 240.0
        self._fx = 525.0
        self._fy = 525.0

        self._camera_x = 0.15
        self._camera_y = 0.0
        self._camera_z = 3.0
        self._table_z = 0.0

    def _info_cb(self, msg: CameraInfo):
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        if not self._info_reported:
            self.get_logger().info(f'Camera info: fx={self._fx:.1f}, '
                                   f'cx={self._cx:.1f}, cy={self._cy:.1f}')
            self._info_reported = True

    def _image_cb(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        lower0 = np.array([0, 40, 40])
        upper0 = np.array([30, 255, 255])
        mask0 = cv2.inRange(hsv, lower0, upper0)

        lower1 = np.array([160, 40, 40])
        upper1 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        mask = cv2.bitwise_or(mask0, mask1)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = DetectedObjectArray()
        detected.header.stamp = msg.header.stamp
        detected.header.frame_id = 'base_link'

        white = cv2.countNonZero(mask)
        self.get_logger().info(f'Mask pixels: {white}/{mask.shape[0]*mask.shape[1]}, contours: {len(contours)}')

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 10:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            uc = M['m10'] / M['m00']
            vc = M['m01'] / M['m00']

            z_cam = self._camera_z - self._table_z
            x_cam = (uc - self._cx) * z_cam / self._fx
            y_cam = (vc - self._cy) * z_cam / self._fy

            obj = DetectedObject()
            obj.aruco_id = 0
            obj.class_name = 'cube'
            obj.confidence = 1.0
            obj.pose = Pose(
                position=Point(
                    x=self._camera_x + x_cam,
                    y=self._camera_y - y_cam,
                    z=self._table_z,
                ),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            detected.objects.append(obj)

            self.get_logger().info(
                f'Detected cube at ({obj.pose.position.x:.3f}, '
                f'{obj.pose.position.y:.3f}, {obj.pose.position.z:.3f})')

        self._pub.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
