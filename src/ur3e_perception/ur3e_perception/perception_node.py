import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener

from ur3e_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Pose, Point, Quaternion


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
        self._table_z = 0.0

        # camera_optical_frame (X=right, Y=down, Z=forward) -> camera_link (X=forward, Y=left, Z=up)
        #   X_body =  Z_opt,  Y_body = -X_opt,  Z_body = -Y_opt
        self._R_opt_to_body = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ])

    def _info_cb(self, msg: CameraInfo):
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        if not self._info_reported:
            self.get_logger().info(f'Camera info: fx={self._fx:.1f}, '
                                   f'cx={self._cx:.1f}, cy={self._cy:.1f}')
            self._info_reported = True

    def _get_camera_pose(self):
        try:
            trans = self._tf_buffer.lookup_transform(
                'base_link', 'camera_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

        t = trans.transform.translation
        pos = np.array([t.x, t.y, t.z])
        q = trans.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])
        return pos, R

    def _image_cb(self, msg: Image):
        if not self._info_reported:
            return

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

        now = self.get_clock().now()
        if not hasattr(self, '_last_log') or (now - self._last_log).nanoseconds > 2e9:
            self._last_log = now
            white = cv2.countNonZero(mask)
            self.get_logger().info(
                f'Mask pixels: {white}/{mask.shape[0]*mask.shape[1]}, contours: {len(contours)}')

        cam = self._get_camera_pose()
        if cam is None:
            self._pub.publish(detected)
            return
        cam_pos, R_link_to_base = cam

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 10:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            uc = M['m10'] / M['m00']
            vc = M['m01'] / M['m00']

            ray_opt = np.array([
                (uc - self._cx) / self._fx,
                (vc - self._cy) / self._fy,
                1.0,
            ])

            ray_body = self._R_opt_to_body @ ray_opt
            ray_base = R_link_to_base @ ray_body

            if abs(ray_base[2]) < 1e-6:
                continue
            t = (self._table_z - cam_pos[2]) / ray_base[2]
            world = cam_pos + t * ray_base

            obj = DetectedObject()
            obj.aruco_id = 0
            obj.class_name = 'cube'
            obj.confidence = 1.0
            obj.pose = Pose(
                position=Point(x=world[0], y=world[1], z=world[2]),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            detected.objects.append(obj)

        self._pub.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
