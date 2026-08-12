import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
from tf2_ros import Buffer, TransformListener

from ur3e_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Pose, Point, Quaternion

# At 1600x1200 a 60 mm cube seen from 3 m covers roughly 690 px^2, so this
# rejects speckle with a wide margin under the real blob size.
MIN_CONTOUR_AREA = 100
# Frame-to-frame association distance. Cubes sit >100 mm apart and only move
# when carried, so this is generous without risking a mix-up.
TRACK_MATCH_RADIUS = 0.09  # m
# How many consecutive frames a track may go unseen before its id is
# retired. The camera runs at 30 Hz, so this is about half a second.
TRACK_MAX_MISSES = 15


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._info_reported = False
        self._tracks: dict[int, tuple[float, float]] = {}
        self._misses: dict[int, int] = {}
        self._next_id = 0

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
        # The camera looks down on the object's TOP face, so intersecting that
        # sightline with the table plane lands the estimate short of the object
        # by the parallax over its own height - about 11 mm for the cube here,
        # which is enough for a finger pad to catch it off-centre and shove it
        # away during the close. Intersect at the top face instead, which gives
        # the correct x/y of the object's centre. The reported z stays on the
        # table so the orchestrator's grasp offsets are unchanged.
        self._object_height = self.declare_parameter(
            'object_height', 0.06).get_parameter_value().double_value

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

        # Orange only. This used to be H 0-30 plus 160-180, i.e. everything from
        # red through orange, which would pick up the red drop-zone marker on the
        # ground as if it were a cube. The cubes are RGB(255,153,0) -> hue 36 deg
        # -> OpenCV H=18, so a band around that is both tighter and safer.
        mask = cv2.inRange(hsv, np.array([10, 80, 60]), np.array([25, 255, 255]))

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

        found = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
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
            t = (self._table_z + self._object_height - cam_pos[2]) / ray_base[2]
            world = cam_pos + t * ray_base

            found.append((float(world[0]), float(world[1])))

        for obj_id, (x, y) in self._assign_ids(found):
            obj = DetectedObject()
            # aruco_id carries the tracker's id - there are no markers in this
            # scene, and the field is the only integer the message has.
            obj.aruco_id = obj_id
            obj.class_name = 'cube'
            obj.confidence = 1.0
            obj.pose = Pose(
                position=Point(x=x, y=y, z=self._table_z),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            detected.objects.append(obj)

        self._pub.publish(detected)

    def _assign_ids(self, found: list) -> list:
        """Give each detection a id that survives across frames.

        Greedy nearest-neighbour against the previous frame. The cubes are far
        apart relative to how far they move between frames, so anything more
        elaborate would be wasted - but ids do have to be stable, because the
        orchestrator picks 'the cube with id N' and must not have it renamed
        underneath it mid-cycle.
        """
        unmatched = list(range(len(found)))
        out = {}

        for obj_id, (px, py) in sorted(self._tracks.items()):
            best, best_d = None, TRACK_MATCH_RADIUS
            for i in unmatched:
                d = math.hypot(found[i][0] - px, found[i][1] - py)
                if d < best_d:
                    best, best_d = i, d
            if best is not None:
                unmatched.remove(best)
                out[obj_id] = found[best]
                self._misses.pop(obj_id, None)
            else:
                # Not seen this frame. Dropping the track immediately would mint
                # a fresh id the moment it reappears, and the arm occludes a cube
                # constantly while working over it - ids churned so badly that
                # the orchestrator's target was renamed mid-sweep. Hold the last
                # known position for a few frames instead.
                self._misses[obj_id] = self._misses.get(obj_id, 0) + 1
                if self._misses[obj_id] <= TRACK_MAX_MISSES:
                    out[obj_id] = (px, py)
                else:
                    self._misses.pop(obj_id, None)

        for i in unmatched:
            self._next_id += 1
            out[self._next_id] = found[i]

        self._tracks = out
        # Only report tracks actually seen recently; a coasting track is kept
        # for id stability but should not be published as a live detection.
        return sorted((i, p) for i, p in out.items() if not self._misses.get(i))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
