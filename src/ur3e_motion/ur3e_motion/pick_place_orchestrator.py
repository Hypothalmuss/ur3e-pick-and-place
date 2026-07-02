from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from ur3e_msgs.msg import DetectedObjectArray


GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.7929

APPROACH_HEIGHT = 0.12
PLACE_X = 0.3
PLACE_Y = 0.3
PLACE_Z = 0.0

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
HOME_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
RETRACT_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

TOOL0_DOWN_ORI = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


class State(Enum):
    WAITING = auto()
    HOME = auto()
    APPROACH = auto()
    GRASP = auto()
    GRASP_CLOSE = auto()
    RETRACT = auto()
    PLACE = auto()
    RELEASE = auto()
    DONE = auto()
    ERROR = auto()


class PickPlaceOrchestrator(Node):
    def __init__(self):
        super().__init__('pick_place_orchestrator')

        cbg = MutuallyExclusiveCallbackGroup()

        self._moveit_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=cbg)
        self._gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            callback_group=cbg)

        self._perception_sub = self.create_subscription(
            DetectedObjectArray, '/detected_objects', self._perception_cb, 10)

        self._state = State.WAITING
        self._cube_pose: Pose | None = None
        self._timer = self.create_timer(0.5, self._tick)

    def _perception_cb(self, msg: DetectedObjectArray) -> None:
        if msg.objects and self._state == State.WAITING:
            self._cube_pose = msg.objects[0].pose
            self.get_logger().info(
                f'Cube at ({self._cube_pose.position.x:.3f}, '
                f'{self._cube_pose.position.y:.3f}, '
                f'{self._cube_pose.position.z:.3f})')

    def _tick(self) -> None:
        if self._state != State.WAITING:
            return

        cube = self._cube_pose
        if cube is None:
            return

        self._state = State.HOME
        self._move_joints(HOME_JOINTS, self._approach)

    def _approach(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        cube = self._cube_pose
        if cube is None:
            self._state = State.ERROR
            return

        approach = Pose(
            position=Point(
                x=cube.position.x,
                y=cube.position.y,
                z=cube.position.z + APPROACH_HEIGHT,
            ),
            orientation=TOOL0_DOWN_ORI,
        )
        self._state = State.APPROACH
        self._move_pose(approach, self._grasp)

    def _grasp(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        cube = self._cube_pose
        if cube is None:
            self._state = State.ERROR
            return

        grasp = Pose(
            position=Point(
                x=cube.position.x,
                y=cube.position.y,
                z=cube.position.z,
            ),
            orientation=TOOL0_DOWN_ORI,
        )
        self._state = State.GRASP
        self._move_pose(grasp, self._grasp_close)

    def _grasp_close(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        self._state = State.GRASP_CLOSE
        self._gripper(GRIPPER_CLOSED, self._retract)

    def _retract(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        self._state = State.RETRACT
        self._move_joints(RETRACT_JOINTS, self._place)

    def _place(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        place = Pose(
            position=Point(x=PLACE_X, y=PLACE_Y, z=PLACE_Z + APPROACH_HEIGHT),
            orientation=TOOL0_DOWN_ORI,
        )
        self._state = State.PLACE
        self._move_pose(place, self._place_down)

    def _place_down(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        place = Pose(
            position=Point(x=PLACE_X, y=PLACE_Y, z=PLACE_Z),
            orientation=TOOL0_DOWN_ORI,
        )
        self._move_pose(place, self._release)

    def _release(self, success: bool) -> None:
        if not success:
            self._state = State.ERROR
            return

        self._state = State.RELEASE
        self._gripper(GRIPPER_OPEN, self._finish)

    def _finish(self, success: bool) -> None:
        self._state = State.DONE
        self.get_logger().info('Pick-and-place complete')
        self._move_joints(RETRACT_JOINTS, self._reset)

    def _reset(self, success: bool = True) -> None:
        self._state = State.WAITING
        self._cube_pose = None
        self.get_logger().info('Ready for next pick-and-place')

    def _move_joints(self, joints: list[float],
                     callback, duration_sec: float = 2.0) -> None:
        if not self._moveit_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('MoveIt2 not available, using direct joint control')
            self._send_joint_goal(joints, duration_sec, callback)
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 3.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.planner_id = 'RRTConnectkConfigDefault'

        goal.request.goal_constraints = [
            self._make_joint_constraints(joints)]

        self._send_future = self._moveit_client.send_goal_async(goal)
        self._send_future.add_done_callback(
            lambda f: self._goal_response_cb(f, callback))

    def _send_joint_goal(self, joints: list[float],
                         duration_sec: float, callback) -> None:
        from control_msgs.action import FollowJointTrajectory
        client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Joint trajectory controller unavailable')
            callback(False)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=joints,
            time_from_start=Duration(
                sec=int(duration_sec),
                nanosec=int((duration_sec % 1) * 1e9)),
        ))
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._joint_goal_response_cb(f, client, callback))

    def _joint_goal_response_cb(self, future, client, callback) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Joint goal rejected')
            callback(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: callback(f.result().result.error_code == 0))

    def _move_pose(self, target_pose: Pose, callback) -> None:
        if not self._moveit_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveIt2 not available for pose motion')
            callback(False)
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.goal_constraints = [
            self._make_pose_constraints(target_pose)]

        self._send_future = self._moveit_client.send_goal_async(goal)
        self._send_future.add_done_callback(
            lambda f: self._goal_response_cb(f, callback))

    def _goal_response_cb(self, future, callback) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('MoveIt2 goal rejected')
            callback(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_cb(f, callback))

    def _result_cb(self, future, callback) -> None:
        result = future.result().result
        ok = result.error_code.val == 1
        if not ok:
            self.get_logger().error(
                f'MoveIt2 failed: error_code={result.error_code.val}')
        callback(ok)

    def _make_joint_constraints(self, joints: list[float]) -> Constraints:
        from moveit_msgs.msg import JointConstraint
        c = Constraints()
        for i, name in enumerate(ARM_JOINTS):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joints[i]
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _make_pose_constraints(self, pose: Pose) -> Constraints:
        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        pc.target_point_offset = Point()
        volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]
        volume.primitives.append(sphere)
        volume.primitive_poses.append(Pose(position=pose.position))
        pc.constraint_region = volume
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        return c

    def _gripper(self, position: float, callback) -> None:
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper controller not available')
            callback(False)
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0

        send_future = self._gripper_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._gripper_response_cb(f, callback))

    def _gripper_response_cb(self, future, callback) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            callback(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: callback(f.result().result.reached_goal))


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
