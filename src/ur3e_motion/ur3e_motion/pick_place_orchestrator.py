import math
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
    PositionIKRequest,
)
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import JointState
from ur3e_msgs.msg import DetectedObjectArray


GRIPPER_OPEN = 0.0
# Close angle that brings the finger-tip gap to ~60 mm so the pads contact the
# 0.06 m cube (0.8 rad is fully closed). The grasp-fix plugin then locks the
# hold once the pads make stable contact.
GRIPPER_CLOSED = 0.70

# tool0 height for the pre-grasp pose. The gripper reaches ~0.16 m below tool0,
# so 0.30 keeps the finger tips ~0.14 m up, clearing the 0.06 m cube before the
# straight vertical descent to the grasp (avoids clipping/knocking it).
APPROACH_HEIGHT = 0.30
LIFT_HEIGHT = 0.32       # tool0 height for retract/lift after grasp
# Place target in the reachable front-left of the workspace (the pick is at
# ~0.4, 0). Kept at a similar radius to the pick so the transfer is a clean arc.
PLACE_X = 0.30
PLACE_Y = 0.30
PLACE_Z = 0.0
# tool0 z at grasp/place-down. The Robotiq 2F-85 reaches ~0.16 m below tool0,
# so tool0 at 0.18 puts the finger tips around the 0.06 m cube (perception
# reports the cube on the ground plane at z=0).
GRASP_Z_OFFSET = 0.18

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
HOME_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
RETRACT_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

TOOL0_DOWN_ORI = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

REACH_MAX = 0.55  # conservative UR3e working radius, accounting for gripper offset

# Safe velocity limits for direct joint control (rad/s)
SAFE_VELOCITY = 0.4  # conservative max joint velocity for simulation


class State(Enum):
    STARTUP = auto()
    WAITING = auto()
    HOME = auto()
    APPROACH = auto()
    GRASP = auto()
    GRASP_CLOSE = auto()
    RETRACT = auto()
    PLACE = auto()
    PLACE_DOWN = auto()
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
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=cbg)

        self._perception_sub = self.create_subscription(
            DetectedObjectArray, '/detected_objects', self._perception_cb, 10)

        self._state = State.STARTUP
        self._cube_pose: Pose | None = None
        self._last_perception_log_time = 0.0
        self._timer = self.create_timer(0.5, self._tick, callback_group=cbg)

        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self._current_joint_positions: dict[str, float] = {}
        self._joint_positions_known = False

        self._cx = 0.0
        self._cy = 0.0
        self._cz = 0.0
        self._cycle_active = False
        self._recovering = False

    def _set_state(self, state: State) -> None:
        self._state = state
        self.get_logger().info(f'State -> {state.name}')

    def _joint_state_cb(self, msg: JointState) -> None:
        self._current_joint_positions = dict(zip(msg.name, msg.position))
        self._joint_positions_known = True

    def _compute_safe_duration(self, target_joints: list[float]) -> float:
        if not self._joint_positions_known:
            return 3.0
        max_disp = 0.0
        for i, name in enumerate(ARM_JOINTS):
            current = self._current_joint_positions.get(name, 0.0)
            disp = abs(target_joints[i] - current)
            if disp > max_disp:
                max_disp = disp
        return max(max_disp / SAFE_VELOCITY, 1.0)

    def _perception_cb(self, msg: DetectedObjectArray) -> None:
        if msg.objects and self._state == State.WAITING:
            self._cube_pose = msg.objects[0].pose
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_perception_log_time >= 2.0:
                self.get_logger().info(
                    f'Cube at ({self._cube_pose.position.x:.3f}, '
                    f'{self._cube_pose.position.y:.3f}, '
                    f'{self._cube_pose.position.z:.3f})')
                self._last_perception_log_time = now

    def _tick(self) -> None:
        if self._state == State.STARTUP:
            self._set_state(State.HOME)
            self._move_joints(HOME_JOINTS, self._startup_done)
            return

        if self._state == State.ERROR:
            if not self._recovering:
                self._recovering = True
                self.get_logger().warn(
                    'Cycle failed; recovering (open gripper, return HOME)')
                self._gripper(GRIPPER_OPEN, self._recover_home)
            return

        if self._state != State.WAITING:
            return

        if self._cycle_active:
            return

        cube = self._cube_pose
        if cube is None:
            return

        radius = math.hypot(cube.position.x, cube.position.y)
        if radius > REACH_MAX:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_perception_log_time >= 5.0:
                self.get_logger().warn(
                    f'Cube at radius {radius:.3f}m exceeds reach limit '
                    f'{REACH_MAX}m, ignoring detection')
                self._last_perception_log_time = now
            return

        self.get_logger().info('Cube detected, proceeding to approach')
        self._cycle_active = True
        self._approach(True)

    def _startup_done(self, success: bool) -> None:
        if not success:
            self.get_logger().warn('Startup HOME move failed, retrying')
            self._set_state(State.STARTUP)
            return
        self._set_state(State.WAITING)
        self.get_logger().info('Startup complete, waiting for cube')

    def _approach(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        cube = self._cube_pose
        if cube is None:
            self._set_state(State.ERROR)
            return

        self._cx = cube.position.x
        self._cy = cube.position.y
        self._cz = cube.position.z

        approach = Pose(
            position=Point(
                x=self._cx,
                y=self._cy,
                z=self._cz + APPROACH_HEIGHT,
            ),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.APPROACH)
        self._move_pose(approach, self._grasp)

    def _grasp(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        grasp = Pose(
            position=Point(
                x=self._cx,
                y=self._cy,
                z=self._cz + GRASP_Z_OFFSET,
            ),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.GRASP)
        self._move_pose(grasp, self._grasp_close)

    def _grasp_close(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        self._set_state(State.GRASP_CLOSE)
        self._gripper(GRIPPER_CLOSED, self._retract)

    def _retract(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        lift = Pose(
            position=Point(x=self._cx, y=self._cy, z=self._cz + LIFT_HEIGHT),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.RETRACT)
        self._move_pose(lift, self._place)

    def _place(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        place = Pose(
            position=Point(x=PLACE_X, y=PLACE_Y, z=PLACE_Z + LIFT_HEIGHT),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.PLACE)
        self._move_pose(place, self._place_down)

    def _place_down(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        place = Pose(
            position=Point(x=PLACE_X, y=PLACE_Y, z=PLACE_Z + GRASP_Z_OFFSET),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.PLACE_DOWN)
        self._move_pose(place, self._release)

    def _release(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        self._set_state(State.RELEASE)
        self._gripper(GRIPPER_OPEN, self._finish)

    def _finish(self, success: bool) -> None:
        self._set_state(State.DONE)
        self.get_logger().info('Pick-and-place complete')
        self._move_joints(RETRACT_JOINTS, self._reset, duration_sec=None)

    def _reset(self, success: bool = True) -> None:
        self._cycle_active = False
        self._set_state(State.WAITING)
        self._cube_pose = None
        self.get_logger().info('Ready for next pick-and-place')

    def _recover_home(self, success: bool = True) -> None:
        # Gripper reopened; return to HOME before retrying so the next detection
        # starts from a clean, unoccluded configuration.
        self._move_joints(HOME_JOINTS, self._recover_done)

    def _recover_done(self, success: bool = True) -> None:
        self._recovering = False
        self._cycle_active = False
        self._cube_pose = None
        self._set_state(State.WAITING)
        self.get_logger().info('Recovered; waiting for cube')

    def _move_joints(self, joints: list[float],
                     callback, duration_sec: float | None = None) -> None:
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
                         duration_sec: float | None, callback) -> None:
        from control_msgs.action import FollowJointTrajectory
        client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Joint trajectory controller unavailable')
            callback(False)
            return

        if duration_sec is None:
            duration_sec = self._compute_safe_duration(joints)
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
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.goal_constraints = [
            self._make_pose_constraints(target_pose)]

        self._send_future = self._moveit_client.send_goal_async(goal)
        self._send_future.add_done_callback(
            lambda f: self._goal_response_cb(f, callback, target_pose))

    def _goal_response_cb(self, future, callback,
                          fallback_pose: Pose | None = None) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('MoveIt2 goal rejected')
            if fallback_pose is not None:
                self._pose_ik_fallback(fallback_pose, callback)
                return
            callback(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_cb(f, callback, fallback_pose))

    def _result_cb(self, future, callback,
                   fallback_pose: Pose | None = None) -> None:
        result = future.result().result
        ok = result.error_code.val == 1
        if not ok:
            self.get_logger().error(
                f'MoveIt2 failed: error_code={result.error_code.val}')
            if fallback_pose is not None:
                self.get_logger().warn(
                    'Cartesian pose planning failed, retrying via IK + '
                    'joint-space planning')
                self._pose_ik_fallback(fallback_pose, callback)
                return
        callback(ok)

    def _pose_ik_fallback(self, target_pose: Pose, callback,
                          avoid_collisions: bool = True) -> None:
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'compute_ik service unavailable, cannot fall back')
            callback(False)
            return

        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.timeout = Duration(sec=1)
        req.ik_request.avoid_collisions = avoid_collisions

        future = self._ik_client.call_async(req)
        future.add_done_callback(
            lambda f: self._ik_response_cb(f, callback, target_pose, avoid_collisions))

    def _ik_response_cb(self, future, callback,
                        target_pose: Pose | None = None,
                        tried_avoid: bool = True) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'compute_ik call failed: {e}')
            callback(False)
            return

        if response.error_code.val != 1:
            self.get_logger().error(
                f'compute_ik failed: error_code={response.error_code.val}')
            if tried_avoid and target_pose is not None:
                self.get_logger().warn(
                    'IK with collision avoidance failed, retrying without')
                self._pose_ik_fallback(target_pose, callback, avoid_collisions=False)
                return
            callback(False)
            return

        name_to_pos = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position))
        try:
            joints = [name_to_pos[name] for name in ARM_JOINTS]
        except KeyError as e:
            self.get_logger().error(f'IK solution missing joint {e}')
            callback(False)
            return

        self.get_logger().info(
            'IK fallback succeeded, sending joints directly to controller')
        self._send_joint_goal(joints, None, callback)

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
        pc.target_point_offset = Vector3()
        volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        # Tight tolerance so the gripper centres on the cube instead of landing
        # up to 5 cm off (which clips/knocks the 6 cm cube during descent).
        sphere.dimensions = [0.015]
        volume.primitives.append(sphere)
        volume.primitive_poses.append(Pose(position=pose.position))
        pc.constraint_region = volume
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.08
        oc.absolute_y_axis_tolerance = 0.08
        oc.absolute_z_axis_tolerance = 0.3
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
