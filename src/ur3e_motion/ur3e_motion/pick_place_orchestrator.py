import json
import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PositionIKRequest,
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur3e_msgs.msg import DetectedObjectArray
from ur3e_msgs.srv import RunTask


# --- Robotiq 2F-85 stroke, measured ------------------------------------------
# Gap between the pad faces as a function of the driver knuckle angle, measured
# on the running model by commanding the gripper across its range and reading
# the finger-tip link separation from TF (minus the 50.7 mm that separation
# retains when the pads are shut).
#
# Do not replace this with the linear 0.085*(1 - angle/0.8) approximation. The
# real stroke steepens as it closes and the linear fit is ~3 mm wide in the
# middle of the range, which is the difference between gripping a 60 mm cube and
# closing on thin air.
GRIPPER_CALIBRATION = [
    (0.00, 0.0848), (0.10, 0.0758), (0.20, 0.0661), (0.25, 0.0610),
    (0.30, 0.0559), (0.35, 0.0506), (0.40, 0.0452), (0.50, 0.0341),
    (0.60, 0.0228), (0.70, 0.0114), (0.80, 0.0000),
]

OBJECT_SIZE = 0.06       # target cube edge length (m)
# Pad interference, i.e. how far inside the object's width the pads are told to
# close. Measured across 0.004 / 0.007 / 0.010 with the friction grasp: the
# outcome barely changed (lift 91-95 mm, launch 0.99-1.13 m/s), because the
# knuckle is position-commanded and cannot stall on contact whatever the target.
# The grasp is carried by the attach-on-close plugin now (use_grasp_fix), so
# this only has to be deep enough to cross that plugin's attach_threshold.
GRIPPER_SQUEEZE = 0.007


def gripper_angle_for_width(width: float) -> float:
    """Knuckle angle that leaves the pad faces `width` apart."""
    if width >= GRIPPER_CALIBRATION[0][1]:
        return GRIPPER_CALIBRATION[0][0]
    for (a0, g0), (a1, g1) in zip(GRIPPER_CALIBRATION, GRIPPER_CALIBRATION[1:]):
        if g1 <= width <= g0:
            return a0 + (g0 - width) * (a1 - a0) / (g0 - g1)
    return GRIPPER_CALIBRATION[-1][0]


GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'
# Watchdog for a gripper goal that never resolves. The driver knuckle jitters
# around its target under the load of the finger linkage, and with a tight
# controller tolerance the action reported neither success nor a stall - the
# cycle sat in GRASP_CLOSE indefinitely.
GRIPPER_TIMEOUT = 6.0       # s
GRIPPER_POSITION_TOL = 0.06  # rad; close enough to call the move done
# NOTE: the close is a single goal on purpose. Stepping the command in over
# ~1 s (10 increments, fire-and-forget) was tried and is far worse - each new
# goal preempts the last, the mimic-joint PID chases a jittering target and
# goes unstable, and the cube left at 4.7 m/s and travelled 2.5 m. If the
# fast close is revisited, rate-limit it inside the controller or the mimic
# plugin, not by spamming action goals from here.

GRIPPER_OPEN = 0.0
# Closing to a fixed 0.70 rad asked for a 10.6 mm pad gap around the 60 mm cube,
# so the fingers drove ~25 mm into it. Derive the angle from the object width
# instead: 0.06 m less the 7 mm squeeze puts the knuckle at ~0.33 rad (measured
# 0.327 in Gazebo), where the pads rest on the faces. The grasp plugin's
# attach_threshold in ur3e_with_effector.urdf.xacro (0.20) must stay below this
# value or it never latches.
GRIPPER_CLOSED = gripper_angle_for_width(OBJECT_SIZE - GRIPPER_SQUEEZE)

# tool0 height for the pre-grasp pose. Measured from TF, the pads sit 0.119 m
# below tool0, so 0.30 keeps the finger tips ~0.18 m up, well clear of the
# 0.06 m cube before the straight vertical descent (avoids clipping/knocking it).
APPROACH_HEIGHT = 0.30
LIFT_HEIGHT = 0.32       # tool0 height for retract/lift after grasp
# Place target in the reachable front-left of the workspace (the pick is at
# ~0.4, 0). Kept at a similar radius to the pick so the transfer is a clean arc.
PLACE_X = 0.30
PLACE_Y = 0.30
PLACE_Z = 0.0
# tool0 z at grasp/place-down. Measured from TF (base_link -> finger tip) the
# pads sit 0.119 m below tool0, not the 0.16 m previously assumed. At the old
# 0.18 the tips bottomed out at z=0.061 - flat on the lid of the 0.06 m cube,
# which tapped it away instead of gripping it. 0.15 puts them at ~0.031, level
# with the cube's mid-height, so the pads close on the side faces. Perception
# reports the cube on the ground plane at z=0, so this offset is absolute.
GRASP_Z_OFFSET = 0.15

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
HOME_JOINTS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
# Park pose: pan swung 90 degrees off the pick area and the arm folded up, so
# the tool sits clear of both the workspace and the overhead camera's view of
# it. Distinct from HOME on purpose - it used to be the same six values, which
# made the dashboard's Home and Retract buttons do exactly the same thing.
# shoulder_lift stays negative (joint_limits.yaml caps it at 0).
RETRACT_JOINTS = [1.5707, -1.9, 1.4, -1.1, -1.5707, 0.0]

TOOL0_DOWN_ORI = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

REACH_MAX = 0.55  # conservative UR3e working radius, accounting for gripper offset

# Safe velocity limits for direct joint control (rad/s)
SAFE_VELOCITY = 0.4  # conservative max joint velocity for simulation

# --- Planning scene ---------------------------------------------------------
# The cube used to exist only in Gazebo, so MoveIt planned straight through it
# and the arm clipped it on the way in. It is published as a collision object at
# every detection, attached to the tool while carried, and put back on release.
CUBE_ID = 'target_cube'
GRASP_LINK = 'tool0'
GRIPPER_TOUCH_LINKS = [
    'gripper_mount_link', 'ur_to_robotiq_link', 'robotiq_85_base_link',
    'robotiq_85_left_finger_link', 'robotiq_85_left_finger_tip_link',
    'robotiq_85_left_inner_knuckle_link', 'robotiq_85_left_knuckle_link',
    'robotiq_85_right_finger_link', 'robotiq_85_right_finger_tip_link',
    'robotiq_85_right_inner_knuckle_link', 'robotiq_85_right_knuckle_link',
]

# --- Straight-line segments -------------------------------------------------
# Descents onto the cube and lifts off it are Cartesian so the tool travels
# vertically. A free joint-space plan between the same two poses arcs sideways,
# which is how the gripper kept knocking the cube over during the descent.
CARTESIAN_STEP = 0.005        # interpolation resolution (m)
CARTESIAN_JUMP = 5.0          # reject IK-branch flips between waypoints
CARTESIAN_MIN_FRACTION = 0.9  # accept the path only if ~fully solved
# A partial straight line is still worth executing: it makes real progress and
# lands the arm in a fresh configuration from which the rest often solves. Below
# CARTESIAN_RETRY_MIN_FRACTION there is too little progress for the retry to
# converge, so it goes to the planned fallback instead.
CARTESIAN_RETRY_MIN_FRACTION = 0.15
CARTESIAN_MAX_ATTEMPTS = 4

# Largest joint displacement an IK solution may ask for before it is treated as
# a reconfiguration rather than a move. Anything bigger is the arm flipping to a
# different branch, which is what produced the wild swings.
MAX_IK_JOINT_JUMP = 2.0  # rad

# The cube is held by friction, so the transfer has to stay gentle - at 0.3 the
# arm shed it mid-swing. Lower scaling keeps the lateral acceleration under what
# the pads can hold, and makes the motion look less frantic besides. Cartesian
# segments ignore these (Humble has no scaling field) and are paced by the
# velocity limits in config/moveit/joint_limits.yaml.
CARRY_VEL_SCALE = 0.15
CARRY_ACC_SCALE = 0.10


# Tasks the dashboard may request. pick_place uses the default place target;
# pick_to takes one from the request.
TASKS = {
    'home',
    'retract',
    'pick_place',
    'pick_to',
    'open_gripper',
    'close_gripper',
    'stop',
}


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
        self._scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=cbg)
        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path', callback_group=cbg)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory', callback_group=cbg)

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

        # --- dashboard task interface ---------------------------------------
        # `auto` reproduces the original behaviour of looping pick-and-place on
        # its own. Off by default: the dashboard drives the robot, and a robot
        # that starts moving the moment it is launched is a nuisance to test.
        self.declare_parameter('auto', False)
        self._auto = self.get_parameter('auto').value

        self._pending_task: str | None = None
        self._active_task: str | None = None
        self._task_x = PLACE_X
        self._task_y = PLACE_Y
        self._place_x = PLACE_X
        self._place_y = PLACE_Y
        self._stop_requested = False
        self._last_result = ''
        self._log = deque(maxlen=40)
        # Bumped whenever a task is started or abandoned. Motion callbacks
        # capture it when the motion is requested and refuse to fire if it has
        # moved on, so an aborted cycle cannot keep driving the arm. Without
        # this a stop only relabelled the state: the in-flight callback chain
        # ran to completion in the background and executed a whole phantom
        # pick-and-place, interleaved with whatever was commanded next.
        self._generation = 0

        self._status_pub = self.create_publisher(String, '/ur3e/task_status', 10)
        self._task_srv = self.create_service(
            RunTask, '/ur3e/run_task', self._run_task_cb, callback_group=cbg)
        self._status_timer = self.create_timer(
            0.25, self._publish_status, callback_group=cbg)

    def _set_state(self, state: State) -> None:
        self._state = state
        self.get_logger().info(f'State -> {state.name}')
        self._note(f'State -> {state.name}')

    # ------------------------------------------------------------------
    # Dashboard task interface
    # ------------------------------------------------------------------

    def _note(self, text: str) -> None:
        """Append to the ring buffer the dashboard shows as a live feed."""
        self._log.append({
            't': self.get_clock().now().nanoseconds / 1e9,
            'text': text,
        })

    def _busy(self) -> bool:
        return self._cycle_active or self._recovering

    def _guard(self, callback):
        """Bind `callback` to the current task generation.

        Every motion helper routes its completion through this, so abandoning a
        task (stop, or a failure that drops into recovery) is enough to make the
        rest of its callback chain evaporate instead of continuing to command
        motion behind the next task's back.
        """
        gen = self._generation

        def wrapped(*args, **kwargs):
            if gen != self._generation:
                self.get_logger().info(
                    'dropping stale callback from an abandoned task')
                return None
            return callback(*args, **kwargs)

        return wrapped

    def _abandon(self) -> None:
        self._generation += 1

    def _run_task_cb(self, request, response):
        task = (request.task or '').strip().lower()

        if task not in TASKS:
            response.accepted = False
            response.message = f"unknown task '{task}'"
            return response

        # Stop is the one command that is always accepted - refusing to stop
        # because the robot is busy would be exactly backwards.
        if task == 'stop':
            self._stop_requested = True
            self._pending_task = None
            response.accepted = True
            response.message = 'stopping'
            self._note('STOP requested')
            return response

        if self._busy() or self._pending_task is not None:
            response.accepted = False
            response.message = f'busy ({self._state.name})'
            return response

        if task == 'pick_to':
            radius = math.hypot(request.x, request.y)
            if radius > REACH_MAX:
                response.accepted = False
                response.message = (
                    f'target radius {radius:.3f} m exceeds reach {REACH_MAX} m')
                return response
            self._task_x = request.x
            self._task_y = request.y

        self._pending_task = task
        response.accepted = True
        response.message = f'{task} accepted'
        self._note(f'task accepted: {task}')
        return response

    def _publish_status(self) -> None:
        cube = self._cube_pose
        cube_out = None
        if cube is not None:
            cube_out = {
                'x': round(cube.position.x, 4),
                'y': round(cube.position.y, 4),
                'z': round(cube.position.z, 4),
                'reachable': math.hypot(
                    cube.position.x, cube.position.y) <= REACH_MAX,
            }

        joints = {
            n: round(self._current_joint_positions.get(n, 0.0), 4)
            for n in ARM_JOINTS if n in self._current_joint_positions
        }

        self._status_pub.publish(String(data=json.dumps({
            'state': self._state.name,
            'busy': self._busy(),
            'auto': self._auto,
            'active_task': self._active_task,
            'pending_task': self._pending_task,
            'last_result': self._last_result,
            'cube': cube_out,
            'joints': joints,
            'gripper': round(
                self._current_joint_positions.get(GRIPPER_JOINT, 0.0), 4),
            'place_target': {'x': self._place_x, 'y': self._place_y},
            'tasks': sorted(TASKS),
            'log': list(self._log),
        })))

    def _start_task(self, task: str) -> None:
        """Dispatch one accepted task. Called from _tick when idle."""
        self._active_task = task
        self._cycle_active = True
        # New task owns the arm from here; anything still in flight is stale.
        self._abandon()

        if task == 'home':
            self._set_state(State.HOME)
            self._move_joints(HOME_JOINTS, self._simple_done)
        elif task == 'retract':
            self._set_state(State.HOME)
            self._move_joints(RETRACT_JOINTS, self._simple_done)
        elif task == 'open_gripper':
            self._gripper(GRIPPER_OPEN, self._simple_done)
        elif task == 'close_gripper':
            self._gripper(GRIPPER_CLOSED, self._simple_done)
        else:  # pick_place / pick_to
            if task == 'pick_to':
                self._place_x, self._place_y = self._task_x, self._task_y
            else:
                self._place_x, self._place_y = PLACE_X, PLACE_Y
            self._note(f'placing at ({self._place_x:.3f}, {self._place_y:.3f})')
            self._approach(True)

    def _simple_done(self, success: bool) -> None:
        self._cycle_active = False
        self._last_result = 'ok' if success else 'failed'
        self._note(f'{self._active_task}: {self._last_result}')
        self._active_task = None
        self._set_state(State.WAITING)

    def _joint_state_cb(self, msg: JointState) -> None:
        self._current_joint_positions = dict(zip(msg.name, msg.position))
        self._joint_positions_known = True

    def _compute_safe_duration(self, target_joints: list[float]) -> float:
        if not self._joint_positions_known:
            return 3.0
        return max(self._max_joint_displacement(target_joints) / SAFE_VELOCITY, 1.0)

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

        # A stop drops into the normal ERROR recovery (reopen the gripper, drop
        # the cube from the scene, go HOME) rather than freezing where it is,
        # which would leave a half-open gripper and a phantom attached object.
        if self._stop_requested:
            self._stop_requested = False
            if self._busy() or self._state != State.WAITING:
                self._note('stopping: recovering to HOME')
                self._abandon()
                self._last_result = 'stopped'
                self._set_state(State.ERROR)
            else:
                self._last_result = 'stopped (was idle)'
            return

        if self._state == State.ERROR:
            if not self._recovering:
                self._recovering = True
                # Whatever was in flight is now orphaned - make sure it cannot
                # come back and drive the arm while recovery is running.
                self._abandon()
                self.get_logger().warn(
                    'Cycle failed; recovering (open gripper, return HOME)')
                self._gripper(GRIPPER_OPEN, self._recover_home)
            return

        if self._state != State.WAITING:
            return

        if self._cycle_active:
            return

        task = self._pending_task
        if task is None:
            # Only the original standalone mode starts a cycle unprompted.
            if not self._auto:
                return
            task = 'pick_place'
        self._pending_task = None

        if task in ('pick_place', 'pick_to'):
            cube = self._cube_pose
            if cube is None:
                self._last_result = 'no cube detected'
                self._note('pick refused: no cube detected')
                return

            radius = math.hypot(cube.position.x, cube.position.y)
            if radius > REACH_MAX:
                msg = (f'cube at radius {radius:.3f}m exceeds reach limit '
                       f'{REACH_MAX}m')
                self.get_logger().warn(msg)
                self._last_result = msg
                self._note(f'pick refused: {msg}')
                return

            self.get_logger().info('Cube detected, proceeding to approach')

        self._last_result = ''
        self._start_task(task)

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

        # Tell MoveIt the cube is there before any motion is planned towards it.
        self._scene_add_cube(self._cx, self._cy)

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
        # Drop the cube from the world before descending. At the grasp pose the
        # pads straddle the cube's mid-height, so the gripper links genuinely
        # overlap its collision box - with it still in the scene every
        # collision-checked fallback is rejected (compute_ik returns
        # NO_IK_SOLUTION and the descent never happens). The Cartesian path
        # already runs with avoid_collisions=False for the same reason; this
        # extends that to the plan/IK fallbacks. It comes back attached to the
        # tool once the gripper has closed.
        # Sequenced, not fire-and-forget: if the descent plans before the diff
        # lands it still sees the cube and fails.
        self._scene_remove_cube(
            # Straight down onto the cube - a planned motion arcs in sideways.
            then=lambda: self._move_cartesian(grasp, self._grasp_close))

    def _grasp_close(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        self._set_state(State.GRASP_CLOSE)
        # The cube rides with the tool from here, so it stops being an obstacle
        # and starts being part of the robot for collision checking.
        self._scene_attach_cube()
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
        # Straight up, so the cube clears before any lateral transfer.
        self._move_cartesian(lift, self._place)

    def _place(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        place = Pose(
            position=Point(x=self._place_x, y=self._place_y,
                           z=PLACE_Z + LIFT_HEIGHT),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.PLACE)
        self._move_pose(place, self._place_down)

    def _place_down(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        place = Pose(
            position=Point(x=self._place_x, y=self._place_y,
                           z=PLACE_Z + GRASP_Z_OFFSET),
            orientation=TOOL0_DOWN_ORI,
        )
        self._set_state(State.PLACE_DOWN)
        # Drop the cube from the scene before descending, for the mirror of the
        # reason it is dropped before the grasp: at the place height the
        # attached box comes to rest on the ground plane, and a collision-
        # checked fallback rejects that as a contact (compute_ik returns
        # NO_IK_SOLUTION). The gripper still physically holds it - only the
        # collision model is stood down for the descent.
        #
        # This must be sequenced rather than fire-and-forget. Racing the detach
        # against the descent is what made PLACE_DOWN fail intermittently: the
        # IK ran while the cube was still attached to tool0, where at this
        # height its underside rests on the ground plane, so every solution was
        # rejected as a collision and the cycle bailed into recovery - which
        # then dropped the cube from 0.32 m instead of placing it.
        self._scene_remove_cube(
            # Straight down again, so the cube is set down not swept down.
            then=lambda: self._move_cartesian(place, self._release))

    def _release(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        self._set_state(State.RELEASE)
        self._gripper(GRIPPER_OPEN, self._depart)

    def _depart(self, success: bool) -> None:
        if not success:
            self._set_state(State.ERROR)
            return

        # Straight up and clear of the cube before anything lateral happens, so
        # the gripper does not drag sideways across what it just set down.
        lift = Pose(
            position=Point(x=self._place_x, y=self._place_y,
                           z=PLACE_Z + LIFT_HEIGHT),
            orientation=TOOL0_DOWN_ORI,
        )
        self._move_cartesian(lift, self._finish)

    def _finish(self, success: bool) -> None:
        # Tool is clear now, so the cube can be an obstacle again. The next cycle
        # re-adds it at whatever position perception actually reports.
        self._scene_add_cube(self._place_x, self._place_y)
        self._set_state(State.DONE)
        self.get_logger().info('Pick-and-place complete')
        self._move_joints(RETRACT_JOINTS, self._reset, duration_sec=None)

    def _reset(self, success: bool = True) -> None:
        self._cycle_active = False
        self._last_result = 'ok'
        self._note(f'{self._active_task or "pick_place"}: ok')
        self._active_task = None
        self._set_state(State.WAITING)
        self._cube_pose = None
        self.get_logger().info('Ready for next pick-and-place')

    def _recover_home(self, success: bool = True) -> None:
        # Gripper reopened; return to HOME before retrying so the next detection
        # starts from a clean, unoccluded configuration. Drop the cube from the
        # tool first - if the cycle failed mid-carry it is still attached, and a
        # phantom attached object would corrupt every later plan.
        self._scene_remove_cube()
        self._move_joints(HOME_JOINTS, self._recover_done)

    def _recover_done(self, success: bool = True) -> None:
        self._recovering = False
        self._cycle_active = False
        self._cube_pose = None
        # A deliberate stop already set its own result; only an actual failure
        # should be reported as one.
        if not self._last_result:
            self._last_result = 'failed'
        self._note(f'{self._active_task or "task"}: {self._last_result}')
        self._active_task = None
        self._set_state(State.WAITING)
        self.get_logger().info('Recovered; waiting for cube')

    # ------------------------------------------------------------------
    # Planning scene: keep MoveIt aware of the cube
    # ------------------------------------------------------------------
    def _apply_scene(self, scene: PlanningScene, what: str, then=None) -> None:
        """Push a planning-scene diff.

        Fire-and-forget by default - most scene updates are advisory and motion
        should not stall on them. Pass `then` for the cases where the next
        motion is only valid once the diff has actually been applied: dropping
        the cube before a descent is one, because planning against a scene that
        still has it attached fails with NO_IK_SOLUTION.
        """
        if not self._scene_client.service_is_ready():
            self.get_logger().warn(
                f'apply_planning_scene unavailable, skipping {what}')
            if then is not None:
                then()
            return
        req = ApplyPlanningScene.Request()
        req.scene = scene

        def done(f):
            self._scene_result_cb(f, what)
            if then is not None:
                then()

        future = self._scene_client.call_async(req)
        future.add_done_callback(done)

    def _scene_result_cb(self, future, what: str) -> None:
        try:
            ok = future.result().success
        except Exception as e:
            self.get_logger().warn(f'planning scene {what} failed: {e}')
            return
        if not ok:
            self.get_logger().warn(f'planning scene {what} rejected')

    def _cube_collision_object(self, x: float, y: float) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = CUBE_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE]
        obj.primitives.append(box)
        # Perception projects onto the ground plane and reports z=0, so the cube
        # centre sits half an edge length up.
        obj.primitive_poses.append(Pose(
            position=Point(x=x, y=y, z=OBJECT_SIZE / 2.0),
            orientation=Quaternion(w=1.0)))
        obj.operation = CollisionObject.ADD
        return obj

    def _scene_add_cube(self, x: float, y: float) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._cube_collision_object(x, y))
        self._apply_scene(scene, 'add cube')

    def _scene_attach_cube(self) -> None:
        """Put the cube on the tool so the carry plans with it.

        Carries its own geometry rather than promoting an existing world
        object: the cube is removed from the scene before the descent, so by
        the time this runs there is nothing left to promote and a bare ADD
        would attach an empty object.
        """
        aco = AttachedCollisionObject()
        aco.link_name = GRASP_LINK
        aco.object = self._cube_collision_object(self._cx, self._cy)
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = GRIPPER_TOUCH_LINKS
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        self._apply_scene(scene, 'attach cube')

    def _scene_remove_cube(self, then=None) -> None:
        """Drop the cube from the scene entirely - off the tool and out of the world.

        Called the moment the gripper opens. Putting the cube straight back into
        the world here would leave the fingers wrapped around it, so every later
        plan would start in collision and be thrown away as invalid. It comes
        back as an obstacle once the tool has retreated clear of it.
        """
        aco = AttachedCollisionObject()
        aco.link_name = GRASP_LINK
        aco.object.id = CUBE_ID
        aco.object.operation = CollisionObject.REMOVE
        world_obj = CollisionObject()
        world_obj.header.frame_id = 'base_link'
        world_obj.id = CUBE_ID
        world_obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        scene.world.collision_objects.append(world_obj)
        self._apply_scene(scene, 'remove cube', then=then)

    # ------------------------------------------------------------------
    # Straight-line Cartesian motion
    # ------------------------------------------------------------------
    def _move_cartesian(self, target_pose: Pose, callback,
                        attempt: int = 1) -> None:
        """Move tool0 to `target_pose` along a straight line.

        Used for the descent onto the cube and the lift off it. Collision
        avoidance is off for these segments by design: they deliberately drive
        the gripper into contact with the object being manipulated, and a short
        straight vertical move cannot reach anything else. Every free-space
        motion still plans with full collision checking.
        """
        # Abandoned tasks must not keep driving the arm; see _guard.
        callback = self._guard(callback)
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'compute_cartesian_path unavailable, planning freely instead')
            self._move_pose(target_pose, callback)
            return

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'arm'
        req.link_name = GRASP_LINK
        req.waypoints = [target_pose]
        req.max_step = CARTESIAN_STEP
        req.jump_threshold = CARTESIAN_JUMP
        req.avoid_collisions = False
        # GetCartesianPath has no velocity/acceleration scaling fields in Humble
        # (added in Iron). These segments are paced by joint_limits.yaml instead.

        future = self._cartesian_client.call_async(req)
        future.add_done_callback(
            lambda f: self._cartesian_response_cb(
                f, callback, target_pose, attempt))

    def _cartesian_response_cb(self, future, callback, target_pose: Pose,
                               attempt: int = 1) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(f'compute_cartesian_path failed: {e}')
            self._move_pose(target_pose, callback)
            return

        if response.fraction < CARTESIAN_MIN_FRACTION:
            # A partial solution usually means the arm arrived in an IK branch
            # from which the rest of the straight line is not reachable - the
            # descent to the place pose hits this intermittently, because the
            # free plan into PLACE can land in different branches. Rather than
            # give up to a pose goal (which then fails collision-checked IK and
            # aborts the whole cycle), walk down as far as the line does solve
            # and re-request from there: the new configuration usually solves
            # the remainder.
            if (CARTESIAN_RETRY_MIN_FRACTION <= response.fraction
                    and attempt < CARTESIAN_MAX_ATTEMPTS):
                self.get_logger().warn(
                    f'straight-line path only {response.fraction:.0%} solved, '
                    f'executing that much and re-requesting '
                    f'(attempt {attempt}/{CARTESIAN_MAX_ATTEMPTS})')
                self._execute_trajectory(
                    response.solution,
                    lambda ok: (
                        self._move_cartesian(target_pose, callback, attempt + 1)
                        if ok else callback(False)))
                return

            self.get_logger().warn(
                f'straight-line path only {response.fraction:.0%} solved, '
                'falling back to a planned motion')
            self._move_pose(target_pose, callback)
            return

        self.get_logger().info(
            f'straight-line path solved ({response.fraction:.0%}), executing')
        self._execute_trajectory(response.solution, callback)

    def _execute_trajectory(self, trajectory, callback) -> None:
        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('execute_trajectory action unavailable')
            callback(False)
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._execute_response_cb(f, callback))

    def _execute_response_cb(self, future, callback) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('trajectory execution rejected')
            callback(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: callback(f.result().result.error_code.val == 1))

    def _move_joints(self, joints: list[float],
                     callback, duration_sec: float | None = None) -> None:
        # Abandoned tasks must not keep driving the arm; see _guard.
        callback = self._guard(callback)
        if not self._moveit_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('MoveIt2 not available, using direct joint control')
            self._send_joint_goal(joints, duration_sec, callback)
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 3.0
        goal.request.max_velocity_scaling_factor = CARRY_VEL_SCALE
        goal.request.max_acceleration_scaling_factor = CARRY_ACC_SCALE
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
        # Abandoned tasks must not keep driving the arm; see _guard.
        callback = self._guard(callback)
        if not self._moveit_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveIt2 not available for pose motion')
            callback(False)
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = CARRY_VEL_SCALE
        goal.request.max_acceleration_scaling_factor = CARRY_ACC_SCALE
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

    def _pose_ik_fallback(self, target_pose: Pose, callback) -> None:
        """Reach `target_pose` via IK when pose planning fails.

        Collision avoidance stays on. The previous version retried with
        `avoid_collisions=False` and then streamed the raw solution to the
        trajectory controller as a single point - no path checking at all, which
        is how the arm ended up driving through itself.
        """
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
        req.ik_request.avoid_collisions = True

        # Seed from where the arm actually is. KDL is a random-restart solver;
        # without a seed it returns an arbitrary branch, which is what made the
        # arm reconfigure violently between two nearby poses.
        if self._joint_positions_known:
            req.ik_request.robot_state.joint_state.name = list(ARM_JOINTS)
            req.ik_request.robot_state.joint_state.position = [
                self._current_joint_positions.get(n, 0.0) for n in ARM_JOINTS]

        future = self._ik_client.call_async(req)
        future.add_done_callback(lambda f: self._ik_response_cb(f, callback))

    def _ik_response_cb(self, future, callback) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'compute_ik call failed: {e}')
            callback(False)
            return

        if response.error_code.val != 1:
            self.get_logger().error(
                f'compute_ik failed: error_code={response.error_code.val}')
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

        jump = self._max_joint_displacement(joints)
        if jump > MAX_IK_JOINT_JUMP:
            self.get_logger().error(
                f'rejecting IK solution: it reconfigures the arm by {jump:.2f} rad '
                f'(limit {MAX_IK_JOINT_JUMP})')
            callback(False)
            return

        # Plan to the IK configuration through MoveIt rather than commanding the
        # controller directly, so the path there is collision checked and time
        # parameterised like every other motion.
        self.get_logger().info(
            f'IK fallback succeeded ({jump:.2f} rad move), planning to it')
        self._move_joints(joints, callback)

    def _max_joint_displacement(self, target_joints: list[float]) -> float:
        if not self._joint_positions_known:
            return 0.0
        return max(
            abs(target_joints[i] - self._current_joint_positions.get(name, 0.0))
            for i, name in enumerate(ARM_JOINTS))

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
        """Drive the gripper to `position`, and never block the cycle on it.

        A gripper goal that neither succeeds nor aborts used to wedge the state
        machine indefinitely. The result and a watchdog race each other now and
        the first one to finish owns the callback.
        """
        # Abandoned tasks must not keep driving the arm; see _guard.
        callback = self._guard(callback)
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper controller not available')
            callback(False)
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0

        done = {'fired': False}

        def finish(success: bool, why: str) -> None:
            if done['fired']:
                return
            done['fired'] = True
            self.destroy_timer(timer)
            if not success:
                self.get_logger().warn(f'gripper move to {position:.3f} rad {why}')
            callback(success)

        def on_timeout() -> None:
            # Judge by where the fingers actually are rather than by the action.
            actual = self._current_joint_positions.get(GRIPPER_JOINT)
            if actual is not None and abs(actual - position) <= GRIPPER_POSITION_TOL:
                finish(True, 'timed out but reached position')
            else:
                shown = f'{actual:.3f}' if actual is not None else 'unknown'
                finish(False, f'timed out at {shown} rad')

        timer = self.create_timer(GRIPPER_TIMEOUT, on_timeout)

        send_future = self._gripper_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._gripper_response_cb(f, finish))

    def _gripper_response_cb(self, future, finish) -> None:
        handle = future.result()
        if not handle or not handle.accepted:
            finish(False, 'was rejected')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: finish(f.result().result.reached_goal, 'did not reach the goal'))


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
