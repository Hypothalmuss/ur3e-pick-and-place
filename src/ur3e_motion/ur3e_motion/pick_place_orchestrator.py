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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
# 0.22, not 0.30. The straight-line descent from hover to grasp is the
# fragile part of the cycle: at 0.30 it is a 150 mm line and solved only
# ~50% of the time in a scattered multi-cube scene. At 0.22 it is 70 mm and
# solves outright. The pads sit 0.119 m below tool0, so this still holds
# them 101 mm up - 41 mm clear of the 60 mm cube before the descent starts.
APPROACH_HEIGHT = 0.22
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

# --- Reach zones -------------------------------------------------------------
# Radius from base_link, on the table plane. Inside ZONE_NEAR the arm cannot get
# the tool vertical over the object without folding into itself - a cube knocked
# in there is unrecoverable and used to deadlock the cycle. Beyond ZONE_FAR the
# descent stops solving. Between them everything has worked: measured good picks
# at r = 0.405, 0.418, 0.424, 0.430, 0.432, 0.474.
ZONE_NEAR = 0.24
ZONE_FAR = 0.50


def zone_of(x: float, y: float) -> str:
    r = math.hypot(x, y)
    if r < ZONE_NEAR:
        return 'too_close'
    if r > ZONE_FAR:
        return 'too_far'
    return 'perfect'


# --- Drop zone ---------------------------------------------------------------
# The red square drawn on the ground in ur3e_workcell.world. Keep these in step
# with the drop_zone_* models there. Sized and placed so all four corners sit in
# the perfect annulus.
DROP_ZONE_X = 0.30
DROP_ZONE_Y = 0.22
DROP_ZONE_HALF = 0.08
# Where cubes get stacked inside the square. Four slots on a 90 mm pitch, which
# leaves a 30 mm gap between 60 mm cubes so a place never lands on a neighbour.
DROP_SLOTS = [
    (DROP_ZONE_X - 0.045, DROP_ZONE_Y - 0.045),
    (DROP_ZONE_X + 0.045, DROP_ZONE_Y - 0.045),
    (DROP_ZONE_X - 0.045, DROP_ZONE_Y + 0.045),
    (DROP_ZONE_X + 0.045, DROP_ZONE_Y + 0.045),
]
# A cube counts as already home if it is within this of a slot centre.
SLOT_OCCUPIED_RADIUS = 0.055


def in_drop_zone(x: float, y: float) -> bool:
    return (abs(x - DROP_ZONE_X) <= DROP_ZONE_HALF
            and abs(y - DROP_ZONE_Y) <= DROP_ZONE_HALF)

# Paces the direct-joint fallback used when MoveIt is unavailable (rad/s).
SAFE_VELOCITY = 1.5

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
# If the place descent still cannot be solved, re-approach the hover pose
# from a slightly different height to land in another IK branch.
# After each place the arm parks and perception needs a moment to see the
# settled scene before the next cube is chosen; picking from a stale frame
# sends the gripper to where a cube used to be.
TIDY_RESCAN_DELAY = 2.0  # s
# Hard stop on a sweep. Without it a cube that cannot be picked is retried
# forever: the blacklist used to key on the perception id, but ids are
# reassigned whenever a cube is occluded or shoved, so every retry looked
# like a brand-new cube (one run reached id #59 and ran for 10 minutes).
TIDY_MAX_ATTEMPTS = 10
# Blacklisted cubes are remembered by position on this grid, which survives
# id churn.
TIDY_BLACKLIST_GRID = 0.05  # m
PLACE_MAX_RETRIES = 3
# Retries LOWER the hover, they do not raise it. Raising it was the original
# design and it made things worse: a higher hover is a longer descent, so
# each retry was strictly harder than the attempt before it (99 re-approaches
# in one sweep, 17 outright failures). Lowering shortens the line and lands
# the arm in a different IK branch, which was the actual goal.
PLACE_RETRY_STEP = 0.02   # m removed from the hover height per retry
HOVER_MIN = 0.18          # never drop the pads closer than ~60 mm above the cube

# Largest joint displacement an IK solution may ask for before it is treated as
# a reconfiguration rather than a move. Anything bigger is the arm flipping to a
# different branch, which is what produced the wild swings.
# 3.0, not 2.0. Wrapped solutions are unwrapped before this check now, so
# what is left is a genuine reach across the workspace - and reaching a cube
# behind the robot legitimately costs ~2.6 rad. The guard exists to catch
# branch flips, not to cap travel: whatever survives is still planned through
# MoveIt with full collision checking rather than commanded directly.
MAX_IK_JOINT_JUMP = 3.0  # rad

# These were 0.15/0.10 because the cube was held by friction and a brisk swing
# shed it. The grasp is a rigid plugin attach now (use_grasp_fix), so the carry
# cannot be shaken loose and the old scaling just made every cycle take minutes.
# Cartesian segments ignore these (Humble has no scaling field) and are paced by
# the velocity limits in config/moveit/joint_limits.yaml instead.
CARRY_VEL_SCALE = 0.6
CARRY_ACC_SCALE = 0.4


# Tasks the dashboard may request. pick_place uses the default place target;
# pick_to takes one from the request.
TASKS = {
    'home',
    'retract',
    'pick_place',
    'pick_to',
    'open_gripper',
    'close_gripper',
    'tidy',
    'pick_selected',
    'stop',
    'estop',
    'reset',
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
    ESTOP = auto()


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
        self._cubes: dict[int, tuple[float, float]] = {}
        self._tidy_active = False
        self._tidy_done = 0
        self._tidy_failed: set = set()
        self._tidy_attempts = 0
        self._last_slot = None
        self._last_pick_cell = None
        self._tidy_resume_at = 0.0
        self._target_id: int | None = None
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
        self._place_retries = 0
        self._grasp_retries = 0
        # move_group outlives this node, so its scene has to be wiped once at
        # startup before anything is planned - see the STARTUP branch of _tick.
        self._scene_cleared = False
        self._scene_obstacles: set = set()
        self._stop_requested = False
        self._estopped = False
        self._live_goals: list = []
        self._selected_ids: set = set()
        self._last_result = ''
        self._log = deque(maxlen=40)
        # Bumped whenever a task is started or abandoned. Motion callbacks
        # capture it when the motion is requested and refuse to fire if it has
        # moved on, so an aborted cycle cannot keep driving the arm. Without
        # this a stop only relabelled the state: the in-flight callback chain
        # ran to completion in the background and executed a whole phantom
        # pick-and-place, interleaved with whatever was commanded next.
        self._generation = 0

        # Commands the arm to hold its current pose. Cancelling the action
        # goals is not enough to halt: once a trajectory has been handed to
        # joint_trajectory_controller it keeps executing, and an estop
        # measured 2.54 rad of further travel after the cancel. Publishing a
        # single-point trajectory at the present position overrides it.
        self._halt_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 1)
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

    def _track_goal(self, handle) -> None:
        """Remember an accepted goal so an emergency stop can cancel it."""
        if handle is None or not getattr(handle, 'accepted', False):
            return
        self._live_goals = [h for h in self._live_goals if h is not handle]
        self._live_goals.append(handle)
        if len(self._live_goals) > 8:
            self._live_goals.pop(0)

    def _cancel_live_goals(self) -> int:
        """Cancel everything in flight. This is what actually halts the arm.

        Cancelling the ExecuteTrajectory/MoveGroup goal stops the controller
        mid-trajectory, which is the difference between this and `stop`: stop
        reopens the gripper and drives back to HOME, i.e. it makes the robot
        move more, which is the wrong response to an emergency.
        """
        n = 0
        for handle in self._live_goals:
            try:
                handle.cancel_goal_async()
                n += 1
            except Exception as e:
                self.get_logger().warn(f'could not cancel a goal: {e}')
        self._live_goals = []
        return n

    def _halt_arm(self) -> None:
        """Command the controller to hold exactly where the arm is now."""
        if not self._joint_positions_known:
            return
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [self._current_joint_positions.get(n, 0.0)
                        for n in ARM_JOINTS]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        # Nearly-now, so it supersedes whatever is executing rather than
        # being queued behind it.
        pt.time_from_start = Duration(sec=0, nanosec=50_000_000)
        traj.points = [pt]
        # Published repeatedly: a single message can land while the
        # controller is mid-update and be superseded by the running goal.
        for _ in range(5):
            self._halt_pub.publish(traj)

    def _engage_estop(self) -> None:
        self._estopped = True
        self._abandon()          # orphan every in-flight callback chain
        n = self._cancel_live_goals()
        self._halt_arm()
        self._cycle_active = False
        self._recovering = False
        self._tidy_active = False
        self._pending_task = None
        self._target_id = None
        self._last_result = 'EMERGENCY STOP'
        self._note(f'EMERGENCY STOP - cancelled {n} goal(s), arm halted')
        self.get_logger().error(
            f'EMERGENCY STOP engaged, cancelled {n} in-flight goal(s)')
        self._set_state(State.ESTOP)

    def _do_reset(self) -> None:
        """Leave the estop and return to a known state for the next command."""
        self._estopped = False
        self._abandon()
        self._cycle_active = True
        self._active_task = 'reset'
        self._note('reset: clearing scene and homing')
        self.get_logger().info('reset: clearing scene and homing')
        self._scene_clear_obstacles()
        self._scene_remove_cube(
            then=lambda: self._move_joints(HOME_JOINTS, self._reset_done))

    def _reset_done(self, success: bool) -> None:
        self._cycle_active = False
        self._last_result = 'reset: ready' if success else 'reset: HOME failed'
        self._note(self._last_result)
        self._active_task = None
        self._set_state(State.WAITING)

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

        # Emergency stop outranks everything, including the busy check and
        # the estop latch itself.
        if task == 'estop':
            self._engage_estop()
            response.accepted = True
            response.message = 'EMERGENCY STOP - arm halted'
            return response

        if self._estopped and task != 'reset':
            response.accepted = False
            response.message = 'emergency stop engaged - reset required'
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

        if task == 'reset':
            self._do_reset()
            response.accepted = True
            response.message = 'reset: homing'
            return response

        if task == 'pick_selected':
            ids = set(int(i) for i in request.ids)
            if not ids:
                response.accepted = False
                response.message = 'select at least one cube'
                return response
            unknown = ids - set(self._cubes)
            if unknown:
                response.accepted = False
                response.message = f'unknown cube id(s): {sorted(unknown)}'
                return response
            self._selected_ids = ids

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
            'cubes': [
                {'id': i, 'x': round(x, 4), 'y': round(y, 4),
                 'r': round(math.hypot(x, y), 4),
                 'zone': zone_of(x, y),
                 'home': in_drop_zone(x, y)}
                for i, (x, y) in sorted(self._cubes.items())
            ],
            'zones': {'near': ZONE_NEAR, 'far': ZONE_FAR},
            'drop_zone': {'x': DROP_ZONE_X, 'y': DROP_ZONE_Y,
                          'half': DROP_ZONE_HALF},
            'target_id': self._target_id,
            'estopped': self._estopped,
            'selected_ids': sorted(self._selected_ids),
            'tidy': {'active': self._tidy_active, 'placed': self._tidy_done,
                     'skipped': len(self._tidy_failed)},
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
        self._place_retries = 0
        self._grasp_retries = 0
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
        elif task in ('tidy', 'pick_selected'):
            if task == 'tidy':
                self._selected_ids = set()   # every cube
            self._tidy_active = True
            self._tidy_done = 0
            self._tidy_failed.clear()
            self._tidy_attempts = 0
            self._tidy_step()
        else:  # pick_place / pick_to
            if task == 'pick_to':
                self._place_x, self._place_y = self._task_x, self._task_y
            else:
                self._place_x, self._place_y = PLACE_X, PLACE_Y
            if not self._select_target():
                self._finish_task('no cube to pick')
                return
            self._note(f'cube #{self._target_id} -> '
                       f'({self._place_x:.3f}, {self._place_y:.3f})')
            self._approach(True)

    def _select_target(self, exclude: set | None = None) -> bool:
        """Aim at the nearest pickable cube. False if there is nothing to take."""
        for cid, x, y in self._pickable_cubes():
            if exclude and self._cell(x, y) in exclude:
                continue
            self._target_id = cid
            self._cx, self._cy, self._cz = x, y, 0.0
            return True
        self._target_id = None
        return False

    def _tidy_step(self) -> None:
        """Pick the next cube outside the drop zone and put it in a free slot."""
        self._verify_last_place()
        self._tidy_attempts += 1
        if self._tidy_attempts > TIDY_MAX_ATTEMPTS:
            self._tidy_active = False
            self._finish_task(
                f'tidy gave up after {TIDY_MAX_ATTEMPTS} attempts, '
                f'{self._tidy_done} placed')
            return
        if not self._select_target(exclude=self._tidy_failed):
            done, bad = self._tidy_done, len(self._tidy_failed)
            self._tidy_active = False
            if done and bad:
                msg = f'tidy done: {done} placed, {bad} skipped'
            elif done:
                msg = f'tidy complete: {done} cube(s) placed'
            elif bad:
                msg = f'tidy failed: {bad} cube(s) unreachable'
            else:
                msg = 'tidy: nothing outside the drop zone'
            self._finish_task(msg)
            return

        slot = self._free_slot()
        if slot is None:
            self._tidy_active = False
            self._finish_task('tidy stopped: drop zone full')
            return

        self._place_x, self._place_y = slot
        self._last_slot = slot
        self._last_pick_cell = self._cell(self._cx, self._cy)
        self._place_retries = 0
        self._grasp_retries = 0
        self._note(f'tidy: cube #{self._target_id} -> slot '
                   f'({self._place_x:.3f}, {self._place_y:.3f})')
        self._approach(True)

    def _verify_last_place(self) -> None:
        """Did the previous cycle actually deliver a cube to its slot?

        A pick that clips the cube instead of grasping it still runs the
        whole cycle and comes back 'successful', so the only trustworthy
        check is whether a cube is now sitting in the slot we aimed at.
        """
        if self._last_slot is None:
            return
        sx, sy = self._last_slot
        landed = any(math.hypot(x - sx, y - sy) < SLOT_OCCUPIED_RADIUS
                     for x, y in self._cubes.values())
        if landed:
            self._tidy_done += 1
            self._note(f'tidy: cube {self._tidy_done} confirmed in slot')
        else:
            if self._last_pick_cell is not None:
                self._tidy_failed.add(self._last_pick_cell)
            self._note('tidy: cube was displaced, not carried - skipping it')
        self._last_slot = None
        self._last_pick_cell = None

    def _finish_task(self, message: str) -> None:
        """End the current task cleanly with a message for the dashboard."""
        self._cycle_active = False
        self._tidy_active = False
        self._last_result = message
        self._note(message)
        self._active_task = None
        self._target_id = None
        self._set_state(State.WAITING)

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
        # Only refresh while idle. Mid-cycle the carried cube is seen in the
        # gripper (or not at all), and letting that overwrite the target would
        # move the goalposts under a running pick.
        if self._state != State.WAITING or self._busy():
            return

        self._cubes = {
            o.aruco_id: (o.pose.position.x, o.pose.position.y)
            for o in msg.objects
        }
        self._cube_pose = msg.objects[0].pose if msg.objects else None

        now = self.get_clock().now().nanoseconds / 1e9
        if self._cubes and now - self._last_perception_log_time >= 2.0:
            summary = ', '.join(
                f'#{i}({x:.2f},{y:.2f}){zone_of(x, y)[0]}'
                for i, (x, y) in sorted(self._cubes.items()))
            self.get_logger().info(f'{len(self._cubes)} cube(s): {summary}')
            self._last_perception_log_time = now

    @staticmethod
    def _cell(x: float, y: float) -> tuple:
        """Coarse position key, stable across id reassignment."""
        return (round(x / TIDY_BLACKLIST_GRID), round(y / TIDY_BLACKLIST_GRID))

    def _pickable_cubes(self) -> list:
        """Cubes outside the drop zone that are actually reachable."""
        return sorted(
            ((i, x, y) for i, (x, y) in self._cubes.items()
             if zone_of(x, y) == 'perfect' and not in_drop_zone(x, y)
             and (not self._selected_ids or i in self._selected_ids)),
            key=lambda c: math.hypot(c[1], c[2]))

    def _free_slot(self):
        """First drop slot with no cube sitting in it."""
        for sx, sy in DROP_SLOTS:
            if not any(math.hypot(x - sx, y - sy) < SLOT_OCCUPIED_RADIUS
                       for x, y in self._cubes.values()):
                return sx, sy
        return None

    def _tick(self) -> None:
        # Latched: the arm stays exactly where the estop left it until reset.
        if self._estopped:
            return

        if self._state == State.STARTUP:
            # move_group outlives this node, so a restart inherits whatever
            # scene the previous run left behind. If that run died mid-carry
            # the cube is still attached to tool0 there, and every plan comes
            # back INVALID_MOTION_PLAN - the arm never even reaches HOME. Wipe
            # it before the first move.
            if not self._scene_cleared:
                self._scene_cleared = True
                self.get_logger().info('clearing any stale planning scene')
                self._scene_clear_obstacles()
                self._scene_remove_cube(
                    then=lambda: self._start_home())
                return
            self._start_home()
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

        # Resume a tidy sweep once perception has had a look at the new scene.
        if self._tidy_active:
            if (self.get_clock().now().nanoseconds / 1e9
                    >= self._tidy_resume_at):
                self._cycle_active = True
                self._place_retries = 0
                self._grasp_retries = 0
                self._tidy_step()
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

    def _start_home(self) -> None:
        self._set_state(State.HOME)
        self._move_joints(HOME_JOINTS, self._startup_done)

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

        # _cx/_cy/_cz are chosen by _start_task (or the tidy loop) before this
        # runs, so a multi-cube scene picks a specific cube rather than whatever
        # perception happened to list first.
        if self._target_id is None:
            self._set_state(State.ERROR)
            return

        # Every other cube is an obstacle. Without this MoveIt only knows about
        # the one being picked and plans straight through the rest - in a
        # three-cube sweep the arm shoved two of them 130-160 mm out of the
        # workspace while transferring the first.
        self._scene_sync_obstacles()

        # Tell MoveIt the cube is there before any motion is planned towards it.
        self._scene_add_cube(self._cx, self._cy)

        # Each retry lifts the hover a little, which puts the arm in a
        # different IK branch - the descent's feasibility depends on which
        # branch the free plan into this pose happened to choose.
        approach = Pose(
            position=Point(
                x=self._cx,
                y=self._cy,
                z=self._cz + max(
                    HOVER_MIN,
                    APPROACH_HEIGHT - PLACE_RETRY_STEP * self._grasp_retries),
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
            then=lambda: self._move_cartesian(
                grasp, self._grasp_descent_done))

    def _grasp_descent_done(self, success: bool) -> None:
        """Retry the approach rather than failing the pick.

        Same reasoning as _place_down_done: a descent that will not solve
        means the arm is in a configuration it cannot go straight down
        from, not that the cube is unreachable. Re-approaching from a
        different hover height lands it in another branch.
        """
        if success:
            self._grasp_close(True)
            return

        self._grasp_retries += 1
        if self._grasp_retries <= PLACE_MAX_RETRIES:
            self.get_logger().warn(
                f'grasp descent failed, re-approaching '
                f'({self._grasp_retries}/{PLACE_MAX_RETRIES})')
            self._note(f'grasp descent retry {self._grasp_retries}')
            self._approach(True)
            return

        self.get_logger().error('grasp descent failed after retries')
        self._set_state(State.ERROR)

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

        # Hover at APPROACH_HEIGHT, not LIFT_HEIGHT: this is the same geometry
        # as the grasp descent, which solves 100% consistently. Each retry
        # nudges the hover up a little, which lands the arm in a different IK
        # branch - the descent's feasibility depends entirely on which branch
        # the free plan into this pose happened to pick.
        hover = max(HOVER_MIN,
                    APPROACH_HEIGHT - PLACE_RETRY_STEP * self._place_retries)
        place = Pose(
            position=Point(x=self._place_x, y=self._place_y,
                           z=PLACE_Z + hover),
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
            then=lambda: self._move_cartesian(place, self._place_down_done))

    def _place_down_done(self, success: bool) -> None:
        """Retry the whole place segment rather than failing the cycle.

        A failed descent means the arm is in a configuration it cannot go
        straight down from - not that the place is impossible. Re-approaching
        from a slightly different hover height puts it in another branch, and
        that has been enough every time. Failing here instead used to abort the
        cycle into recovery, which dropped the cube from hover height.
        """
        if success:
            self._release(True)
            return

        self._place_retries += 1
        if self._place_retries <= PLACE_MAX_RETRIES:
            self.get_logger().warn(
                f'place descent failed, re-approaching '
                f'({self._place_retries}/{PLACE_MAX_RETRIES})')
            self._note(f'place descent retry {self._place_retries}')
            # The cube is still attached in reality; put it back in the scene
            # so the re-approach plans with it, matching the carry.
            self._scene_attach_cube()
            self._place(True)
            return

        self.get_logger().error('place descent failed after retries')
        self._set_state(State.ERROR)

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
        # HOME, not RETRACT. These two were the same six values until Retract
        # became a real park pose (pan +90 deg), and that silently moved the
        # end-of-cycle pose right across the workspace: the next approach then
        # needed a ~2.1 rad reconfiguration and was rejected by the IK jump
        # limit, so every pick after the first failed. HOME sits central and
        # close to everything. Retract stays available as its own task.
        self._move_joints(HOME_JOINTS, self._reset, duration_sec=None)

    def _reset(self, success: bool = True) -> None:
        self._cycle_active = False
        self._cube_pose = None
        self._target_id = None

        if self._tidy_active:
            # One cube home; go round again once perception has seen the
            # settled scene. _tick picks this up after TIDY_RESCAN_DELAY.
            # Success is not assumed here - the cycle can complete having
            # shoved the cube instead of carrying it, and reporting that as
            # 'placed' made the sweep claim 2 when only 1 had arrived. The
            # next rescan checks the slot and counts it then.
            self._last_result = f'tidy: {self._tidy_done} placed'
            self._note('tidy: cycle done, verifying on next scan')
            self._tidy_resume_at = (
                self.get_clock().now().nanoseconds / 1e9 + TIDY_RESCAN_DELAY)
            self._set_state(State.WAITING)
            return

        self._last_result = 'ok'
        self._note(f'{self._active_task or "pick_place"}: ok')
        self._active_task = None
        self._set_state(State.WAITING)
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

        if self._tidy_active:
            # Don't let one awkward cube stall the whole sweep - blacklist it
            # and carry on with the rest, then report it at the end.
            if self._target_id is not None:
                self._tidy_failed.add(self._cell(self._cx, self._cy))
                self._note(f'tidy: cube #{self._target_id} failed, skipping')
            self._target_id = None
            self._tidy_resume_at = (
                self.get_clock().now().nanoseconds / 1e9 + TIDY_RESCAN_DELAY)
            self._set_state(State.WAITING)
            return
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

    def _scene_sync_obstacles(self) -> None:
        """Publish every cube except the target as a collision object.

        The target is handled separately (it gets attached to the tool), so it
        is excluded here - leaving it in as a world obstacle would make its own
        grasp descent unplannable.
        """
        scene = PlanningScene()
        scene.is_diff = True

        wanted = set()
        for cid, (x, y) in self._cubes.items():
            if cid == self._target_id:
                continue
            name = f'obstacle_cube_{cid}'
            wanted.add(name)
            obj = self._cube_collision_object(x, y)
            obj.id = name
            scene.world.collision_objects.append(obj)

        for name in self._scene_obstacles - wanted:
            gone = CollisionObject()
            gone.header.frame_id = 'base_link'
            gone.id = name
            gone.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(gone)

        self._scene_obstacles = wanted
        if scene.world.collision_objects:
            self._apply_scene(scene, 'sync obstacles')

    def _scene_clear_obstacles(self) -> None:
        if not self._scene_obstacles:
            return
        scene = PlanningScene()
        scene.is_diff = True
        for name in self._scene_obstacles:
            gone = CollisionObject()
            gone.header.frame_id = 'base_link'
            gone.id = name
            gone.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(gone)
        self._scene_obstacles = set()
        self._apply_scene(scene, 'clear obstacles')

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
        self._track_goal(handle)
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
        self._track_goal(handle)
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
        self._track_goal(handle)
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
        # Include the gripper joint too: MoveIt can disregard a robot_state
        # that does not describe the whole model, and a disregarded seed is
        # the same as no seed at all.
        if self._joint_positions_known:
            seed = list(ARM_JOINTS) + [GRIPPER_JOINT]
            req.ik_request.robot_state.joint_state.name = seed
            req.ik_request.robot_state.joint_state.position = [
                self._current_joint_positions.get(n, 0.0) for n in seed]
            req.ik_request.robot_state.is_diff = False

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

        # compute_ik works from the URDF's +/-2*pi limits, not the +/-pi in
        # joint_limits.yaml, so it happily returns a solution a whole revolution
        # away from the current state. That is the same physical pose, but the
        # jump check saw it as a 6.07 rad reconfiguration and rejected the pick.
        # Unwrap each joint to its equivalent nearest the current angle first.
        joints = [self._nearest_equivalent(name, q)
                  for name, q in zip(ARM_JOINTS, joints)]

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

    def _nearest_equivalent(self, name: str, q: float) -> float:
        """Same angle, expressed nearest the joint's current value.

        Only shifts by whole revolutions, so the arm's physical pose is
        unchanged; it just stops a wrapped IK solution from looking like a
        full-turn reconfiguration. Results outside +/-pi are left alone -
        joint_limits.yaml keeps the planner inside that band, and quietly
        handing it something outside would only fail later.
        """
        cur = self._current_joint_positions.get(name)
        if cur is None:
            return q
        best = q
        while best - cur > math.pi:
            best -= 2.0 * math.pi
        while cur - best > math.pi:
            best += 2.0 * math.pi
        return best if abs(best) <= math.pi else q

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
        self._track_goal(handle)
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
