from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    HOME = auto()
    APPROACH = auto()
    GRASP = auto()
    RETRACT = auto()
    PLACE = auto()
    RELEASE = auto()
    DONE = auto()
    ERROR = auto()


ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

JOINT_TARGETS = {
    State.HOME:     [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
    State.APPROACH: [0.0, -1.2,    0.5, -1.8,    0.0, 0.0],
    State.RETRACT:  [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
    State.PLACE:    [0.0, -1.2,    0.5, -1.8,    0.0, 0.0],
}


class PickPlaceBT:
    def __init__(self):
        self._state = State.IDLE
        self._error = ''

    @property
    def state(self) -> State:
        return self._state

    @property
    def error(self) -> str:
        return self._error

    def tick(self, gripper_closed: bool = False) -> str | None:
        if self._state == State.IDLE:
            self._state = State.HOME
            return 'move'

        if self._state == State.HOME:
            self._state = State.APPROACH
            return 'move'

        if self._state == State.APPROACH:
            self._state = State.GRASP
            return 'open_gripper'

        if self._state == State.GRASP:
            self._state = State.RETRACT
            return 'close_gripper'

        if self._state == State.RETRACT:
            self._state = State.DONE
            return 'done'

        return None

    def target_joints(self) -> list[float] | None:
        return JOINT_TARGETS.get(self._state)

    def reset(self):
        self._state = State.IDLE
        self._error = ''
