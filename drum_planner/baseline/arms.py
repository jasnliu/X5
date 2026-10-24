"""Selected-arm execution contract; old sessions without a side mean right."""
from ..common import arm_side


def side_of(data):
    return arm_side(data.get('moving_side', 'right'))


def other_side(side):
    return 'left' if arm_side(side) == 'right' else 'right'


def closed_gripper(side):
    if arm_side(side) == 'right':
        from centering.motors import RIGHT_GRIPPER_CLOSED
        return RIGHT_GRIPPER_CLOSED
    from camera_playback.left_hold import LEFT_GRIPPER_TARGET
    return LEFT_GRIPPER_TARGET  # negative internal angle, positive raw CAN target


def require_side(data, side):
    if side_of(data) != arm_side(side):
        raise ValueError(f'Arm mismatch: {side_of(data)} plan/controller cannot run in {side} mode')


def passive_snapshot(req):
    other = other_side(side_of(req))
    value = {other: list(req[other])}
    if other == 'left':
        value['left_opening_m'] = req['left_opening_m']
    return value
