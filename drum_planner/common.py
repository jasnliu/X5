import json
from pathlib import Path

import numpy as np

from camera_calibration.core import ROOT, JOINTS, checked_transform, digest, passes
from camera_calibration.kinematics import correction_offsets
from camera_calibration.io import atomic_json

DEFAULT_CALIBRATION = ROOT / 'calibration_sessions/20261024T211629Z-1227b7b1-reprocessed/calibration.json'
DEFAULT_RECORDING = ROOT / 'recordings/record3.json'
DEFAULT_LEFT_RECORDING = ROOT / 'left_recordings/record1.json'
LEFT_JOINTS = [name.replace('right', 'left') for name in JOINTS]


def arm_side(side):
    if side not in ('left', 'right'):
        raise ValueError('Arm side must be left or right')
    return side


def arm_joints(side='right'):
    return JOINTS if arm_side(side) == 'right' else LEFT_JOINTS


def goal_recording(side='right'):
    return DEFAULT_RECORDING if arm_side(side) == 'right' else DEFAULT_LEFT_RECORDING


def calibration(path=DEFAULT_CALIBRATION):
    path = Path(path).resolve()
    obj = json.loads(path.read_text())
    if (obj.get('schema') != 'openarmx-eye-to-hand-calibration-v2'
            or not obj.get('validated') or not obj.get('requires_kinematic_correction')
            or not passes(obj['training']['training_metrics'])
            or not passes(obj['validation']['metrics'])
            or obj['validation']['used_for_fitting']):
        raise ValueError('A passing, held-out validated v2 calibration is required')
    if set(obj['training']['training_ids']) & set(obj['validation']['heldout_ids']):
        raise ValueError('Training/validation overlap')
    if obj['model_sha256'] != digest(ROOT / 'model/openarmx.urdf'):
        raise ValueError('Calibration and robot model differ')
    if obj['encoder_source_sha256'] != digest(ROOT / 'safe_zone/encoder.py'):
        raise ValueError('Encoder convention changed since calibration')
    correction_offsets(obj['kinematic_correction'])
    for key in ('T_base_depth', 'T_base_color'):
        checked_transform(obj[key])
    return obj


def endpoint(path=None, *, side='right'):
    arm_side(side)
    path = goal_recording(side) if path is None else path
    record = json.loads(Path(path).read_text())
    if (record.get('schema') != f'openarmx-{side}-motion-recording-v1'
            or record.get('joint_order') != arm_joints(side)
            or record.get('joint_position_unit') != 'rad'
            or record.get('model_sha256') != digest(ROOT / 'model/openarmx.urdf')):
        raise ValueError(f'Incompatible {side}-arm recording')
    q = np.asarray(record['samples'][-1]['positions_rad'], dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError('Invalid recording endpoint')
    return q


def read_state(path):
    obj = json.loads(Path(path).read_text())
    if obj.get('schema') != 'x5-planner-start-state-v1' or obj.get('coordinate_convention') != 'encoder_to_joint':
        raise ValueError('Expected a saved starting state in existing encoder_to_joint coordinates')
    for side in ('right', 'left'):
        q = np.asarray(obj[side + '_rad'], dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('Provide seven finite angles for BOTH arms; the opposite arm remains static')
    if obj.get('right_gripper_closed_motor_joint_rad') != float(np.radians(7.)):
        raise ValueError('Right gripper must use customized CLOSED +7 degree convention')
    opening = obj.get('left_gripper_opening_m')
    if not isinstance(opening, (int, float)) or not 0 <= opening <= .044:
        raise ValueError('Invalid static left gripper opening')
    return obj


def hardware_unavailable(*args, **kwargs):
    raise RuntimeError('SOFTWARE-ONLY preview API: no actuation here; use the separately gated live execution command')
