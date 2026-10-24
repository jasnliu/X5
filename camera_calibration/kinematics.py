"""Offline, calibration-local joint-coordinate refinement. No motor interfaces.

These are effective model offsets, NOT measured hardware encoder-zero errors.
J1 and J7 are fixed: their offsets are indistinguishable from the free camera
and board-mount transforms. Neither the source URDF nor joint converters change.
"""
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from . import core

SCHEMA = 'openarmx-calibration-joint-offsets-v1'
FITTED_JOINTS = core.JOINTS[1:6]
MAX_OFFSET_RAD = np.radians(5.)
MIN_CORRECTION_TRAIN = 20


class HandKinematics:
    """Batched FK of the saved world-to-right-hand chain, never the live model."""
    def __init__(self, path):
        self.model_sha256 = core.digest(path)
        root = ET.parse(path).getroot()
        if root.find('ros2_control') is not None:
            raise ValueError('Hardware control tags are forbidden')
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        link = core.HAND
        visited = set()
        moving = []
        while link != 'world':
            if link in visited or link not in parents:
                raise ValueError('Disconnected or cyclic right-hand URDF chain')
            visited.add(link)
            joint = parents[link]
            origin = joint.find('origin')
            T = np.eye(4)
            if origin is not None:
                T[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                T[:3, :3] = Rotation.from_euler(
                    'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            core.checked_transform(T)
            index, axis = None, None
            if joint.get('type') != 'fixed':
                name = joint.get('name')
                if (name not in core.JOINTS or joint.get('type') not in ('revolute', 'continuous')
                        or joint.find('mimic') is not None):
                    raise ValueError('Unsupported moving joint in right-hand chain')
                index = core.JOINTS.index(name)
                moving.append(name)
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
                    raise ValueError('Invalid URDF joint axis')
                axis = axis / np.linalg.norm(axis)
            self.chain.insert(0, (T, index, axis))
            link = joint.find('parent').get('link')
        if sorted(moving) != sorted(core.JOINTS):
            raise ValueError('Expected all seven right-arm joints in the hand chain')

    def fk(self, q):
        q = np.asarray(q, dtype=float)
        if q.ndim != 2 or q.shape[1] != 7 or not len(q) or not np.isfinite(q).all():
            raise ValueError('Expected finite N by 7 joint coordinates')
        result = np.tile(np.eye(4), (len(q), 1, 1))
        for origin, index, axis in self.chain:
            result = result @ origin
            if index is not None:
                motion = np.tile(np.eye(4), (len(q), 1, 1))
                motion[:, :3, :3] = Rotation.from_rotvec(q[:, index, None] * axis).as_matrix()
                result = result @ motion
        return result

    def measured(self, samples):
        try:
            q = np.array([[s['joints_rad'][name] for name in core.JOINTS] for s in samples], dtype=float)
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError('Saved seven-joint readings are required for kinematic refinement') from e
        A = self.fk(q)
        saved = np.array([core.checked_transform(s['T_base_hand']) for s in samples])
        if not np.allclose(A, saved, atol=1e-8, rtol=0):
            raise ValueError('Saved joint readings and hand FK disagree with the session URDF')
        return q


def correction_offsets(correction):
    if correction.get('schema') != SCHEMA:
        raise ValueError('Unknown calibration-local kinematics schema')
    offsets = correction.get('offsets_rad', {})
    if set(offsets) != set(core.JOINTS):
        raise ValueError('Correction must explicitly specify all seven right-arm offsets')
    delta = np.array([offsets[name] for name in core.JOINTS], dtype=float)
    if (not np.isfinite(delta).all() or np.max(abs(delta)) > MAX_OFFSET_RAD
            or delta[0] != 0 or delta[6] != 0):
        raise ValueError('Invalid, excessive or unidentifiable joint correction')
    if correction.get('coordinate_rule') != 'q_model = q_saved + offsets_rad':
        raise ValueError('Unknown joint-coordinate correction convention')
    return delta


def corrected_joint_values(joints, correction):
    """Map existing encoder_to_joint output into calibration model coordinates.

    Do not add these values to firmware zeros or to raw encoder packets. For a
    future inverse-kinematics controller the inverse mapping is subtraction,
    BEFORE the existing joint_to_motor conversion. No controller is wired here.
    """
    delta = correction_offsets(correction)
    result = dict(joints)
    for name, offset in zip(core.JOINTS, delta):
        result[name] = float(joints[name] + offset)
        if not np.isfinite(result[name]):
            raise ValueError('Nonfinite joint coordinate')
    return result


def corrected_samples(samples, correction, model_path):
    model = HandKinematics(model_path)
    if model.model_sha256 != correction.get('model_sha256'):
        raise ValueError('Kinematic correction does not match the session URDF')
    q = model.measured(samples)
    A = model.fk(q + correction_offsets(correction))
    return [dict(s, T_base_hand=a.tolist()) for s, a in zip(samples, A)]


def _refine(samples, K, D, board, model, baseline, *, corner_residuals=False):
    q = model.measured(samples)
    B = np.array([core.checked_transform(s['T_color_board']) for s in samples])
    objects = [np.ascontiguousarray(board.points[np.asarray(s['corner_ids'], dtype=int)])
               for s in samples] if corner_residuals else []
    observed = [np.asarray(s['corners_px'], dtype=float) for s in samples] if corner_residuals else []

    def unpack(p):
        delta = np.r_[0., p[12:], 0.]
        return core._unpack(p[:6]), core._unpack(p[6:12]), model.fk(q + delta)

    def residual(p):
        C, X, A = unpack(p)
        lhs, rhs = A @ X, C @ B
        pos = (lhs[:, :3, 3] - rhs[:, :3, 3]) / .005
        rot = Rotation.from_matrix(lhs[:, :3, :3].transpose(0, 2, 1)
                                   @ rhs[:, :3, :3]).as_rotvec() / np.radians(1)
        pose = np.c_[pos, rot].ravel()
        if not corner_residuals:
            return pose
        # Fixed recovery policy: retain the original 5 mm / 1 degree pose
        # scales and add each observed corner at a one-pixel residual scale.
        # No intrinsics, board dimensions, joints or thresholds are relaxed.
        pred = core.inverse(C) @ lhs
        rv = Rotation.from_matrix(pred[:, :3, :3]).as_rotvec()
        pixels = [
            (cv2.projectPoints(obj, rv[i], pred[i, :3, 3], K, D)[0].reshape(-1, 2)-uv).ravel()
            for i, (obj, uv) in enumerate(zip(objects, observed))]
        return np.r_[np.concatenate(pixels), pose]

    seed = np.r_[core._pack(np.array(baseline['T_base_color'])),
                 core._pack(np.array(baseline['T_hand_board'])), np.zeros(5)]
    lower = np.r_[np.full(12, -np.inf), np.full(5, -MAX_OFFSET_RAD)]
    upper = -lower
    opt = least_squares(residual, seed, loss='linear' if corner_residuals else 'soft_l1', bounds=(lower, upper),
                        max_nfev=350, xtol=1e-10, ftol=1e-10, gtol=1e-10)
    if not opt.success or not np.isfinite(opt.x).all():
        raise ValueError('Kinematic refinement did not converge')
    if np.max(abs(opt.x[12:])) >= .99 * MAX_OFFSET_RAD:
        raise ValueError('Kinematic refinement reached its five-degree parameter bound')

    # The five new parameters must be distinguishable from the twelve nuisance
    # camera/mount parameters. Reject rank-deficient or severely ill-conditioned
    # solutions instead of blessing arbitrary offsets. This is not uncertainty
    # certification: residual mechanical/model errors remain possible.
    J = opt.jac
    nuisance, offsets = J[:, :12], J[:, 12:]
    independent = offsets - nuisance @ np.linalg.lstsq(nuisance, offsets, rcond=None)[0]
    singular = np.linalg.svd(independent, compute_uv=False)
    if (np.linalg.matrix_rank(nuisance) < 12 or singular[-1] < 1e-6
            or singular[-1] / singular[0] < 1e-4):
        raise ValueError('Insufficient joint diversity to identify a local kinematic correction')

    C, X, A = unpack(opt.x)
    corrected = [dict(s, T_base_hand=a.tolist()) for s, a in zip(samples, A)]
    delta = np.r_[0., opt.x[12:], 0.]
    correction = dict(
        schema=SCHEMA, model_sha256=model.model_sha256,
        coordinate_rule='q_model = q_saved + offsets_rad',
        offsets_rad=dict(zip(core.JOINTS, delta.tolist())),
        offsets_deg=dict(zip(core.JOINTS, np.degrees(delta).tolist())),
        fitted_joints=FITTED_JOINTS, fixed_gauge_joints=[core.JOINTS[0], core.JOINTS[6]],
        maximum_allowed_absolute_offset_deg=5.,
        independent_jacobian_singular_values=singular.tolist(),
        independent_jacobian_condition=float(singular[0] / singular[-1]),
        observed_joint_ranges_rad={name: [float(q[:, j].min()), float(q[:, j].max())]
                                   for j, name in enumerate(core.JOINTS)},
        scope='Calibration-local effective geometry; not independently measured motor zeros',
        warning='Apply to encoder_to_joint output, which already includes the existing right J5 offset. '
                'Do not apply twice or change firmware. Accuracy outside the sampled region is unverified.')
    m = core.metrics(corrected, C, X, K, D, board.points)
    return dict(T_base_color=C.tolist(), T_hand_board=X.tolist(),
                coverage=baseline['coverage'], training_metrics=m,
                training_passed=core.passes(m), training_ids=[s['id'] for s in samples],
                kinematic_correction=correction,
                method=('Park initialization + bounded J2-J6 corner/pose bundle least-squares'
                        if corner_residuals else 'Park initialization + bounded J2-J6 and rigid-transform soft-L1 refinement'))


def solve_with_kinematics(training, K, D, board, model_path):
    """Rigid fit first; guarded local correction only if rigid consistency fails.

    No validation data is accepted here. Every training pose participates; no
    outliers are discarded and core.passes thresholds are never relaxed.
    """
    if any(s['phase'] != 'training' for s in training):
        raise ValueError('Only training poses may enter the calibration solver')
    if len({s['id'] for s in training}) != len(training):
        raise ValueError('Duplicate training sample IDs')
    baseline = core.solve(training, K, D, board)
    baseline['solver_policy'] = 'rigid_then_crossvalidated_local_kinematics_v1'
    if baseline['training_passed']:
        return baseline
    try:
        if len(training) < MIN_CORRECTION_TRAIN:
            raise ValueError('Kinematic refinement needs at least 20 training poses for five-fold checks')
        model = HandKinematics(model_path)
        candidate = _refine(training, K, D, board, model, baseline)
        candidate['baseline_training_metrics'] = baseline['training_metrics']
        if not candidate['training_passed']:
            # Retain the actual refined errors instead of showing the much
            # worse discarded rigid fit as though refinement never improved it.
            candidate['refinement_rejection'] = 'Local kinematic refinement still fails the unchanged training limits'
            candidate['solver_policy'] = baseline['solver_policy']
            return candidate
        rows, baseline_rows, folds = [], [], []
        for fold in range(5):
            fit_samples = [s for i, s in enumerate(training) if i % 5 != fold]
            test_samples = [s for i, s in enumerate(training) if i % 5 == fold]
            rigid = core.solve(fit_samples, K, D, board)
            fitted = _refine(fit_samples, K, D, board, model, rigid)
            corrected = corrected_samples(test_samples, fitted['kinematic_correction'], model_path)
            m = core.metrics(corrected, np.array(fitted['T_base_color']),
                             np.array(fitted['T_hand_board']), K, D, board.points)
            old = core.metrics(test_samples, np.array(rigid['T_base_color']),
                               np.array(rigid['T_hand_board']), K, D, board.points)
            rows.extend(m['samples']); baseline_rows.extend(old['samples'])
            folds.append(dict(training_ids=fitted['training_ids'],
                              test_ids=[s['id'] for s in test_samples],
                              offsets_deg=fitted['kinematic_correction']['offsets_deg']))
        report, old = core.aggregate_metrics(rows), core.aggregate_metrics(baseline_rows)

        def score(m):
            return sum((m[key]['rms'] / scale)**2 for key, scale in
                       [('translation_mm', 5), ('rotation_deg', 1), ('reprojection_px', 4)])

        ratio = score(report) / max(score(old), 1e-15)
        cv_passed = core.passes(report) and ratio < .8
        candidate['internal_crossvalidation'] = dict(
            folds=folds, metrics=report, baseline_metrics=old, passed=cv_passed,
            corrected_to_baseline_normalized_squared_error_ratio=ratio,
            maximum_allowed_ratio=.8, reserved_validation_used=False)
        candidate['training_passed'] = bool(candidate['training_passed'] and cv_passed)
        if not cv_passed:
            candidate['refinement_rejection'] = 'Correction failed training-only predictive checks'
        candidate['solver_policy'] = baseline['solver_policy']
        return candidate
    except ValueError as e:
        baseline['refinement_rejection'] = str(e)
        return baseline
