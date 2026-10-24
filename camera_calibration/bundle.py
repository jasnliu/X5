"""Bounded offline recovery using original image corners and the same model.

No devices, outlier deletion, free intrinsics, changed limits or motor settings.
The per-image observation refinement never uses robot poses or other captures.
Five reserved validation poses are accepted only by the separate evaluator.
"""
import copy
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from . import core, kinematics

POLICY = 'ippe_lm_fixed_corner_pose_bundle_v1'
OBSERVATION_POLICY = 'independent_ippe_seed_lm_100_1e-12_v1'


def refine_observations(samples, K, D, board):
    """Refine each accepted IPPE estimate against its OWN unchanged corners."""
    rows = copy.deepcopy(samples)
    changes = []
    for s in rows:
        ids = np.asarray(s['corner_ids'], dtype=int)
        uv = np.ascontiguousarray(s['corners_px'], dtype=float)
        if (ids.ndim != 1 or len(ids) < 10 or len(np.unique(ids)) != len(ids)
                or np.any(ids < 0) or np.any(ids >= len(board.points))
                or uv.shape != (len(ids), 2) or not np.isfinite(uv).all()):
            raise ValueError('Invalid saved image-corner observations')
        obj = np.ascontiguousarray(board.points[ids])
        original = core.checked_transform(s['T_color_board'])
        r = Rotation.from_matrix(original[:3, :3]).as_rotvec().reshape(3, 1)
        t = original[:3, 3].reshape(3, 1).copy()
        def pixel_error(rv, tv):
            image = cv2.projectPoints(obj, rv, tv, K, D)[0].reshape(-1, 2)
            return float(np.sqrt(np.mean(np.sum((image-uv)**2, axis=1))))
        before = pixel_error(r, t)
        r, t = cv2.solvePnPRefineLM(obj, uv, K, D, r, t,
            criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_COUNT, 100, 1e-12))
        transform = core.transform(Rotation.from_rotvec(r.ravel()).as_matrix(), t.ravel())
        core.checked_transform(transform)
        after = pixel_error(r, t)
        if (not np.isfinite(after) or after > before+1e-6
                or np.min((transform[:3, :3] @ obj.T + transform[:3, 3:4])[2]) <= 0):
            raise ValueError('Image-pose refinement worsened the observation or put the board behind the camera')
        distance, angle = core.distance(original, transform)
        changes.append(dict(sample=s['id'],original_reprojection_rms_px=before,
            refined_reprojection_rms_px=after,translation_change_mm=distance*1000,
            rotation_change_deg=angle))
        s.update(T_color_board=transform.tolist(),reprojection_rms_px=after)
    return rows, changes


def evaluation_samples(samples, fit, K, D, board):
    """Apply a frozen observation policy, never tune it on validation outcomes."""
    policy = fit.get('observation_policy')
    if policy is None:
        return samples
    if policy != OBSERVATION_POLICY:
        raise ValueError('Unknown calibration observation policy')
    return refine_observations(samples, K, D, board)[0]


def solve(training, K, D, board, model_path):
    if (len(training) < kinematics.MIN_CORRECTION_TRAIN
            or any(s['phase'] != 'training' for s in training)
            or len({s['id'] for s in training}) != len(training)):
        raise ValueError('Bundle recovery needs at least 20 distinct TRAINING poses; validation is never fitted')
    rows, changes = refine_observations(training, K, D, board)
    model = kinematics.HandKinematics(model_path)
    baseline = core.solve(rows, K, D, board)
    candidate = kinematics._refine(rows, K, D, board, model, baseline, corner_residuals=True)
    candidate.update(solver_policy=POLICY,observation_policy=OBSERVATION_POLICY,
        observation_refinement=changes,original_observations_preserved=True,
        objective=dict(position_scale_m=.005,rotation_scale_deg=1.,corner_scale_px=1.,loss='linear'),
        baseline_training_metrics=baseline['training_metrics'])
    if not candidate['training_passed']:
        candidate['refinement_rejection'] = 'Corner/pose bundle still fails unchanged training limits'
        return candidate
    predictions, original, folds = [], [], []
    for fold in range(5):
        fit_samples = [s for i,s in enumerate(rows) if i % 5 != fold]
        test_samples = [s for i,s in enumerate(rows) if i % 5 == fold]
        rigid = core.solve(fit_samples, K, D, board)
        fitted = kinematics._refine(fit_samples, K, D, board, model, rigid, corner_residuals=True)
        corrected = kinematics.corrected_samples(test_samples, fitted['kinematic_correction'], model_path)
        predictions.extend(core.metrics(corrected, np.asarray(fitted['T_base_color']),
            np.asarray(fitted['T_hand_board']), K, D, board.points)['samples'])
        original.extend(core.metrics(test_samples, np.asarray(rigid['T_base_color']),
            np.asarray(rigid['T_hand_board']), K, D, board.points)['samples'])
        folds.append(dict(training_ids=fitted['training_ids'],test_ids=[s['id'] for s in test_samples],
            offsets_deg=fitted['kinematic_correction']['offsets_deg']))
    report, old = core.aggregate_metrics(predictions), core.aggregate_metrics(original)
    def score(m):
        return sum((m[k]['rms']/scale)**2 for k,scale in
            [('translation_mm',5),('rotation_deg',1),('reprojection_px',4)])
    ratio = score(report)/max(score(old),1e-15)
    passed = core.passes(report) and ratio < .8
    candidate['internal_crossvalidation'] = dict(folds=folds,metrics=report,baseline_metrics=old,
        passed=passed,corrected_to_baseline_normalized_squared_error_ratio=ratio,
        maximum_allowed_ratio=.8,reserved_validation_used=False)
    candidate['training_passed'] = bool(candidate['training_passed'] and passed)
    if not passed:
        candidate['refinement_rejection'] = 'Corner/pose bundle failed training-only predictive checks'
    return candidate
