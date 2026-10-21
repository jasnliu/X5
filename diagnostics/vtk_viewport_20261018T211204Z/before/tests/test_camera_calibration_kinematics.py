"""Offline regression tests for guarded, calibration-local model refinement."""
import copy
import json
import socket
import subprocess

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camera_calibration import core, kinematics
from camera_calibration.io import Session, load_session, copy_session_for_solve
from safe_zone.geometry import Model


@pytest.fixture(autouse=True)
def forbid_hardware(monkeypatch):
    def denied(*args, **kwargs):
        raise AssertionError('Offline calibration must never access hardware/processes')
    monkeypatch.setattr(socket, 'socket', denied)
    monkeypatch.setattr(subprocess, 'Popen', denied)


@pytest.fixture
def dataset():
    model = kinematics.HandKinematics(core.URDF)
    board = core.Board()
    K = np.array([[900., 0., 640.], [0., 900., 360.], [0., 0., 1.]])
    D = np.zeros(8)
    C = core.transform(Rotation.from_euler('xyz', [.08, -.12, .1]).as_matrix(), [.1, .1, -1.1])
    X = core.transform(Rotation.from_euler('xyz', [.2, .1, -.25]).as_matrix(), [.03, .02, .16])
    rng = np.random.default_rng(481)
    q = rng.uniform([-.5, -.2, -.8, .5, -1., -.7, -1.], [.5, .3, .8, 1.7, 1., .7, 1.], (30, 7))
    delta = np.radians([0., -1.1, 2.4, 1.6, -.7, 1.3, 0.])
    A, actual = model.fk(q), model.fk(q + delta)
    samples = []
    for i, (a, corrected, angles) in enumerate(zip(A, actual, q)):
        B = core.inverse(C) @ corrected @ X
        assert np.min((B[:3, :3] @ board.points.T + B[:3, 3:4])[2]) > 0
        uv = cv2.projectPoints(board.points, Rotation.from_matrix(B[:3, :3]).as_rotvec(),
                               B[:3, 3], K, D)[0].reshape(-1, 2)
        samples.append(dict(id=i+1, phase='validation' if i < 5 else 'training',
                            joints_rad=dict(zip(core.JOINTS, angles.tolist())),
                            T_base_hand=a.tolist(), T_color_board=B.tolist(),
                            corner_ids=list(range(15)), corners_px=uv.tolist()))
    return board, K, D, C, X, delta, samples


def fit_dataset(dataset):
    board, K, D, _, _, _, samples = dataset
    return kinematics.solve_with_kinematics(samples[5:], K, D, board, core.URDF)


def make_session(tmp_path, dataset):
    _, K, D, _, _, _, samples = dataset
    session = Session(tmp_path, capture_policy='validation_first_auto_v1')
    session.data['camera'] = dict(
        color=dict(width=1280, height=720, intrinsic_width=1280, intrinsic_height=720,
                   K=K.tolist(), D_opencv_order=D.tolist(), distortion_model=4),
        T_color_depth_m=core.transform(t=[.025, 0, 0]).tolist())
    for sample in samples:
        session.append(sample, np.zeros((2, 3, 3), np.uint8))
    return session


def test_batched_fk_matches_existing_model(dataset):
    *_, samples = dataset
    model = Model(core.URDF)
    for s in samples:
        assert np.allclose(model.transforms(s['joints_rad'])[core.HAND], s['T_base_hand'], atol=1e-12)


def test_recovers_known_offsets_and_transforms_without_mutating_observations(dataset):
    board, K, D, C, X, delta, samples = dataset
    before = copy.deepcopy(samples)
    assert not core.solve(samples[5:], K, D, board)['training_passed']
    fit = fit_dataset(dataset)
    assert fit['training_passed'] and fit['internal_crossvalidation']['passed']
    assert np.allclose(kinematics.correction_offsets(fit['kinematic_correction']), delta, atol=1e-7)
    assert np.allclose(fit['T_base_color'], C, atol=1e-7)
    assert np.allclose(fit['T_hand_board'], X, atol=1e-7)
    assert samples == before
    seen = []
    for fold in fit['internal_crossvalidation']['folds']:
        assert set(fold['training_ids']).isdisjoint(fold['test_ids'])
        assert set(fold['training_ids'] + fold['test_ids']) == set(range(6, 31))
        seen.extend(fold['test_ids'])
    assert sorted(seen) == list(range(6, 31))


def test_rigid_success_does_not_add_unnecessary_parameters(dataset, monkeypatch):
    board, K, D, C, X, _, samples = dataset
    for s in samples:
        B = core.inverse(C) @ np.array(s['T_base_hand']) @ X
        s['T_color_board'] = B.tolist()
        s['corners_px'] = cv2.projectPoints(board.points, Rotation.from_matrix(B[:3, :3]).as_rotvec(),
                                           B[:3, 3], K, D)[0].reshape(-1, 2).tolist()
    def denied(*args, **kwargs):
        raise AssertionError('Passing rigid fit must not trigger extra fitting')
    monkeypatch.setattr(kinematics, '_refine', denied)
    fit = fit_dataset(dataset)
    assert fit['training_passed'] and 'kinematic_correction' not in fit


@pytest.mark.parametrize('problem', ['missing_joint', 'nan_joint', 'mismatched_fk', 'insufficient_count'])
def test_bad_refinement_inputs_fail_closed(dataset, problem):
    samples = dataset[-1]
    if problem == 'missing_joint':
        del samples[5]['joints_rad'][core.JOINTS[3]]
    elif problem == 'nan_joint':
        samples[5]['joints_rad'][core.JOINTS[3]] = float('nan')
    elif problem == 'mismatched_fk':
        samples[5]['T_base_hand'][0][3] += .01
    else:
        del samples[24:]
    fit = fit_dataset(dataset)
    assert not fit['training_passed'] and fit['refinement_rejection']


def test_heldout_cannot_enter_solver(dataset):
    board, K, D, *_, samples = dataset
    with pytest.raises(ValueError, match='Only training'):
        kinematics.solve_with_kinematics(samples, K, D, board, core.URDF)
    with pytest.raises(ValueError, match='Duplicate'):
        kinematics.solve_with_kinematics(samples[5:] + [samples[5]], K, D, board, core.URDF)


def test_bad_pose_not_discarded_or_made_to_pass(dataset):
    dataset[-1][9]['T_color_board'][0][3] += .08
    fit = fit_dataset(dataset)
    assert not fit['training_passed']
    assert len(fit['training_metrics']['samples']) == 25


def test_five_degree_bound_is_not_silently_expanded(dataset):
    board, K, D, C, X, delta, samples = dataset
    delta[2] = np.radians(7.)
    model = kinematics.HandKinematics(core.URDF)
    for s in samples:
        q = np.array([s['joints_rad'][j] for j in core.JOINTS])
        B = core.inverse(C) @ model.fk((q+delta)[None])[0] @ X
        s['T_color_board'] = B.tolist()
        s['corners_px'] = cv2.projectPoints(board.points, Rotation.from_matrix(B[:3, :3]).as_rotvec(),
                                           B[:3, 3], K, D)[0].reshape(-1, 2).tolist()
    fit = fit_dataset(dataset)
    assert not fit['training_passed']
    assert 'five-degree' in fit['refinement_rejection']


@pytest.mark.parametrize('problem', ['rank', 'nonconvergence'])
def test_optimizer_failures_do_not_produce_an_approved_fit(dataset, monkeypatch, problem):
    original = kinematics.least_squares
    def invalid(*args, **kwargs):
        result = original(*args, **kwargs)
        if problem == 'rank':
            result.jac[:, 12:] = 0
        else:
            result.success = False
        return result
    monkeypatch.setattr(kinematics, 'least_squares', invalid)
    fit = fit_dataset(dataset)
    assert not fit['training_passed'] and fit['refinement_rejection']


def test_good_training_fit_is_rejected_when_out_of_fold_predictions_fail(dataset, monkeypatch):
    original = kinematics.corrected_samples
    def bad_predictions(*args, **kwargs):
        samples = original(*args, **kwargs)
        for s in samples:
            s['T_base_hand'][0][3] += .1
        return samples
    monkeypatch.setattr(kinematics, 'corrected_samples', bad_predictions)
    fit = fit_dataset(dataset)
    assert core.passes(fit['training_metrics'])
    assert not fit['training_passed']
    assert not fit['internal_crossvalidation']['passed']
    assert 'predictive' in fit['refinement_rejection']


def test_missing_joint_excitation_is_rejected(dataset):
    board, K, D, C, X, delta, samples = dataset
    # First joint alone moves: camera/mount gauges cannot identify the offsets.
    model = kinematics.HandKinematics(core.URDF)
    for i, s in enumerate(samples):
        q = np.array([.03*i, .1, -.2, 1., .3, .2, .1])
        s['joints_rad'] = dict(zip(core.JOINTS, q.tolist()))
        s['T_base_hand'] = model.fk(q[None])[0].tolist()
        s['T_color_board'] = (core.inverse(C) @ model.fk((q+delta)[None])[0] @ X).tolist()
    with pytest.raises(ValueError, match='rotation diversity'):
        fit_dataset(dataset)


@pytest.mark.parametrize('problem', ['model_hash', 'first_joint', 'last_joint', 'too_large', 'nan', 'rule', 'schema'])
def test_correction_metadata_rejected_when_invalid(dataset, problem):
    fit = fit_dataset(dataset)
    correction = fit['kinematic_correction']
    if problem == 'model_hash':
        correction['model_sha256'] = 'wrong'
    elif problem in ('first_joint', 'last_joint'):
        correction['offsets_rad'][core.JOINTS[0 if problem == 'first_joint' else 6]] = .01
    elif problem in ('too_large', 'nan'):
        correction['offsets_rad'][core.JOINTS[2]] = .2 if problem == 'too_large' else float('nan')
    elif problem == 'rule':
        correction['coordinate_rule'] = 'q_model = q_saved - offsets_rad'
    else:
        correction['schema'] = 'unknown'
    with pytest.raises(ValueError):
        kinematics.corrected_samples(dataset[-1][:5], correction, core.URDF)


def test_corrected_mapping_retains_gripper_and_left_coordinates(dataset):
    correction = fit_dataset(dataset)['kinematic_correction']
    q = dict(dataset[-1][0]['joints_rad'], openarmx_left_joint1=.23, openarmx_right_finger_joint1=.04)
    converted = kinematics.corrected_joint_values(q, correction)
    assert converted['openarmx_left_joint1'] == q['openarmx_left_joint1']
    assert converted['openarmx_right_finger_joint1'] == q['openarmx_right_finger_joint1']
    for j in core.JOINTS:
        assert converted[j] - correction['offsets_rad'][j] == pytest.approx(q[j])


def test_export_roundtrip_validation_is_not_fitted(tmp_path, dataset):
    board = dataset[0]
    fits = []
    for bad in (False, True):
        session = make_session(tmp_path, dataset)
        if bad:
            for sample in session.data['samples'][:5]:
                sample['T_color_board'][0][3] += .1
        original = copy.deepcopy(session.data['samples'])
        fits.append(session.fit(board))
        output = session.validate(board)
        assert output['validated'] == (not bad)
        assert output['schema'] == 'openarmx-eye-to-hand-calibration-v2'
        assert output['requires_kinematic_correction']
        assert output['kinematic_correction'] == fits[-1]['kinematic_correction']
        assert not output['motion_authorized'] and not output['depth_mapping_verified']
        assert output['validation']['heldout_ids'] == [1, 2, 3, 4, 5]
        assert output['validation']['used_for_fitting'] is False
        assert session.data['samples'] == original
        loaded = load_session(session.path)
        assert loaded.validate(board)['validated'] == (not bad)
    for key in ('T_base_color', 'T_hand_board', 'kinematic_correction', 'input_sha256'):
        # Capture timestamps differ between sessions; the input signatures should too.
        if key != 'input_sha256':
            assert fits[0][key] == fits[1][key]


def test_failed_revalidation_invalidates_previous_export(tmp_path, dataset):
    s = make_session(tmp_path, dataset); board = dataset[0]
    s.fit(board); assert s.validate(board)['validated']
    s.data['samples'][0]['T_color_board'][0][3] += .15
    assert not s.validate(board)['validated']
    assert not json.loads((s.path / 'calibration.json').read_text())['validated']


@pytest.mark.parametrize('problem', ['training', 'camera', 'convention'])
def test_frozen_input_changes_invalidate_export(tmp_path, dataset, problem):
    s = make_session(tmp_path, dataset); board = dataset[0]
    s.fit(board); assert s.validate(board)['validated']
    if problem == 'training':
        s.data['samples'][5]['joints_rad'][core.JOINTS[2]] += .01
    elif problem == 'camera':
        s.data['camera']['color']['K'][0][0] += 1.
    else:
        s.data['encoder_convention'] = 'changed'
    with pytest.raises(ValueError, match='Frozen training inputs changed'):
        s.validate(board)
    assert not json.loads((s.path / 'calibration.json').read_text())['validated']


def test_reprocess_copy_preserves_original_and_refuses_overwrite(tmp_path, dataset):
    s = make_session(tmp_path, dataset)
    (s.path / 'training_report.json').write_text('{"training_passed": false}')
    before = {p: p.read_bytes() for p in s.path.rglob('*') if p.is_file()}
    target = tmp_path / 'reprocessed'
    copied = copy_session_for_solve(s.path, target)
    copied.fit(dataset[0]); assert copied.validate(dataset[0])['validated']
    assert copied.data['reprocessed_from']['session'] == str(s.path)
    assert (target / 'previous_training_report.json').read_bytes() == (s.path / 'training_report.json').read_bytes()
    assert copied.data['samples'] == s.data['samples']
    assert all(p.read_bytes() == data for p, data in before.items())
    assert not (s.path / 'calibration.json').exists()
    with pytest.raises(FileExistsError):
        copy_session_for_solve(s.path, target)
    with pytest.raises(ValueError, match='outside'):
        copy_session_for_solve(s.path, s.path / 'nested')
    # A source-resolving absolute image path must never be reused as an output
    # path, even though the source image itself passes containment/hash checks.
    s.data['samples'][0]['image'] = str(s.path / s.data['samples'][0]['image'])
    s.save()
    image_before = (s.path / 'images/0001.png').read_bytes()
    with pytest.raises(ValueError, match='escape'):
        copy_session_for_solve(s.path, tmp_path / 'escaped-output')
    assert (s.path / 'images/0001.png').read_bytes() == image_before


def test_production_solve_cli_offline_with_new_output(tmp_path, dataset, capsys):
    from camera_calibration.cli import main
    session = make_session(tmp_path, dataset)
    dest = tmp_path / 'cli-output'
    assert main(['solve', str(session.path), '--output', str(dest)]) == 0
    result = json.loads((dest / 'calibration.json').read_text())
    assert result['validated'] and result['requires_kinematic_correction']
    assert not (session.path / 'calibration.json').exists()
    assert 'PASS' in capsys.readouterr().out
    # Reloading a frozen fit must never refit the reserved validation data.
    assert main(['solve', str(dest)]) == 0
    assert main(['solve', str(session.path), '--output', str(dest)]) == 1
