"""Real saved-data recovery and fail-closed regressions; no devices or processes."""
import copy
import json
import socket
import subprocess

import numpy as np
import pytest

from camera_calibration import bundle, core, kinematics
from camera_calibration.io import copy_session_for_solve, load_session
from safe_zone.geometry import Model

SOURCE = core.ROOT / 'calibration_sessions/20261024T211629Z-1227b7b1'
EXPORT = SOURCE.with_name(SOURCE.name + '-reprocessed')


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    def denied(*a, **kw):
        pytest.fail('Offline recovery attempted hardware/process access')
    monkeypatch.setattr(socket, 'socket', denied)
    monkeypatch.setattr(subprocess, 'Popen', denied)


@pytest.fixture(scope='module')
def saved():
    session = load_session(SOURCE)
    K, D = core.camera_parameters(session.data['camera'])
    return session.data, K, D, core.Board(SOURCE / 'target.json')


@pytest.fixture(scope='module')
def fit(saved):
    data, K, D, board = saved
    rows = copy.deepcopy([s for s in data['samples'] if s['phase'] == 'training'])
    before = copy.deepcopy(rows)
    result = bundle.solve(rows, K, D, board, SOURCE / 'model.urdf')
    assert rows == before
    return result


def test_all_35_training_poses_and_five_folds_pass_without_heldout(saved, fit):
    ids = {s['id'] for s in saved[0]['samples'] if s['phase'] == 'training'}
    assert len(ids) == 35 and set(fit['training_ids']) == ids
    assert fit['training_passed'] and core.passes(fit['training_metrics'])
    cv = fit['internal_crossvalidation']
    assert cv['passed'] and core.passes(cv['metrics'])
    assert not cv['reserved_validation_used']
    assert cv['maximum_allowed_ratio'] == .8
    seen = []
    for fold in cv['folds']:
        assert set(fold['training_ids']).isdisjoint(fold['test_ids'])
        assert set(fold['training_ids'] + fold['test_ids']) == ids
        seen.extend(fold['test_ids'])
    assert sorted(seen) == sorted(ids)
    assert fit['solver_policy'] == bundle.POLICY
    assert fit['objective'] == dict(position_scale_m=.005, rotation_scale_deg=1., corner_scale_px=1., loss='linear')
    assert abs(kinematics.correction_offsets(fit['kinematic_correction'])).max() < np.radians(4.95)


def test_reserved_validation_passes_using_independent_general_fk(saved, fit):
    data, K, D, board = saved
    heldout = [s for s in data['samples'] if s['phase'] == 'validation']
    before = copy.deepcopy(heldout)
    rows = bundle.evaluation_samples(heldout, fit, K, D, board)
    # Independent FK implementation, not the solver's batched FK.
    model = Model(SOURCE / 'model.urdf')
    for s in rows:
        q = kinematics.corrected_joint_values(s['joints_rad'], fit['kinematic_correction'])
        s['T_base_hand'] = model.transforms(q)[core.HAND].tolist()
    metrics = core.metrics(rows, np.array(fit['T_base_color']), np.array(fit['T_hand_board']), K, D, board.points)
    assert core.passes(metrics) and heldout == before
    expected = json.loads((EXPORT / 'calibration.json').read_text())['validation']['metrics']
    for key in ('translation_mm', 'rotation_deg', 'reprojection_px'):
        for stat in ('rms', 'maximum'):
            assert metrics[key][stat] == pytest.approx(expected[key][stat], abs=1e-5)


def test_image_refinement_is_per_image_and_preserves_raw_corners(saved):
    data, K, D, board = saved
    samples = data['samples']; before = copy.deepcopy(samples)
    rows, changes = bundle.refine_observations(samples, K, D, board)
    single, _ = bundle.refine_observations([samples[0]], K, D, board)
    assert single[0] == rows[0] and samples == before
    for old, new, change in zip(samples, rows, changes):
        assert old['corners_px'] == new['corners_px'] and old['corner_ids'] == new['corner_ids']
        assert old['joints_rad'] == new['joints_rad'] and old['T_base_hand'] == new['T_base_hand']
        assert change['refined_reprojection_rms_px'] <= change['original_reprojection_rms_px'] + 1e-6


@pytest.mark.parametrize('problem', ['validation', 'duplicate', 'count', 'bad_corner', 'nan'])
def test_invalid_inputs_cannot_be_approved(saved, problem):
    data, K, D, board = saved
    rows = copy.deepcopy([s for s in data['samples'] if s['phase'] == 'training'])
    if problem == 'validation': rows[0]['phase'] = 'validation'
    elif problem == 'duplicate': rows.append(rows[0])
    elif problem == 'count': rows = rows[:19]
    elif problem == 'bad_corner': rows[0]['corner_ids'][0] = 99
    else: rows[0]['corners_px'][0][0] = float('nan')
    with pytest.raises(ValueError):
        bundle.solve(rows, K, D, board, SOURCE / 'model.urdf')


def test_corrupted_image_cannot_be_hidden_by_fallback(saved):
    data, K, D, board = saved
    rows = copy.deepcopy([s for s in data['samples'] if s['phase'] == 'training'])
    for corner in rows[0]['corners_px']: corner[0] += 80.
    try:
        result = bundle.solve(rows, K, D, board, SOURCE / 'model.urdf')
    except ValueError:
        return
    assert not result['training_passed']
    assert len(result['training_metrics']['samples']) == 35


def test_good_training_cannot_bypass_bad_fold_predictions(saved, monkeypatch):
    data, K, D, board = saved
    original = kinematics.corrected_samples
    def bad_predictions(*args, **kwargs):
        rows = original(*args, **kwargs)
        for row in rows: row['T_base_hand'][0][3] += .1
        return rows
    monkeypatch.setattr(kinematics, 'corrected_samples', bad_predictions)
    fit = bundle.solve([s for s in data['samples'] if s['phase'] == 'training'], K, D, board, SOURCE/'model.urdf')
    assert core.passes(fit['training_metrics'])
    assert not fit['training_passed'] and not fit['internal_crossvalidation']['passed']


@pytest.mark.parametrize('problem', ['rank', 'bound', 'convergence'])
def test_bundle_keeps_optimizer_guards(saved, monkeypatch, problem):
    data, K, D, board = saved
    original = kinematics.least_squares
    def broken(*a, **kw):
        result = original(*a, **kw)
        if problem == 'rank': result.jac[:, 12:] = 0
        elif problem == 'bound': result.x[12] = kinematics.MAX_OFFSET_RAD
        else: result.success = False
        return result
    monkeypatch.setattr(kinematics, 'least_squares', broken)
    with pytest.raises(ValueError):
        bundle.solve([s for s in data['samples'] if s['phase'] == 'training'], K, D, board, SOURCE/'model.urdf')


def test_failed_original_refinement_reports_its_actual_errors(saved):
    data, K, D, board = saved
    rows = [s for s in data['samples'] if s['phase'] == 'training']
    fit = kinematics.solve_with_kinematics(rows, K, D, board, SOURCE/'model.urdf')
    assert not fit['training_passed']
    assert fit['training_metrics']['rotation_deg']['maximum'] == pytest.approx(2.1177, abs=.001)
    assert fit['baseline_training_metrics']['rotation_deg']['maximum'] > 2.5


def test_unknown_observation_policy_rejected_and_old_fit_untouched(saved):
    data, K, D, board = saved
    assert bundle.evaluation_samples(data['samples'], {}, K, D, board) is data['samples']
    with pytest.raises(ValueError, match='Unknown'):
        bundle.evaluation_samples(data['samples'], {'observation_policy': 'unrecognized'}, K, D, board)


@pytest.mark.parametrize('metric,rms,maximum', [('translation_mm', 5., 10.), ('rotation_deg', 1., 2.), ('reprojection_px', 4., 8.)])
def test_acceptance_thresholds_are_unchanged(metric, rms, maximum):
    good = {k: dict(rms=0., maximum=0.) for k in ('translation_mm', 'rotation_deg', 'reprojection_px')}
    good[metric] = dict(rms=rms, maximum=maximum)
    assert core.passes(good)
    for stat in ('rms', 'maximum'):
        bad = copy.deepcopy(good); bad[metric][stat] += 1e-6
        assert not core.passes(bad)


def test_actual_session_fit_export_reload_and_heldout_isolation(tmp_path, saved):
    before = {str(p.relative_to(SOURCE)): core.digest(p) for p in SOURCE.rglob('*') if p.is_file()}
    session = copy_session_for_solve(SOURCE, tmp_path / 'copy')
    observations = copy.deepcopy(session.data['samples'])
    expected = json.loads((EXPORT / 'calibration.json').read_text())
    # Bad reserved corners are never fed into fitting, and validation must fail.
    for row in session.data['samples']:
        if row['phase'] == 'validation':
            for corner in row['corners_px']: corner[0] += 80.
    fit = session.fit(saved[-1])
    for key in ('T_base_color', 'T_hand_board'):
        assert np.allclose(fit[key], expected[key], atol=1e-7, rtol=0)
    assert not session.validate(saved[-1])['validated']
    # Restore original reserved observations; no refit, same frozen calibration.
    session.data['samples'] = observations
    result = session.validate(saved[-1])
    assert result['validated'] and not result['motion_authorized']
    assert result['validation']['heldout_ids'] == [1, 2, 3, 4, 5]
    assert result['validation']['used_for_fitting'] is False
    assert load_session(session.path).validate(saved[-1])['validated']
    assert before == {str(p.relative_to(SOURCE)): core.digest(p) for p in SOURCE.rglob('*') if p.is_file()}
