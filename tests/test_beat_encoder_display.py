"""Both beat interfaces display cached feedback only; no device or GUI needed."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from camera_playback.app import App
from camera_playback.generated_app import GeneratedApp
from goal_motion.app import App as JointGoalApp
from safe_zone.gripper_feedback import motor8_feedback, gripper_feedback_text


@pytest.fixture(params=[App, GeneratedApp], ids=['legacy', 'generated'])
def app(request):
    result = request.param.__new__(request.param)
    result.side = 'right'
    result.q = {f'openarmx_{side}_joint{i}': math.radians(value)
                for side, values in (
                    ('right', [1.1, 2.2, -3.3, 4.4, -5.5, 6.6, -7.7]),
                    ('left', [-12.3, 4.5, 67.8, -9., -45., 0., 10.]))
                for i, value in enumerate(values, 1)}
    # Any attempt to query or command a transport must fail: display uses q.
    result.bus = SimpleNamespace(motor8_feedback={
        'right': motor8_feedback(33411), 'left': motor8_feedback(35774)},
        poll=Mock(side_effect=AssertionError('Display cannot poll devices')),
        positions=Mock(side_effect=AssertionError('Display cannot query devices')))
    return result


def test_both_arms_all_seven_joints_and_raw_grippers(app):
    text = app.encoder_text()
    # Preserve the existing right-hand block byte-for-byte.
    assert text.startswith(JointGoalApp.encoder_text(app) + '\nLeft encoder degrees:\n')
    for side in ('right', 'left'):
        heading = side.capitalize() + ' encoder degrees:\n'
        row = text.split(heading, 1)[1].splitlines()[0]
        expected = '  '.join(f'{i}: {math.degrees(app.q[f"openarmx_{side}_joint{i}"]):6.1f}'
                             for i in range(1, 8))
        assert row == expected
        assert gripper_feedback_text(app.bus.motor8_feedback[side]) in text
    app.bus.poll.assert_not_called()
    app.bus.positions.assert_not_called()


def test_left_snapshot_updates_without_changing_right_or_q(app):
    right_before = JointGoalApp.encoder_text(app)
    old = app.encoder_text()
    app.q['openarmx_left_joint6'] = math.radians(-22.5)
    expected_q = app.q.copy()
    new = app.encoder_text()
    assert new != old and '6:  -22.5' in new
    assert new.startswith(right_before + '\nLeft encoder degrees:\n')
    assert app.q == expected_q
    app.bus.poll.assert_not_called()


def test_preview_without_raw_gripper_feedback(app):
    app.bus = None
    text = app.encoder_text()
    assert 'Right encoder degrees:' in text and 'Left encoder degrees:' in text
    assert text.count('raw feedback unavailable') == 2
