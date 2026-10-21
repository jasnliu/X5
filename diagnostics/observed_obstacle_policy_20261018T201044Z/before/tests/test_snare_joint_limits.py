"""Joint-derived snare bounds; no physical devices are opened."""
import contextlib
import io
import math
import socket
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from snare_lab.config import joint_rotation_limits, validate_joint_depth
from snare_lab import runner
from strike_lab.config import Limits, Rules, StrikeGoal
from strike_lab.engine import limit_command, validate_curve
from strike_lab.methods import METHODS
from snare_lab.motion import ScaledCurve, FastDownCurve, build_method, _descent_fits
from camera_playback.mit_strike import Command, Sample


def forbid_can(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden in snare limit tests')


sys.addaudithook(forbid_can)


class SnareJointLimitTests(unittest.TestCase):
    def recording(self, anchor=0.):
        joints = np.zeros(7)
        joints[5] = anchor
        return SimpleNamespace(last_joints=joints)

    def test_maximum_tracks_selected_recording_anchor(self):
        for anchor in (-.5, 0., .5, .74):
            limits = joint_rotation_limits(anchor, -.75, .75, {'hard_depth_deg': 12.})
            self.assertAlmostEqual(limits.hard_depth_deg, math.degrees(.75-anchor))

    def test_no_fixed_12_15_or_30_degree_cap(self):
        for degrees in (12., 15., 30., 40., math.degrees(.75)):
            validate_joint_depth(degrees, 0., -.75, .75)
        with self.assertRaisesRegex(ValueError, 'exceeds left J6 rotation'):
            validate_joint_depth(math.degrees(.75)+.0001, 0., -.75, .75)

    def test_joint_limit_not_full_joint_span_is_used(self):
        validate_joint_depth(1., .7, -.75, .75)
        with self.assertRaises(ValueError):
            validate_joint_depth(5., .7, -.75, .75)

    def test_invalid_anchors_and_magnitudes_rejected(self):
        for anchor in (-.751, .75, .751, math.nan, math.inf):
            with self.subTest(anchor=anchor), self.assertRaises(ValueError):
                joint_rotation_limits(anchor, -.75, .75, {})
        for degrees in (-1., 0., .1, math.nan, math.inf):
            with self.subTest(degrees=degrees), self.assertRaises(ValueError):
                validate_joint_depth(degrees, 0., -.75, .75)

    def test_dynamic_limits_still_validated(self):
        for values in ({'torque': 13.}, {'velocity': 11.}, {'hz': 0.},
                       {'acceleration': math.nan}, {'jerk': -1.}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                joint_rotation_limits(0., -.75, .75, values)

    def test_ride_depth_guard_is_unchanged(self):
        self.assertEqual(Limits().hard_depth_deg, 12.)
        with self.assertRaises(ValueError):
            Limits(hard_depth_deg=40.)
        with self.assertRaises(ValueError):
            StrikeGoal(12.).validate(Rules(), Limits())

    def test_left_tuning_preserves_dynamics_but_derives_depth(self):
        parameters, _, limits = runner.load_left_tuning(self.recording(), -.75, .75)
        self.assertAlmostEqual(limits.hard_depth_deg, math.degrees(.75))
        self.assertEqual(parameters['load_torque'], 0.)
        self.assertEqual(parameters['down_time'], .2)
        self.assertEqual(limits.torque, Limits().torque)
        self.assertEqual(limits.velocity, Limits().velocity)

    def test_internal_corridor_maps_to_positive_j6_limit(self):
        for anchor in (-.74, 0., .7):
            limits = joint_rotation_limits(anchor, -.75, .75, {})
            displayed, internal, lo, hi = runner.strike_corridor(
                self.recording(anchor), limits, -.75, .75)
            self.assertEqual(displayed, anchor)
            self.assertEqual(internal, -anchor)
            self.assertEqual(lo, -.75)
            self.assertLessEqual(hi, .75)
            self.assertGreaterEqual(hi, internal)

    def test_command_limiter_accepts_above_12_but_not_beyond_joint(self):
        limits = joint_rotation_limits(0., -.75, .75, {})
        sample = Sample(-.6, 0., 0., 2, 1.)
        command = Command(-.6, 0., 1., 1., 0.)
        bounded, _, _ = limit_command(command, sample, 1., 0., limits, None, 0.)
        self.assertEqual(bounded.position, -.6)
        with self.assertRaisesRegex(RuntimeError, 'outside experiment corridor'):
            limit_command(Command(-.76, 0., 1., 1.), sample, 1., 0., limits, None, 0.)

    def test_measured_joint_and_return_corridor_enforced(self):
        for position in (-.75, -.5, 0., .05):
            runner.check_strike_position(SimpleNamespace(position=position), -.75, .05)
        for position in (-.7501, .0501, math.nan):
            with self.assertRaises(RuntimeError):
                runner.check_strike_position(SimpleNamespace(position=position), -.75, .05)

    def test_both_modes_reject_invalid_depth_before_device_or_sim_creation(self):
        geometry = SimpleNamespace(lower=np.full(7, -.75), upper=np.full(7, .75),
                                   check_line=Mock())
        args = SimpleNamespace(recording='unused.json', degrees=44., fast=True)
        with patch.object(runner, 'load_geometry', return_value=(None, None, geometry.lower, geometry.upper)), \
                patch.object(runner, 'left_geometry', return_value=geometry), \
                patch.object(runner, 'load_recording', return_value=self.recording()), \
                patch.object(runner.Motors, '__init__') as open_motor, \
                patch.object(runner, 'SimBackend') as sim:
            for run in (runner.run_hardware, runner.run_simulate):
                with self.assertRaisesRegex(ValueError, 'exceeds left J6 rotation'):
                    run(args)
            open_motor.assert_not_called()
            sim.assert_not_called()
            geometry.check_line.assert_not_called()

    def test_path_and_curve_preflights_are_retained(self):
        geometry = SimpleNamespace(lower=np.full(7, -.75), upper=np.full(7, .75),
                                   check_line=Mock())
        with contextlib.redirect_stdout(io.StringIO()):
            _, _, limits = runner.prepare_strike(geometry, self.recording(), 12.)
        self.assertGreater(limits.hard_depth_deg, 30.)
        geometry.check_line.assert_called_once()
        # Large joint-valid strokes are slowed rather than raising dynamic limits.
        with contextlib.redirect_stdout(io.StringIO()):
            runner.prepare_strike(geometry, self.recording(), 40.)
        geometry.check_line.side_effect = ValueError('Outside taught zone')
        with self.assertRaisesRegex(ValueError, 'Outside taught zone'):
            runner.prepare_strike(geometry, self.recording(), 12.)

    def test_large_strokes_are_retimed_with_depth_and_dynamics_preserved(self):
        parameters, _, limits = runner.load_left_tuning(self.recording(), -.75, .75)
        for degrees in (12.5, 20., 40., math.degrees(.75)):
            with self.subTest(degrees=degrees):
                method = build_method(0., 0., parameters, degrees, limits)
                self.assertIsInstance(method.curve, FastDownCurve)
                self.assertIsInstance(method.curve.original, ScaledCurve)
                validate_curve(method, limits)
                bottom_time = method.curve.down
                self.assertAlmostEqual(method.curve.at(bottom_time)[0], math.radians(degrees))
                self.assertEqual(method.curve.at(method.curve.duration), (0., 0., 0.))
        # The former rejection really was dynamic, not a changed joint check.
        raw = METHODS['powered'].create(0., 0., parameters, target=math.radians(12.5))
        with self.assertRaisesRegex(ValueError, 'speed/acceleration'):
            validate_curve(raw, limits)

    def test_default_ten_degree_descent_faster_return_unchanged(self):
        parameters, _, limits = runner.load_left_tuning(self.recording(), -.75, .75)
        actual = build_method(0., .1, parameters, 10., limits)
        original = METHODS['powered'].create(0., .1, parameters, target=math.radians(10.))
        self.assertLess(actual.curve.down, .160)
        self.assertAlmostEqual(actual.curve.previous_fast_down, .160)
        self.assertAlmostEqual(actual.curve.up, .220)
        self.assertAlmostEqual(actual.curve.duration, actual.curve.down + .220)
        for t in np.linspace(0., .220, 101):
            np.testing.assert_allclose(actual.curve.at(actual.curve.down+t),
                                       original.curve.at(.2+t), atol=1e-10)

    def test_return_unchanged_at_all_depths_and_descent_limits_enforced(self):
        parameters, _, limits = runner.load_left_tuning(self.recording(), -.75, .75)
        for degrees in (.5, 10., 11.5, 12.5, 20., 40., math.degrees(.75)):
            with self.subTest(degrees=degrees):
                method = build_method(0., 0., parameters, degrees, limits)
                curve = method.curve
                self.assertLess(curve.down, curve.previous_down)
                self.assertTrue(_descent_fits(curve.descent, method.target, limits))
                self.assertEqual(curve.up, curve.original.duration-curve.previous_down)
                for t in np.linspace(0., curve.up, 101):
                    np.testing.assert_allclose(curve.at(curve.down+t),
                        curve.original.at(curve.previous_down+t), atol=1e-9)
                np.testing.assert_allclose(curve.descent.at(curve.down),
                    curve.original.at(curve.previous_down), atol=1e-9)


if __name__ == '__main__':
    unittest.main()
