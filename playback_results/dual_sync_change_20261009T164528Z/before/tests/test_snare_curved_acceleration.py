"""Reference-only curved snare acceleration checks; no robot I/O."""
import json
import math
import unittest
from unittest.mock import patch

import numpy as np
from numpy.polynomial import Polynomial

from snare_lab.motion import (AcceleratingDescent, build_method, _descent_fits,
                              _extrema, _accelerating_descent)
from snare_lab.config import ROOT, LEFT_POWERED_OVERRIDES, joint_rotation_limits
from strike_lab.engine import validate_curve


def at_depth(curve, target):
    lo, hi = 0., curve.duration
    for _ in range(55):
        mid = (lo + hi) / 2
        if curve.at(mid)[0] < target:
            lo = mid
        else:
            hi = mid
    return curve.at((lo + hi) / 2)


class CurvedAccelerationTests(unittest.TestCase):
    def setUp(self):
        tuning = json.loads((ROOT / 'config/experiment_hardware_tuned.json').read_text())
        self.parameters = dict(tuning['parameters']['powered'], **LEFT_POWERED_OVERRIDES)
        self.limits = joint_rotation_limits(0., -.75, .75, tuning['limits'])

    def method(self, degrees=10.):
        return build_method(0., 0., self.parameters, degrees, self.limits)

    def test_moderate_peak_increase_and_later_peak(self):
        method = self.method()
        curve, descent = method.curve, method.curve.descent
        self.assertIsInstance(descent, AcceleratingDescent)
        old = curve.previous_descent
        old_peak = _extrema(Polynomial(old.c).deriv() / old.duration)[1]
        self.assertAlmostEqual(descent.peak_speed / old_peak, 1.20)
        self.assertAlmostEqual(descent.peak_fraction, .70)
        self.assertAlmostEqual(math.degrees(descent.peak_speed), 130.134, places=3)
        self.assertAlmostEqual(curve.down, .14631, places=5)
        self.assertAlmostEqual(curve.up, .220)
        self.assertLess(descent.peak_speed, self.limits.velocity)

    def test_late_travel_is_faster_not_just_an_earlier_peak(self):
        curve = self.method().curve
        for fraction in (.7, .8, .9, .95):
            target = math.radians(10.) * fraction
            before = at_depth(curve.previous_descent, target)[1]
            after = at_depth(curve.descent, target)[1]
            self.assertGreater(after, before * 1.20)
            self.assertLess(after, before * 1.35)

    def test_acceleration_then_braking_with_continuous_joins(self):
        for degrees in (.5, 10., 11.5, 20., 40.):
            with self.subTest(degrees=degrees):
                curve = self.method(degrees).curve
                descent = curve.descent
                first, second = descent.legs
                self.assertGreaterEqual(_extrema(
                    Polynomial(first.c).deriv(2) / first.duration**2)[0], -1e-9)
                self.assertLessEqual(_extrema(
                    Polynomial(second.c).deriv(2) / second.duration**2)[1], 1e-9)
                np.testing.assert_allclose(first.at(first.duration), second.at(0.), atol=1e-9)
                np.testing.assert_allclose(second.at(second.duration),
                    curve.original.at(curve.previous_down), atol=1e-9)
                self.assertTrue(_descent_fits(descent, math.radians(degrees), self.limits))

    def test_return_position_velocity_acceleration_preserved(self):
        for degrees in (.5, 10., 11.5, 20., 40.):
            curve = self.method(degrees).curve
            self.assertEqual(curve.up, curve.original.duration - curve.previous_down)
            for elapsed in np.linspace(0., curve.up, 201):
                np.testing.assert_allclose(curve.at(curve.down + elapsed),
                    curve.original.at(curve.previous_down + elapsed), atol=1e-9)

    def test_depth_sweep_retains_limits_and_exact_endpoint(self):
        for degrees in np.linspace(.5, self.limits.hard_depth_deg, 41):
            method = self.method(float(degrees))
            curve = method.curve
            self.assertTrue(curve.curved_acceleration)
            self.assertTrue(_descent_fits(curve.descent, method.target, self.limits))
            validate_curve(method, self.limits)
            self.assertAlmostEqual(curve.at(curve.down)[0], method.target)
            self.assertAlmostEqual(curve.at(curve.down)[1], 0.)

    def test_unavailable_curve_keeps_previous_descent(self):
        with patch('snare_lab.motion._accelerating_descent', return_value=None):
            curve = self.method().curve
        self.assertFalse(curve.curved_acceleration)
        self.assertIs(curve.descent, curve.previous_descent)
        self.assertAlmostEqual(curve.down, .160)
        self.assertAlmostEqual(curve.up, .220)

    def test_execution_reuses_preflight_curve_search(self):
        _accelerating_descent.cache_clear()
        self.method()
        before = _accelerating_descent.cache_info()
        build_method(0., .2, self.parameters, 10., self.limits)
        after = _accelerating_descent.cache_info()
        self.assertEqual(after.misses, before.misses)
        self.assertEqual(after.hits, before.hits + 1)


if __name__ == '__main__':
    unittest.main()
