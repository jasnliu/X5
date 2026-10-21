"""Physical run2 regression: left J4 -0.099 deg during its final center leg."""
import math
import unittest
from unittest.mock import patch
import numpy as np
from smooth_playback.trajectory import Geometry
from camera_playback.dual_recording import left_geometry
from camera_playback.center_feedback import check_center_feedback, check_center_path


class CenterFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.g = left_geometry(Geometry().model)

    def test_measured_center_boundary_overshoot_is_not_a_recording_fault(self):
        q = np.array([.004795147631,.002877088578,-.181256580453,
                      -.001726253147,-.578294804303,.040471046005,.001342641337])
        with self.assertRaisesRegex(ValueError,'Joint limit'):
            self.g.check(q, measured=True)
        original = q.copy()
        check_center_feedback(self.g,q)
        check_center_path(self.g,q,self.g.center)
        np.testing.assert_array_equal(q,original) # Never rewrites a measurement.

    def test_commanded_targets_stay_strictly_inside_joint_limits(self):
        q=self.g.center.copy();q[3]=math.radians(-.1)
        with self.assertRaisesRegex(ValueError,'Joint limit'):
            check_center_path(self.g,self.g.center,q)

    def test_over_point_two_degree_center_excursion_still_fails(self):
        q=self.g.center.copy();q[3]=math.radians(-.201)
        with self.assertRaisesRegex(ValueError,'Joint limit'):
            check_center_feedback(self.g,q)

    def test_other_joint_boundaries_are_not_widened(self):
        q=self.g.center.copy();q[5]=self.g.upper[5]+math.radians(.1)
        with self.assertRaisesRegex(ValueError,'Joint limit'):
            check_center_feedback(self.g,q)

    def test_actual_tcp_is_checked_not_just_clipped_proxy(self):
        q=self.g.center.copy();q[3]=math.radians(-.1)
        actual_tcp=self.g.tcp(q)
        original=self.g.zone.contains
        with patch.object(self.g.zone,'contains',side_effect=lambda p,t: False if np.array_equal(p,actual_tcp) else original(p,t)):
            with self.assertRaisesRegex(ValueError,'Measured center motion outside'):
                check_center_feedback(self.g,q)

    def test_nonfinite_or_invalid_feedback_rejected(self):
        for q in ([0.]*6,[math.nan]*7):
            with self.subTest(q=q),self.assertRaises(ValueError):check_center_feedback(self.g,q)
