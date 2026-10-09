"""Detected ride depth +0.5 degrees in hardware swing, never the search."""
from dataclasses import replace
import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App, SWING_EVENTS
from camera_playback.hybrid_strike import HybridController, load_tuning
from camera_playback import hybrid_workflow as hw
from test_hybrid_workflow import app_fixture


class RideBoostTests(unittest.TestCase):
    def fixture(self, detected=10, corridor=13., hard_depth=13.):
        a=app_fixture()
        p,r,l=load_tuning()
        a.hybrid_session.controller=HybridController(0.,-math.radians(corridor),.01,
            p,r,replace(l,hard_depth_deg=hard_depth))
        a.strike_index=detected-5
        a.strike_hit_pending=dict(degrees=detected,message='detected')
        a._program_failure=Mock()
        return a

    def test_10_becomes_10_point_5_after_successful_search(self):
        a=self.fixture()
        original=a.strike_plan.targets[a.strike_index].copy()
        a._finish_hit_strike_attempt()
        self.assertEqual(a.continuous_strike_degrees,10.5)
        self.assertAlmostEqual(a.continuous_strike_target[6],-math.radians(10.5))
        np.testing.assert_array_equal(a.continuous_strike_target[:6],a.strike_plan.anchor_joints[:6])
        a.hybrid_session.request.assert_called_once_with('swing',math.radians(10.5),SWING_EVENTS)
        np.testing.assert_array_equal(a.strike_plan.targets[a.strike_index],original)
        self.assertEqual(a._current_strike_degrees(),10)
        a._program_failure.assert_not_called()

    def test_boost_is_absolute_from_anchor_never_accumulates(self):
        a=self.fixture(11)
        first,q=hw.boosted_swing_target(a,11)
        second,q2=hw.boosted_swing_target(a,11)
        self.assertEqual(first,11.5);self.assertEqual(second,11.5)
        np.testing.assert_array_equal(q,q2)
        np.testing.assert_array_equal(a.strike_plan.anchor_joints,np.zeros(7))

    def test_12_point_5_uses_existing_13_degree_corridor(self):
        a=self.fixture(12)
        a._finish_hit_strike_attempt()
        self.assertEqual(a.continuous_strike_degrees,12.5)
        a.hybrid_session.request.assert_called_once_with('swing',math.radians(12.5),SWING_EVENTS)
        self.assertEqual(a.hybrid_session.controller.limits.hard_depth_deg,13.)

    def test_geometric_corridor_blocks_boost_before_hihat_or_swing_start(self):
        a=self.fixture(10,corridor=10.)
        a._finish_hit_strike_attempt()
        a._program_failure.assert_called_once()
        self.assertIn('boost is unsafe',a._program_failure.call_args.args[0])
        self.assertFalse(a.continuous_strike_active)
        a.hihat.start_sequence.assert_not_called()
        a.hybrid_session.request.assert_not_called()

    def test_hard_limit_is_not_widened_for_boost(self):
        a=self.fixture(12,hard_depth=12.6)
        a._finish_hit_strike_attempt()
        a._program_failure.assert_called_once()
        a.hihat.start_sequence.assert_not_called()
        a.hybrid_session.request.assert_not_called()

    def test_nonhybrid_path_does_not_receive_boost(self):
        a=self.fixture();a.hybrid_enabled=False
        a._begin_continuous_striking=Mock()
        a._finish_hit_strike_attempt()
        degrees,target=a._begin_continuous_striking.call_args.args
        self.assertEqual(degrees,10)
        self.assertAlmostEqual(target[6],-math.radians(10))

    def test_bad_depth_values_are_rejected_without_starting_hihat(self):
        for degrees in (True,np.nan,np.inf,-np.inf,'10.5'):
            a=self.fixture()
            a._begin_continuous_striking(degrees,np.zeros(7))
            a._program_failure.assert_called_once()
            a.hihat.start_sequence.assert_not_called()

    def test_reported_minimum_does_not_truncate_fractional_command(self):
        a=self.fixture()
        a.continuous_j7_measurement_strike_count=1
        a.continuous_j7_measurement_reached=True
        a.continuous_j7_lowest_encoder_rad=-math.radians(10.5)
        a.continuous_j7_measurement_anchor_rad=0.
        a.continuous_j7_measurement_label='beat 2'
        a.continuous_strike_degrees=10.5
        a._sample_continuous_j7_minimum=Mock()
        # Fixture mocks the inherited logger; invoke the actual method.
        with patch('builtins.print') as output:App._log_continuous_j7_minimum(a)
        self.assertIn('commanded: 10.5 deg',output.call_args.args[0])


if __name__=='__main__':unittest.main()
