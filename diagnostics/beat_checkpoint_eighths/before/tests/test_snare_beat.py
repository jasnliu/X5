"""Snare beat clock, exact shared reference, and ownership guards. NO devices."""
from collections import deque
from dataclasses import replace
import math
import random
import struct
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.snare_beat import (MeasureGrid, SnareController, SnareStatus,
    SNARE_DEGREES, validate_tempo)
from camera_playback import snare_workflow as workflow
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.mit_strike import Sample
from camera_playback.dual_recording import left_geometry, DEFAULT_LEFT_RECORDING
from camera_playback.recording_cache import load_cached_smooth_recording
from camera_playback.smooth_recording_worker import LeftRecordingMotors
from camera_playback.hybrid_strike import HybridStatus
from camera_playback import hybrid_workflow
from centering.motors import parameter, packet
from safe_zone.encoder import FRAME, joint_to_motor
from smooth_playback.trajectory import Geometry
from snare_lab.runner import load_left_tuning
from snare_lab.motion import build_method
from tests.test_left_center_hold import fake_bus


class Choices:
    def __init__(self, choices): self.choices, self.i = choices, 0
    def randrange(self, n):
        assert n == 4
        result = self.choices[self.i % len(self.choices)] - 1
        self.i += 1
        return result


class SnareBeatTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.g = left_geometry(Geometry().model)
        g = cls.g
        cls.recording = load_cached_smooth_recording(DEFAULT_LEFT_RECORDING,
            g.model, g.zone, g.lower, g.upper, g.center, .4, playback_speed=.8, side='left')
        cls.parameters, cls.rules, cls.limits = load_left_tuning(cls.recording, g.lower[5], g.upper[5])
        cls.anchor = -float(cls.recording.last_joints[5])
        cls.template = build_method(cls.anchor, 0., cls.parameters, SNARE_DEGREES, cls.limits)

    def controller(self, choices=(1, 2, 3, 4), initial=None):
        return SnareController(self.template, self.rules, self.limits, -.75, .75,
                               self.anchor if initial is None else initial, 0., Choices(choices))

    def sample(self, now, q=None, v=0.):
        return Sample(self.anchor if q is None else q, v, 0., 2, now)

    def prime(self, c):
        for i in range(60): c.update(self.sample(i*.002), i*.002, -self.anchor)

    def test_one_uniform_random_choice_per_measure_and_absolute_clock(self):
        g = MeasureGrid(10., .6, random.Random(221))
        counts = [0]*4
        for m in range(10000):
            self.assertEqual(g.measure, m)
            self.assertIn(g.beat, (1, 2, 3, 4))
            self.assertAlmostEqual(g.at, 10.+(4*m+g.beat-1)*.6)
            counts[g.beat-1] += 1
            g.advance()
        self.assertTrue(all(2300 < n < 2700 for n in counts))

    def test_preflight_is_exact_snare_sh_11_degree_reference(self):
        d = SimpleNamespace(app=SimpleNamespace(hybrid_enabled=True), g=self.g, recording=self.recording)
        workflow.preflight(d)
        self.assertAlmostEqual(d.snare_template.target, math.radians(11.))
        self.assertAlmostEqual(d.snare_template.curve.up, .220)
        self.assertTrue(d.snare_template.curve.curved_acceleration)
        for t in np.linspace(0, self.template.curve.duration, 501):
            np.testing.assert_array_equal(d.snare_template.curve.at(t), self.template.curve.at(t))

    def test_all_four_beats_and_adjacent_measures_preserve_full_return(self):
        # Worst boundary is beat 4 immediately followed by beat 1. No overlap,
        # no omitted bar, no change of depth/dynamics even at max beat BPM.
        for bpm in (20., 100., 120.):
            c = self.controller((4, 1, 2, 3))
            self.prime(c)
            c.start(1., 60/bpm)
            hits = []
            previous_release = None
            q, v = self.anchor, 0.
            end = 1.+8*4*60/bpm
            for i in range(60, math.ceil(end/.002)):
                now = i*.002
                command = c.update(self.sample(now, q, v), now, -self.anchor)
                q, v = command.position, command.velocity  # Ideal servo, not hardware evidence.
                if c.status.released_at != previous_release:
                    previous_release = c.status.released_at
                    hits.append((c.status.measure, c.status.beat, c.status.scheduled_at, now))
                if len(hits) == 8 and c.status.completed == 8:
                    break
            self.assertEqual([(m, b) for m, b, _, _ in hits],
                             list(enumerate((4, 1, 2, 3)*2, 1)))
            self.assertEqual(c.status.completed, 8)
            for m, b, target, release in hits:
                self.assertAlmostEqual(target, 1.+(4*(m-1)+b-1)*60/bpm)
                self.assertLess(abs(release+self.template.curve.down-target), .00201)

    def test_stop_finishes_existing_full_curve_and_cancels_future(self):
        c = self.controller((1,))
        self.prime(c); c.start(1., .6)
        q, v = self.anchor, 0.
        stopped = False
        for i in range(60, 2200):
            now = i*.002
            command = c.update(self.sample(now, q, v), now, -self.anchor)
            q, v = command.position, command.velocity
            if c.method is not None and not stopped:
                c.finish(); stopped = True
        self.assertTrue(stopped)
        self.assertEqual(c.status.completed, 1)
        self.assertFalse(c.status.moving or c.status.scheduled)
        self.assertAlmostEqual(q, self.anchor, places=5)

    def test_delayed_grid_does_not_catch_up(self):
        c = self.controller((1,)); self.prime(c); c.start(.1, .6)
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            c.update(self.sample(.2), .2, -self.anchor)
        self.assertIsNone(c.method)

    def test_no_hit_without_settled_anchor_or_fresh_feedback(self):
        c = self.controller((1,)); c.start(1., .6)
        with self.assertRaisesRegex(RuntimeError, 'settled'):
            c.update(self.sample(1-self.template.curve.down), 1-self.template.curve.down, -self.anchor)
        c = self.controller(); self.prime(c)
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            c.update(self.sample(0.), 1., -self.anchor)

    def test_wrong_path_target_and_excessive_velocity_rejected(self):
        c = self.controller(); self.prime(c); c.start(1., .6)
        with self.assertRaisesRegex(RuntimeError, 'path moved'):
            c.update(self.sample(.2), .2, -self.anchor+.01)
        with self.assertRaisesRegex(RuntimeError, 'velocity'):
            c.update(self.sample(.2, v=99), .2, -self.anchor)

    def test_tempo_preflight_prevents_truncated_return(self):
        for bpm in (20., 100., 120.): validate_tempo(bpm, self.template, self.rules)
        for bpm in (121., 180., float('nan')):
            with self.assertRaises(ValueError): validate_tempo(bpm, self.template, self.rules)

    def test_reset_restarts_measure_one(self):
        c = self.controller(); self.prime(c); c.start(1., .6)
        c.grid.advance(); c.finish(); c.start(9., .6)
        self.assertEqual(c.grid.measure, 0)
        self.assertEqual(c.grid.epoch, 9.)

    def test_left_path_routes_only_j6_to_shared_goal(self):
        bus = fake_bus(); bus.active = bus.left_drive.active = True
        session = SimpleNamespace(status=SnareStatus(armed=True), goal=SimpleNamespace(value=0.))
        bus.left_drive.left_joint6_session = session
        q = self.recording.last_joints
        bus.left_drive.set_positions(q)
        self.assertAlmostEqual(session.goal.value, q[5])
        calls = bus.sockets['left'].sock.send.call_args_list
        self.assertEqual([FRAME.unpack(c.args[0])[0] & 255 for c in calls], [1,2,3,4,5,7])
        session.status = replace(session.status, moving=True)
        with self.assertRaisesRegex(RuntimeError, 'finish'):
            bus.left_drive.set_positions(q)

    def test_recording_worker_never_sends_csp_j6(self):
        bus = LeftRecordingMotors.__new__(LeftRecordingMotors)
        bus.control_side = 'left'; bus.j6_goal = SimpleNamespace(value=0.)
        bus.sockets = {'left': Mock()}
        bus._send('left', parameter(6, 0x7016, joint_to_motor('left', 6, .123)))
        self.assertAlmostEqual(bus.j6_goal.value, .123, places=6)
        bus.sockets['left'].send.assert_not_called()

    def test_off_center_disable_refused_and_centered_shutdown_allowed(self):
        bus = fake_bus(); bus.require_center_before_relax = True
        bus.active = bus.left_drive.active = True
        now = time.monotonic()
        bus.states.update({('left', i): (q, 2, now) for i, q in enumerate([*LEFT_CENTER, LEFT_GRIPPER_TARGET], 1)})
        bus.states['left', 6] = (.2, 2, now)
        with self.assertRaisesRegex(RuntimeError, 'center'):
            bus.left_drive.send_control(packet(4, 6))
        bus.sockets['left'].sock.send.assert_not_called()
        bus.states['left', 6] = (0., 2, now)
        bus.snare_center_history = deque((now-d, LEFT_CENTER.copy()) for d in (.64, .3, 0.))
        bus.left_drive.send_control(packet(4, 6))
        bus.sockets['left'].sock.send.assert_called_once()

    def test_hybrid_stop_waits_for_snare_full_return(self):
        from tests.test_hybrid_workflow import app_fixture
        a = app_fixture()
        left = Mock(finished=False)
        a.bus.left_joint6_session = left
        a.hybrid_stopping = True; a.hybrid_stop_request = 1
        a.hybrid_session.status = HybridStatus(request_id=1, ready=True,
            sample=Sample(0., 0., 0., 2, 1.), at=1.)
        with patch.object(hybrid_workflow, 'close') as close:
            hybrid_workflow.tick(a, 1.)
            close.assert_not_called()
        workflow.finish(a); left.finish.assert_called_once()


if __name__ == '__main__': unittest.main()
