"""Collection tests never open physical CAN or microphone devices."""
import math
import errno
import socket
import sys
import threading
import unittest
from unittest.mock import Mock, patch
import numpy as np

from x5_collection.hybrid import FiniteController, prepare_powered
from x5_collection.plan import make_plan, DEPTHS, CSV_FIELDS
from camera_playback.hybrid_strike import load_tuning
from camera_playback.mit_strike import Sample
from strike_lab.backends import SimBackend
from strike_lab.config import Plant
from strike_lab.engine import limit_command
from safe_zone.encoder import FRAME


def forbid_can(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.AF_CAN:
        raise AssertionError('Hardware forbidden in collection tests')


sys.addaudithook(forbid_can)


class CollectionTests(unittest.TestCase):
    def test_outside_start_only_allows_small_inward_center_recovery(self):
        from x5_collection.collect import Collector
        from smooth_playback.trajectory import Geometry
        c = Collector.__new__(Collector)
        c.g = Geometry(); c.event = Mock()
        q = np.array([.0247429618, .0067132067, .0124673838, .0500613413,
                      -.5386780036, -.1793385214, .3983808652])
        c.check_startup_path(q)
        self.assertTrue(c.startup_recovery)
        with patch.object(c, 'zone_excess', return_value=.0021):
            with self.assertRaisesRegex(RuntimeError, 'too far outside'):
                c.check_startup_path(q)
        q[0] = c.g.upper[0]+.01
        with self.assertRaisesRegex(RuntimeError, 'joint limits'):
            c.check_startup_path(q)

    def test_center_only_mode_entry_refuses_before_opening_can(self):
        from x5_collection.transport import CollectionMotors
        bus = CollectionMotors.__new__(CollectionMotors)
        bus.center_evidence = Mock(side_effect=RuntimeError('not at center'))
        with patch('x5_collection.transport.Joint7Channel') as channel:
            with self.assertRaisesRegex(RuntimeError, 'not at center'):
                bus.enter_mit_at_center()
            channel.assert_not_called()

    def test_mit_recording_reference_is_bounded(self):
        from camera_playback.mit_center_return import MitCenterReturn
        p, r, l = load_tuning()
        b = SimBackend(Plant(inertia=p['inertia'], gravity=p['load_torque'], friction=p['friction']))
        b.prepare(threading.Event(), lambda _: None)
        b.q = 1.4
        c = MitCenterReturn(1.4, 1.4, p['load_torque'], b.now())
        worst = 0.
        for i in range(2000):
            phase = min(1., i*.005/4.8)
            target = 1.4-.8*(10*phase**3-15*phase**4+6*phase**5)
            s = b.receive()
            c.goal = target
            command = c.update(s.position, b.now())
            self.assertLessEqual(abs(command.velocity), .35)
            self.assertLessEqual(abs(command.position-s.position), .04+1e-8)
            worst = max(worst, abs(s.position-target))
            b.send(command); b.advance(.005)
        self.assertLess(worst, math.radians(8))
        self.assertLess(abs(b.q-.6), math.radians(.05))

    def test_balanced_bounded_plan(self):
        plan = make_plan()
        self.assertEqual(len(plan), 30)
        self.assertEqual(sum(r['strikes'] for r in plan), 40)
        for depth in DEPTHS:
            self.assertEqual(sum(r['depth_deg'] == depth and r['strikes'] == 1 for r in plan), 4)
            self.assertEqual(sum(r['depth_deg'] == depth and r['strikes'] == 2 for r in plan), 2)
        self.assertTrue(all(r['ringdown_s'] >= 6 for r in plan))
        self.assertEqual(CSV_FIELDS, ('recording_file', 'hit_time_seconds', 'label'))

    def test_finite_controller_all_depths_never_third_hit(self):
        for depth in DEPTHS:
            for count in (1, 2):
                with self.subTest(depth=depth, count=count):
                    p, r, l = load_tuning()
                    b = SimBackend(Plant(inertia=p['inertia'], gravity=p['load_torque'], friction=p['friction']))
                    anchor, bias = b.prepare(threading.Event(), lambda _: None)
                    c = FiniteController(anchor, anchor-math.radians(12), anchor+math.radians(.6), p, r, l, bias)
                    last, torque, started = None, bias, False
                    releases = []
                    for i in range(2500):
                        s, now = b.receive(), b.now()
                        previous = c.status.released_at
                        command = c.update(s, now)
                        command, torque, _ = limit_command(command, s, now, anchor, l, last, torque)
                        self.assertGreaterEqual(command.position, c.lower)
                        b.send(command); last = now; b.advance(.002)
                        if c.status.released_at != previous:
                            releases.append(now)
                        if not started and c.status.ready:
                            c.request_block(math.radians(depth), count); started = True
                    self.assertEqual(len(releases), count)
                    self.assertEqual(c.status.count, count)
                    self.assertTrue(c.status.ready)
                    self.assertFalse(c.status.swing)
                    self.assertEqual(c.status.partial_returns, count-1)
                    self.assertEqual(c.parameters, p)
                    self.assertLess(c.status.peak_drop, math.radians(12))
                    if count == 2:
                        self.assertTrue(.15 < releases[1]-releases[0] < .3)

    def test_disallowed_depths_and_counts(self):
        p, r, l = load_tuning()
        c = FiniteController(.6, .6-math.radians(12), .61, p, r, l)
        for depth in (0, 9.99, 12.01, float('nan')):
            with self.assertRaises(ValueError):
                c.validate_depth(math.radians(depth))
        for count in (0, 3, 40):
            with self.assertRaises(ValueError):
                c.request_block(math.radians(11), count)

    def test_measured_corridor_still_aborts(self):
        p, r, l = load_tuning()
        c = FiniteController(.6, .6-math.radians(12), .61, p, r, l)
        with self.assertRaisesRegex(RuntimeError, 'corridor'):
            c.update(Sample(.6-math.radians(12.01), 0, 0, 2, 1.), 1.)

    def test_powered_transfer_never_disables_or_enables(self):
        channel = Mock()
        channel.sample = Sample(.6, 0., .7, 2, 10.)
        channel.mode = 0; channel.mode_at = 10.
        with patch('x5_collection.hybrid.time.monotonic', return_value=10.):
            bias = prepare_powered(channel, .6, threading.Event(), lambda s: None)
        kinds = [(FRAME.unpack(c.args[0])[0] >> 24) & 31 for c in channel.send.call_args_list]
        self.assertNotIn(4, kinds)
        self.assertNotIn(3, kinds)
        self.assertIn(18, kinds)
        self.assertEqual(bias, .7)

    def test_stationary_setup_retries_transient_queue_pressure(self):
        channel = Mock()
        channel.sample = Sample(.6, 0., .7, 2, 10.)
        channel.mode = 0; channel.mode_at = 10.
        channel.send.side_effect = [OSError(errno.ENOBUFS, 'queue full'), None, None, None, None]
        with patch('x5_collection.hybrid.time.monotonic', return_value=10.), patch('x5_collection.hybrid.time.sleep'):
            prepare_powered(channel, .6, threading.Event(), lambda s: None)
        self.assertEqual(channel.send.call_args_list[0], channel.send.call_args_list[1])


if __name__ == '__main__':
    unittest.main()
