"""Offline hybrid method, swing, and actual spawned worker/IPC checks."""
from dataclasses import replace
import math
import os
import socket
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.hybrid_strike import (
    HybridController, HybridSession, MIN_REBOUND, load_tuning,
)
from camera_playback.mit_strike import Command, Sample
from safe_zone.encoder import FRAME
from strike_lab.backends import SimBackend
from strike_lab.config import Plant
from strike_lab.engine import Engine, limit_command
from strike_lab.methods.hybrid import DEFINITION


def forbid_can(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden in hybrid tests')


sys.addaudithook(forbid_can)
EVENTS = (('beat 1', True, .6), ('beat 2', True, .4), ('extra after beat 2', False, .2),
          ('beat 3', True, .6), ('beat 4', True, .4), ('extra after beat 4', False, .2))


class Harness:
    def __init__(self, degrees=10, hard_depth=12., **plant_changes):
        self.p, self.rules, self.limits = load_tuning()
        self.limits = replace(self.limits, hard_depth_deg=hard_depth)
        plant = Plant(inertia=self.p['inertia'], gravity=self.p['load_torque'], friction=self.p['friction'])
        self.backend = SimBackend(replace(plant, **plant_changes))
        anchor, bias = self.backend.prepare(threading.Event(), lambda _: None)
        self.c = HybridController(anchor, anchor-math.radians(hard_depth), anchor+math.radians(.6),
                                  self.p, self.rules, self.limits, bias)
        self.last_send, self.last_torque = None, bias
        self.c.request_search(math.radians(degrees))

    def tick(self):
        b, c = self.backend, self.c
        s, now = b.receive(), b.now()
        command = c.update(s, now)
        bounded, self.last_torque, _ = limit_command(
            command, s, now, c.anchor, self.limits, self.last_send, self.last_torque)
        b.send(bounded)
        self.last_send = now
        b.advance(.002)
        return command, s, now

    def until(self, predicate, seconds=4):
        for _ in range(int(seconds/.002)):
            result = self.tick()
            if predicate(self.c.status):
                return result
        raise AssertionError(('Synthetic controller timed out', self.c.status))


class HybridTests(unittest.TestCase):
    def test_beat_can_search_and_swing_at_12_without_changing_method_parameters(self):
        h = Harness(12, hard_depth=13.)
        h.until(lambda s: s.completed == 1)
        self.assertTrue(h.c.status.ready)
        self.assertEqual(h.p, load_tuning()[0])
        h.c.request_swing(math.radians(12), EVENTS)
        h.until(lambda s: s.count >= 16, 8.)
        self.assertIsNone(h.c.status.error)

    def test_half_degree_boost_survives_real_controller_swing_and_return(self):
        for detected in (10, 12):
            with self.subTest(detected=detected):
                h = Harness(detected, hard_depth=13.)
                h.until(lambda s: s.completed == 1)
                anchor = h.c.anchor
                depth = math.radians(detected + .5)
                h.c.request_swing(depth, EVENTS)
                h.until(lambda s: s.count >= 16, 8.)
                self.assertEqual(h.c.amount, depth)
                self.assertEqual(h.c.anchor, anchor)
                self.assertIsNone(h.c.status.error)
                self.assertLess(abs(math.degrees(h.c.status.closed_peak) - (detected + .5)), .3)
                h.c.stop_swing()
                h.until(lambda s: s.ready and not s.swing)
                self.assertAlmostEqual(h.c.anchor, anchor)

    def test_uses_effective_hardware_tuning_not_untuned_method_defaults(self):
        p, _, l = load_tuning()
        self.assertEqual((p['kick_torque'], p['kick_time'], p['return_time']), (.3, .04, .22))
        self.assertEqual(l.hz, 500.)
        self.assertNotEqual(p, DEFINITION.defaults)

    def test_search_commands_identical_to_experiment_hybrid_until_settling(self):
        for degrees in range(5, 12):
            with self.subTest(degrees=degrees):
                h = Harness(degrees)
                reference = None
                phases = set()
                for _ in range(2000):
                    command, sample, now = h.tick()
                    method = h.c.method
                    if method is not None and reference is None:
                        reference = DEFINITION.create(h.c.anchor, method.bias, h.p,
                                                      target=math.radians(degrees))
                        reference.start(method.started, [])
                    if reference is not None:
                        expected = reference.update(sample, now)
                        if not reference.done:
                            self.assertEqual(command, expected)
                            phases.add(reference.phase)
                    if h.c.status.completed:
                        break
                self.assertEqual(h.c.status.completed, 1)
                self.assertTrue(h.c.status.ready)
                self.assertTrue({'drive', 'coast', 'catch', 'withdrawal'} <= phases)
                self.assertLess(abs(math.degrees(h.c.status.peak_drop)-degrees), .3)

    def test_search_is_full_return_and_preserves_anchor_across_depths(self):
        h = Harness(5)
        anchor = h.c.anchor
        for degrees in range(5, 12):
            if degrees != 5:
                h.c.request_search(math.radians(degrees))
            h.until(lambda s: s.completed == degrees-4)
            self.assertTrue(h.c.status.ready)
            self.assertEqual(h.c.status.partial_returns, 0)
            self.assertEqual(h.c.anchor, anchor)
            self.assertLess(abs(h.backend.q-anchor), math.radians(.15))

    def test_continuous_swing_all_search_depths_partial_returns_and_main_beats(self):
        for degrees in range(5, 12):
            with self.subTest(degrees=degrees):
                h = Harness(degrees)
                h.until(lambda s: s.completed == 1)
                h.c.request_swing(math.radians(degrees), EVENTS)
                old_count = old_bottoms = old_main = 0
                scheduled, main_times, labels = [], [], []
                partials = 0
                for _ in range(10000):
                    prior_method = h.c.method
                    _, sample, now = h.tick()
                    s = h.c.status
                    if s.count > old_count:
                        if prior_method is not None:
                            self.assertGreaterEqual(sample.position-(h.c.anchor-math.radians(degrees)), MIN_REBOUND)
                            partials += 1
                        old_count = s.count
                    if s.bottoms > old_bottoms:
                        scheduled.append(s.scheduled_at if s.scheduled_at is not None else s.bottom_at)
                        labels.append(s.label)
                        self.assertLess(abs(s.last_lateness), .030)
                        old_bottoms = s.bottoms
                    if s.main_count > old_main:
                        main_times.append(s.main_at)
                        self.assertTrue(s.main_beat)
                        old_main = s.main_count
                    if s.bottoms >= 30:
                        break
                self.assertGreater(partials, 0)
                self.assertEqual(partials, s.partial_returns)
                self.assertGreater(s.closed_count, 0)
                self.assertLess(abs(math.degrees(s.closed_peak)-degrees), .3)
                self.assertEqual(labels[0], 'pickup (extra before beat 1)')
                intervals = [.2, .6, .4, .2, .6, .4]
                for i, (a, b) in enumerate(zip(scheduled, scheduled[1:])):
                    self.assertAlmostEqual(b-a, intervals[i % 6])
                for a, b in zip(main_times, main_times[1:]):
                    self.assertAlmostEqual(b-a, .6)

    def test_short_rebound_is_bounded_and_stop_joins_it_continuously(self):
        h = Harness(11)
        h.until(lambda s: s.completed == 1)
        h.c.request_swing(math.radians(11), EVENTS)
        h.until(lambda s: s.phase == 'withdrawal')
        curve = h.c.partial_curve
        self.assertIsNotNone(curve)
        samples = [curve.at(curve.duration*i/1000) for i in range(1001)]
        self.assertLessEqual(max(abs(q[1]) for q in samples), h.limits.velocity)
        self.assertLessEqual(max(abs(q[2]) for q in samples), h.limits.acceleration)
        self.assertLessEqual(max(abs(b[2]-a[2])/(curve.duration/1000)
                                 for a,b in zip(samples,samples[1:])), h.limits.jerk)
        elapsed = h.c.status.at-h.c.method.catch_at-h.c.method.catch.catch_time
        before = curve.at(elapsed)
        h.c.stop_swing()
        self.assertTrue(h.c.finish_after_partial)
        for a,b in zip(before, h.c.partial_curve.at(elapsed)):
            self.assertAlmostEqual(a,b)
        count = h.c.status.count
        h.until(lambda s:s.ready and not s.swing)
        self.assertEqual(h.c.status.count,count)

    def test_timing_identifies_heavier_load_without_changing_hybrid_method(self):
        for inertia in (.018, .024, .028):
            h = Harness(11, inertia=inertia)
            original_parameters = h.p.copy()
            h.until(lambda s: s.completed == 1)
            self.assertGreater(h.c.timing_inertia, original_parameters['inertia'])
            h.c.request_swing(math.radians(11), EVENTS)
            h.until(lambda s: s.bottoms >= 15, seconds=8.)
            self.assertGreater(h.c.status.partial_returns, 0)
            self.assertLess(abs(h.c.status.last_lateness), .03)
            self.assertEqual(h.c.parameters, original_parameters)

    def test_stop_at_every_phase_finishes_current_return_without_new_release(self):
        for phase in ('drive', 'coast', 'catch', 'withdrawal', 'settling', 'hold'):
            with self.subTest(phase=phase):
                h = Harness()
                h.until(lambda s: s.completed == 1)
                h.c.request_swing(math.radians(10), EVENTS)
                h.until(lambda s: s.count >= 2 and s.phase == phase)
                count = h.c.status.count
                h.c.stop_swing()
                h.until(lambda s: s.ready and s.phase == 'hold')
                self.assertEqual(h.c.status.count, count)
                self.assertFalse(h.c.status.swing)
                self.assertLess(abs(h.backend.q-h.c.anchor), math.radians(.15))

    def test_late_or_stale_feedback_never_launches_catchup(self):
        h = Harness()
        h.until(lambda s: s.phase == 'drive')
        with self.assertRaisesRegex(RuntimeError, 'feedback'):
            h.c.update(h.backend.receive(), h.backend.now()+.021)
        h = Harness()
        h.until(lambda s: s.completed == 1)
        h.c.request_swing(math.radians(10), EVENTS)
        h.until(lambda s: s.bottoms == 1)
        now = h.c.next_event_at+.081
        s = Sample(h.c.anchor, 0., .745, 2, now)
        count = h.c.status.count
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            h.c.update(s, now)
        self.assertEqual(h.c.status.count, count)

    def test_invalid_depths_and_corridor_are_rejected(self):
        h = Harness()
        for depth in (0., math.nan, -1., math.radians(12)):
            with self.assertRaises(ValueError):
                h.c.validate_depth(depth)
        with self.assertRaisesRegex(RuntimeError, 'corridor'):
            h.c.update(Sample(h.c.lower-.001, 0., .745, 2, 1.), 1.)

    def test_idle_feedback_must_be_primed_before_new_release(self):
        h = Harness(5)
        h.until(lambda s: s.completed == 1)
        h.c.request_search(math.radians(6))
        s = h.backend.receive()
        h.c.update(s, s.at+.010)  # Inside 20 ms abort age, outside 5 ms release age.
        self.assertEqual(h.c.status.count, 1)
        self.assertIsNone(h.c.method)
        fresh = replace(s, at=s.at+.012)
        h.c.update(fresh, fresh.at)
        self.assertEqual(h.c.status.count, 2)

    def test_shared_limiter_is_used_by_experiment_engine(self):
        h = Harness()
        e = Engine.__new__(Engine)
        e.anchor, e.limits = h.c.anchor, h.limits
        e.last_send, e.last_torque = 1., .5
        e.backend = Mock()
        e.backend.now.return_value = 1.002
        s = Sample(e.anchor, -.2, .5, 2, 1.002)
        cmd = Command(e.anchor-.1, -1, 60, 2.4, .745)
        expected = limit_command(cmd, s, 1.002, e.anchor, e.limits, 1., .5)
        self.assertEqual(e.transmit(cmd, s, 1.002), expected)
        e.backend.send.assert_called_once_with(expected[0])


class SyntheticChannel:
    """Protocol-level fake drive for real spawn/IPC. It cannot open a socket."""
    def __init__(self, interface):
        p, _, _ = load_tuning()
        self.plant = Plant(inertia=p['inertia'], gravity=p['load_torque'], friction=p['friction'])
        self.q = .5
        self.v = 0.
        self.command = Command(self.q, 0, 40, 1.8, self.plant.gravity)
        self.at = time.monotonic()
        self.sample = None
        self.mode = None
        self.mode_at = 0.
        self.state = 2
        self.closed = False

    def send(self, frame):
        assert not self.closed
        cid, _, data = FRAME.unpack(frame)
        kind = (cid >> 24) & 31
        if kind == 4:
            self.state = 0
        elif kind == 3:
            self.state = 2
        elif kind == 18 and data[:2] == b'\x05\x70':
            self.mode = data[4]
        elif kind == 17:
            self.mode_at = time.monotonic()
        elif kind == 1:
            self.receive()
            self.command = Command(
                -(int.from_bytes(data[:2], 'big')/65535*25.14-12.57),
                -(int.from_bytes(data[2:4], 'big')/65535*66-33),
                int.from_bytes(data[4:6], 'big')/65535*500,
                int.from_bytes(data[6:8], 'big')/65535*5,
                -(((cid >> 8) & 65535)/65535*28-14))

    def receive(self):
        now = time.monotonic()
        elapsed = now-self.at
        n = max(1, math.ceil(elapsed/.0005))
        dt = elapsed/n
        c, p = self.command, self.plant
        for _ in range(n):
            tau = c.kp*(c.position-self.q)+c.kd*(c.velocity-self.v)+c.torque
            if self.state == 2:
                self.v += (tau-p.gravity-p.friction*self.v)/p.inertia*dt
                self.q += self.v*dt
        self.at = now
        self.sample = Sample(self.q, self.v, tau, self.state, now)

    def wait(self, timeout):
        time.sleep(min(.002, max(0, timeout)))

    def close(self):
        self.closed = True


class WorkerTests(unittest.TestCase):
    def test_spawned_session_search_swing_finish_and_exclusive_ownership(self):
        bus = SimpleNamespace(control_side='right', active=True, right_joint7_session=None,
                              sockets={'right': Mock()})
        bus.sockets['right'].getsockname.return_value = ('synthetic-only',)
        session = HybridSession(bus, .5, .5-math.radians(12), .5+math.radians(.6),
                                channel_factory=SyntheticChannel)
        def wait_for(predicate, seconds=5):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                s = session.status
                self.assertIsNone(s.error, s.error)
                if predicate(s):
                    return s
                time.sleep(.005)
            self.fail(str(session.status))
        try:
            self.assertIs(bus.right_joint7_session, session)
            session.request('search', math.radians(7))
            wait_for(lambda s: s.completed == 1 and s.ready)
            session.request('swing', math.radians(7), EVENTS)
            wait_for(lambda s: s.bottoms >= 4)
            request = session.request('finish')
            s = wait_for(lambda s: s.request_id >= request and s.ready and not s.swing)
            self.assertGreater(s.partial_returns, 0)
            self.assertGreaterEqual(s.main_count, 2)
        finally:
            session.stop()
        self.assertIsNone(bus.right_joint7_session)
        self.assertFalse(session.process.is_alive())

    def test_default_transport_refused_in_offline_tests(self):
        with patch.dict(os.environ, STRIKE_LAB_OFFLINE_ONLY='1'):
            with self.assertRaisesRegex(RuntimeError, 'forbidden'):
                HybridSession(Mock(), .5, .2, .6)


if __name__ == '__main__':
    unittest.main()
