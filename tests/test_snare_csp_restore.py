"""Post-strike mode handoff and final disable confirmation, entirely mocked."""
import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from snare_lab import runner


class Clock:
    def __init__(self):
        self.now = 10.

    def sleep(self, seconds):
        self.now += seconds


class Bus:
    def __init__(self, clock):
        self.clock = clock
        self.states = {('left', i): (0., 2, 9.) for i in range(1, 9)}
        self.modes = {6: (0, 9.)}  # Stale RUNNING state from MIT, not CSP.
        self.poll_count = 0
        self.handshake_done = False
        self.advance_feedback = True
        self.force_stopped = False
        self.disable_requested = False
        self.query_count = 0
        self.prepare_calls = []

    def poll(self):
        self.poll_count += 1
        if self.advance_feedback and self.handshake_done:
            state = 0 if self.disable_requested or self.force_stopped else 2
            self.states = {('left', i): (0., state, self.clock.now) for i in range(1, 9)}
            self.modes[6] = (5, self.clock.now)

    def fresh(self):
        return True

    def _prepare_center_motor_confirmed(self, motor, target, current):
        self.prepare_calls.append((motor, target, current))
        # Remain stale RUNNING initially, then observe disabled, then CSP.
        yield
        self.states['left', 6] = (0., 0, self.clock.now)
        yield
        self.handshake_done = True
        yield

    def request_left_joint6_mode_readback(self):
        self.query_count += 1

    def relax(self):
        self.disable_requested = True


class SnareCspRestoreTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.bus = Bus(self.clock)
        self.stack = contextlib.ExitStack()
        self.stack.enter_context(patch.object(runner.time, 'monotonic', side_effect=lambda: self.clock.now))
        self.stack.enter_context(patch.object(runner.time, 'sleep', side_effect=self.clock.sleep))
        self.output = self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.addCleanup(self.stack.close)

    def test_old_running_sample_cannot_skip_handoff(self):
        runner._restore_strike_joint_csp(self.bus, .018)
        self.assertEqual(self.bus.prepare_calls, [(6, .018, runner.CURRENT[5])])
        self.assertGreater(self.bus.poll_count, 10)
        self.assertGreaterEqual(self.clock.now, 10.10)
        self.assertIn('restore confirmed', self.output.getvalue())
        self.assertNotIn('FAULT', self.output.getvalue())

    def test_failed_setup_write_is_not_swallowed(self):
        self.bus._prepare_center_motor_confirmed = Mock(side_effect=RuntimeError('CAN write failed'))
        with self.assertRaisesRegex(RuntimeError, 'CAN write failed'):
            runner._restore_strike_joint_csp(self.bus, .018)
        self.assertNotIn('restore confirmed', self.output.getvalue())

    def test_stale_running_and_stale_csp_do_not_confirm(self):
        self.bus.advance_feedback = False
        self.bus.modes[6] = (5, 9.)
        def setup(*args):
            yield
        self.bus._prepare_center_motor_confirmed = setup
        with self.assertRaisesRegex(RuntimeError, 'reverse playback blocked'):
            runner._restore_strike_joint_csp(self.bus, .018)

    def test_fresh_running_but_wrong_mode_does_not_confirm(self):
        real_poll = self.bus.poll
        def poll():
            real_poll()
            self.bus.modes[6] = (0, self.clock.now)
        self.bus.poll = poll
        with self.assertRaisesRegex(RuntimeError, 'reverse playback blocked'):
            runner._restore_strike_joint_csp(self.bus, .018)

    def test_stopped_after_enable_does_not_confirm(self):
        self.bus.force_stopped = True
        with self.assertRaisesRegex(RuntimeError, 'reverse playback blocked'):
            runner._restore_strike_joint_csp(self.bus, .018)

    def test_setup_timeout_is_bounded(self):
        def setup(*args):
            while True:
                yield
        self.bus._prepare_center_motor_confirmed = setup
        with self.assertRaisesRegex(RuntimeError, 'confirmation timed out'):
            runner._restore_strike_joint_csp(self.bus, .018)
        self.assertLess(self.clock.now, 15.1)

    def test_relax_waits_for_fresh_disabled_feedback(self):
        self.bus.handshake_done = True
        runner._relax_confirmed(self.bus)
        self.assertTrue(self.bus.disable_requested)
        self.assertTrue(all(s[1] == 0 and s[2] >= 10. for s in self.bus.states.values()))

    def test_cached_disabled_feedback_cannot_confirm_relax(self):
        self.bus.states = {('left', i): (0., 0, 9.) for i in range(1, 9)}
        self.bus.advance_feedback = False
        with self.assertRaisesRegex(RuntimeError, 'not confirmed relaxed'):
            runner._relax_confirmed(self.bus)


if __name__ == '__main__':
    unittest.main()
