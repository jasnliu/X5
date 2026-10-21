"""Left-arm (J6) hybrid strike session: generalized by side/motor/sign.

Mirrors tests/test_hybrid_strike.py's WorkerTests pattern exactly, but for
the left arm's strike joint (motor 6, sign=-1.0 — "struck" means an
INCREASING displayed joint value, confirmed against the actual snare
recording's endpoint, the opposite of the right arm's J7 convention). This
proves the side/motor/sign generalization added to camera_playback.mit_strike
and camera_playback.hybrid_strike actually works end-to-end for the left
arm, without touching real hardware (STRIKE_LAB_OFFLINE_ONLY is enforced via
an audit hook exactly like the existing right-arm test).
"""
import math
import os
import socket
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.hybrid_strike import HybridSession, load_tuning
from camera_playback.mit_strike import Command, Sample
from safe_zone.encoder import FRAME
from strike_lab.config import Plant


def forbid_can(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden in left-hybrid tests')


sys.addaudithook(forbid_can)
EVENTS = (('beat 1', True, .6), ('beat 2', True, .4), ('extra after beat 2', False, .2),
          ('beat 3', True, .6), ('beat 4', True, .4), ('extra after beat 4', False, .2))


class LeftSyntheticChannel:
    """Protocol-level fake drive for motor 6, sign=-1.0 (left strike joint).

    Decodes the same wire frame Command.frame(6, -1.0) produces: raw =
    -sign*internal, so internal = -sign*raw — the exact inverse, parametrized,
    of SyntheticChannel in test_hybrid_strike.py (which hardcodes sign=1.0).
    """
    def __init__(self, interface, motor=6, sign=-1.0):
        self.motor, self.sign = motor, sign
        p, _, _ = load_tuning()
        self.plant = Plant(inertia=p['inertia'], gravity=p['load_torque'], friction=p['friction'])
        self.q = .1
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
            s = self.sign
            self.command = Command(
                -s*(int.from_bytes(data[:2], 'big')/65535*25.14-12.57),
                -s*(int.from_bytes(data[2:4], 'big')/65535*66-33),
                int.from_bytes(data[4:6], 'big')/65535*500,
                int.from_bytes(data[6:8], 'big')/65535*5,
                -s*(((cid >> 8) & 65535)/65535*28-14))

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


class LeftWorkerTests(unittest.TestCase):
    def test_spawned_left_session_search_swing_finish_and_exclusive_ownership(self):
        # Internal (sign=-1.0) anchor/corridor derived from a displayed-joint
        # anchor of -0.1 rad, with 12 deg of strike room below the anchor and
        # .6 deg of overshoot room above it -- same relative shape as the
        # right-arm test, just through the sign flip (internal = -displayed).
        displayed_anchor = -.1
        internal_anchor = -displayed_anchor
        lower = internal_anchor-math.radians(12)
        upper = internal_anchor+math.radians(.6)
        bus = SimpleNamespace(control_side='left', active=True, left_joint6_session=None,
                              sockets={'left': Mock()})
        bus.sockets['left'].getsockname.return_value = ('synthetic-only',)
        session = HybridSession(bus, internal_anchor, lower, upper,
                                channel_factory=LeftSyntheticChannel, motor=6, sign=-1.0)
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
            self.assertIs(bus.left_joint6_session, session)
            self.assertIsNone(getattr(bus, 'right_joint7_session', None))
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
        self.assertIsNone(bus.left_joint6_session)
        self.assertFalse(session.process.is_alive())

    def test_default_transport_refused_in_offline_tests(self):
        with patch.dict(os.environ, STRIKE_LAB_OFFLINE_ONLY='1'):
            with self.assertRaisesRegex(RuntimeError, 'forbidden'):
                HybridSession(Mock(control_side='left'), .1, -.1, .2, motor=6, sign=-1.0)

    def test_invalid_motor_rejected(self):
        bus = SimpleNamespace(control_side='left', active=True, left_joint6_session=None,
                              sockets={'left': Mock()})
        with self.assertRaisesRegex(ValueError, 'J6/J7'):
            HybridSession(bus, .1, -.1, .2, channel_factory=LeftSyntheticChannel, motor=5)

    def test_right_and_left_sessions_coexist_independently(self):
        """A left-J6 session and a right-J7 session must not collide."""
        bus = SimpleNamespace(control_side='right', active=True, right_joint7_session=None,
                              left_joint6_session=object(),  # Pretend a left session is active.
                              sockets={'right': Mock()})
        bus.sockets['right'].getsockname.return_value = ('synthetic-only',)
        # The right session must not be blocked by an unrelated left session.
        from test_hybrid_strike import SyntheticChannel
        session = HybridSession(bus, .5, .5-math.radians(12), .5+math.radians(.6),
                                channel_factory=SyntheticChannel)
        try:
            self.assertIs(bus.right_joint7_session, session)
        finally:
            session.stop()


if __name__ == '__main__':
    unittest.main()
