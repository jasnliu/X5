"""Exercise production worker loop/IPC with quantized fake J6 packets. NO CAN."""
import errno
import pickle
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from camera_playback.snare_beat import _run
from camera_playback.mit_strike import Sample
from safe_zone.encoder import FRAME
from tests import test_snare_beat as fixtures


class WorkerIOTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): fixtures.SnareBeatTests.setUpClass()

    def run_worker(self, heartbeat_lost=False, pressure=None, patterns=(1,)):
        f = fixtures.SnareBeatTests
        t = [0.]
        heartbeat = SimpleNamespace(value=0.)
        goal = SimpleNamespace(value=-f.anchor)
        grid = [1., .6, 1.]
        stop = threading.Event()
        states, frames = [], []
        class Channel:
            def __init__(self, interface):
                self.sample = Sample(f.anchor, 0., 0., 2, 0.)
                self.closed = False
                self.attempts = 0
            def receive(self):
                t[0] += .001
                if not heartbeat_lost: heartbeat.value = t[0]
                q, v = self.sample.position, self.sample.velocity
                self.sample = Sample(q, v, 0., 2, t[0])
                if t[0] >= 4.: grid[2] = 0.
                if t[0] >= 5.5: stop.set()
            def wait(self, seconds): t[0] += min(.001, seconds)
            def send(self, frame):
                self.attempts += 1
                if pressure == 'temporary' and self.attempts % 37 in (0, 1):
                    raise OSError(errno.ENOBUFS, 'Full CAN queue')
                if pressure == 'persistent' and t[0] >= 1.8:
                    raise OSError(errno.ENOBUFS, 'Full CAN queue')
                if pressure == 'permanent' and t[0] >= 1.8:
                    raise OSError(errno.ENETDOWN, 'Interface down')
                cid, _, data = FRAME.unpack(frame)
                assert (cid >> 24) & 31 == 1 and cid & 255 == 6
                frames.append(frame)
                # sign=-1 J6: raw == internal negative strike position.
                q = int.from_bytes(data[:2], 'big')/65535*25.14-12.57
                v = int.from_bytes(data[2:4], 'big')/65535*66.-33.
                self.sample = Sample(q, v, 0., 2, t[0])
            def close(self): self.closed = True
        channel = Channel('fake')
        sender = SimpleNamespace(send=lambda b: states.append(pickle.loads(b)), close=lambda: None)
        with patch('time.monotonic', side_effect=lambda: t[0]), \
             patch('camera_playback.snare_beat.signal.signal'), \
             patch('camera_playback.snare_beat.random.Random', return_value=fixtures.Choices(patterns)), \
             patch('snare_lab.runner._LeftJoint6Channel', return_value=channel), \
             patch('camera_playback.snare_beat.Joint7Worker._prepare'):
            _run('fake', f.template, f.rules, f.limits, -.75, .75,
                 goal, grid, stop, heartbeat, sender)
        self.assertTrue(channel.closed)
        self.assertGreater(len(frames), 100)
        return states

    def test_worker_streams_only_j6_mit_and_finishes_on_grid_cancel(self):
        states = self.run_worker()
        self.assertFalse([s.error for s in states if s.error])
        self.assertEqual(states[-1].completed, 2)
        self.assertFalse(states[-1].moving or states[-1].scheduled)
        hits = {(s.measure, s.beat) for s in states if s.released_at is not None}
        self.assertEqual(hits, {(1, 1), (2, 1)})

    def test_temporary_queue_pressure_recomputes_without_losing_full_strokes(self):
        states = self.run_worker(pressure='temporary')
        self.assertFalse([s.error for s in states if s.error])
        self.assertEqual(states[-1].completed, 2)
        self.assertGreater(states[-1].queue_retries, 10)
        self.assertGreater(states[-1].commands_sent, 100)
        self.assertLess(states[-1].max_gap, .02)

    def test_two_hits_per_bar_survive_quantized_feedback_and_queue_pressure(self):
        states = self.run_worker(pressure='temporary', patterns=((1, 3),))
        self.assertFalse([s.error for s in states if s.error])
        self.assertEqual(states[-1].completed, 4)
        self.assertFalse(states[-1].moving or states[-1].scheduled)
        hits = {(s.measure, s.beat) for s in states if s.released_at is not None}
        self.assertEqual(hits, {(1, 1), (1, 3), (2, 1), (2, 3)})

    def test_persistent_queue_pressure_is_not_silently_ignored(self):
        states = self.run_worker(pressure='persistent')
        self.assertTrue(states[-1].error)
        self.assertTrue('deadline' in states[-1].error)

    def test_nontransient_can_failure_is_still_fatal(self):
        states = self.run_worker(pressure='permanent')
        self.assertIn('Interface down', states[-1].error)
        self.assertEqual(states[-1].queue_retries, 0)

    def test_parent_heartbeat_loss_locks_out_future_strikes_without_disable(self):
        states = self.run_worker(heartbeat_lost=True)
        self.assertIn('heartbeat', states[-1].error)
        self.assertLessEqual(states[-1].completed, 1)


if __name__ == '__main__': unittest.main()
