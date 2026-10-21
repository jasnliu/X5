"""Actual spawned 200 Hz left recording worker with a fake MIT-owned J6.

Fake CAN sockets only. The fake J6 follows shared memory; any J6 CSP position
packet on the wire fails the test. Both forward and reverse use the saved path.
"""
import json
import math
import multiprocessing as mp
from pathlib import Path
import struct
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from camera_playback.smooth_recording_worker import LeftRecordingMotors, RecordingSession
from camera_playback.left_return import ReverseRecording
from camera_playback.mit_strike import Sample
from camera_playback.snare_beat import SnareStatus
from safe_zone.encoder import EFF, FRAME
from tests import test_snare_beat as fixtures
from tests.test_dual_recording import LeftFakeSocket
from tests.test_smooth_recording import FakeLock


class OwnedJ6Socket(LeftFakeSocket):
    def __init__(self, directory, config, goal):
        super().__init__(directory, config)
        self.goal = goal

    def send(self, frame):
        cid, _, data = FRAME.unpack(frame)
        kind, motor = (cid >> 24) & 31, cid & 255
        index = int.from_bytes(data[:2], 'little')
        if self.side == 'left' and motor == 6:
            if kind == 18 and index == 0x7016:
                raise AssertionError('CSP position sent to MIT-owned J6')
            if kind == 17 and index == 0x7005:
                result = struct.pack('<H2xB3x', index, 0)
                self.responses.append(FRAME.pack(EFF | 17 << 24 | motor << 8 | 0xfd, 8, result))
                return 16
        return super().send(frame)

    def feedback(self, motor):
        if self.side == 'left' and motor == 6:
            self.targets[6] = self.goal.value
        super().feedback(motor)


class FakeOwnedLeftMotors(LeftRecordingMotors):
    def __init__(self, directory, handles, j6_goal=None):
        config = json.loads((directory/'fake_initial.json').read_text())
        with patch('camera_playback.smooth_recording_worker.socket.socket',
                   side_effect=lambda *a: OwnedJ6Socket(directory, config, j6_goal)):
            super().__init__(directory, [FakeLock(), FakeLock()], j6_goal=j6_goal)


class SnareRecordingBridgeTests(unittest.TestCase):
    def test_spawned_forward_and_reverse_keep_j6_in_mit(self):
        fixtures.SnareBeatTests.setUpClass()
        original = fixtures.SnareBeatTests.recording
        ctx = mp.get_context('spawn')
        for recording in (original, ReverseRecording(original, original.duration_s)):
            with self.subTest(reverse=isinstance(recording, ReverseRecording)), tempfile.TemporaryDirectory() as directory:
                directory = Path(directory)
                goal = ctx.Value('d', float(recording.first_joints[5]))
                snare = SimpleNamespace(finished=True, status=SnareStatus(armed=True), goal=goal)
                bus = SimpleNamespace(active=True, control_side='right', control_gripper=True,
                    isolated_motors=set(), right_joint7_session=None, left_joint6_session=snare)
                (directory/'fake_initial.json').write_text(json.dumps(dict(
                    first=recording.first_joints.tolist(), right_center=[0.]*7)))
                session = RecordingSession(recording, bus, directory=directory, bus_factory=FakeOwnedLeftMotors)
                try:
                    deadline = time.monotonic()+25.
                    while time.monotonic() < deadline:
                        result = session.poll()
                        if result['done']: break
                        time.sleep(.01)
                    else: self.fail('Fake recording timeout')
                    self.assertTrue(result['result']['success'], result)
                    self.assertTrue(result['result']['gains_restored'])
                    self.assertAlmostEqual(goal.value, float(recording.last_joints[5]), places=6)
                    for line in (directory/'commands.jsonl').read_text().splitlines():
                        row = json.loads(line)
                        cid, _, data = FRAME.unpack(bytes.fromhex(row['frame']))
                        self.assertFalse(row['side']=='left' and cid & 255 == 6
                            and (cid >> 24) & 31 == 18 and int.from_bytes(data[:2], 'little') == 0x7016)
                finally:
                    session.stop()


if __name__ == '__main__': unittest.main()
