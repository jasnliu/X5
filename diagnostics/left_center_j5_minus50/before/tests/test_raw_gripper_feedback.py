"""Exact wire feedback display/archive, independent of finger geometry."""
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from safe_zone.encoder import EFF, FRAME, SingleArmObserver
from safe_zone.gripper_feedback import EncoderState, motor8_feedback, gripper_feedback_text
from motion_recording.recording import MotionRecording, joint_names, gripper_name
from motion_recording.editor import EditableRecording
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from centering.motors import Motors


class RawGripperTests(unittest.TestCase):
    def test_two_positive_angles_that_used_to_display_zero_remain_distinct(self):
        for count in (33411, 35774, 0, 65535):
            raw = motor8_feedback(count)
            self.assertEqual(raw['encoder_count'], count)
            self.assertEqual(raw['raw_rad'], count / 65535 * 25.14 - 12.57)
            text = gripper_feedback_text(raw)
            self.assertIn(f'raw count {count} / 65535', text)
            self.assertIn(repr(raw['raw_rad']), text)
            self.assertNotIn('mm', text)
        self.assertNotEqual(gripper_feedback_text(motor8_feedback(33411)),
                            gripper_feedback_text(motor8_feedback(35774)))
        self.assertIn('unavailable', gripper_feedback_text(None))

    def test_query_feedback_preserves_exact_wire_bytes_for_both_arms(self):
        for side in ('left', 'right'):
            for count in (33411, 35774):
                sock = Mock()
                frames = [FRAME.pack(EFF | 2 << 24 | i << 8 | 0xfd, 8,
                          (count if i == 8 else 32768).to_bytes(2, 'big') + bytes(6))
                          for i in range(1, 9)]
                sock.recv.side_effect = [BlockingIOError()] + frames
                sock.send.return_value = 16
                with patch('safe_zone.encoder.socket.socket', return_value=sock), \
                     patch('safe_zone.encoder.os.open', return_value=17), \
                     patch('safe_zone.encoder.os.close'), \
                     patch('safe_zone.encoder.fcntl.flock'), \
                     patch('safe_zone.encoder.select.select', return_value=([sock], [], [])):
                    observer = SingleArmObserver(side, 'can1' if side == 'left' else 'can0')
                    state = observer.sample()
                    observer.close()
                self.assertEqual(state[gripper_name(side)], 0.)  # Legacy RViz only.
                self.assertEqual(state.motor8_feedback[side], motor8_feedback(count))
                self.assertEqual(state.copy().motor8_feedback, state.motor8_feedback)
                self.assertEqual(len(state), 8)  # No telemetry masquerades as a ROS joint.

    def test_controller_retains_raw_count_separately_from_signed_control_angle(self):
        bus = Motors.__new__(Motors)
        bus.control_side, bus.control_gripper, bus.active = 'right', True, False
        bus.states, bus.modes = {}, {}
        bus.last_query = time.monotonic()
        bus.right_joint7_query_period = None
        count = 35774
        frame = FRAME.pack(EFF | 2 << 24 | 8 << 8 | 0xfd, 8, count.to_bytes(2, 'big') + bytes(6))
        bus.sockets = {'right': Mock(recv=Mock(side_effect=[frame, BlockingIOError()]))}
        bus.poll()
        self.assertEqual(bus.motor8_feedback['right'], motor8_feedback(count))
        self.assertEqual(bus.states['right', 8][0], -motor8_feedback(count)['raw_rad'])

    def test_saved_and_cropped_raw_data_are_exact_and_legacy_is_not_inverted(self):
        for side in ('left', 'right'):
            rec = MotionRecording('a' * 64, side)
            rec.start('2026-10-06T12:00:00.000Z')
            for i, count in enumerate((33411, 35774)):
                state = EncoderState({name: 0. for name in joint_names(side)},
                                     motor8_feedback={side: motor8_feedback(count)})
                state[gripper_name(side)] = 0.
                rec.add(1. + i, state, [0., 0., 0.])
            rec.stop()
            with tempfile.TemporaryDirectory() as directory:
                path = rec.save(Path(directory) / 'raw.json')
                clip = EditableRecording.load(path, 'a' * 64)
                self.assertEqual(clip.raw_gripper_at(.1), motor8_feedback(33411))
                self.assertEqual(clip.raw_gripper_at(.9), motor8_feedback(35774))
                clip.overwrite_crop(1, 1)
                self.assertEqual(json.loads(path.read_text())['samples'][0]['motor8_encoder_count'], 35774)
                legacy = rec.as_dict()
                for sample in legacy['samples']:
                    sample.pop('motor8_encoder_count'); sample.pop('motor8_raw_rad')
                path.write_text(json.dumps(legacy))
                self.assertIsNone(EditableRecording.load(path, 'a' * 64).raw_gripper_at(0.))

    def test_center_joint5_and_raw_gripper_closed_target(self):
        np.testing.assert_array_equal(LEFT_CENTER, np.radians([0, 0, 0, 0, -25, 0, 0]))
        self.assertAlmostEqual(-LEFT_GRIPPER_TARGET, math.radians(14.16))


if __name__ == '__main__':
    unittest.main()
