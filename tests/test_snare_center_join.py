"""Run4 regression: process join expires the stricter center-disable proof."""
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock,patch
import unittest
from camera_playback.left_hold import LEFT_CENTER,LEFT_GRIPPER_TARGET
from tests.test_left_center_hold import fake_bus

class CenterJoinTests(unittest.TestCase):
    def test_join_then_new_feedback_and_settle_window_precede_all_disables(self):
        now=[100.]
        with patch('time.monotonic',side_effect=lambda:now[0]):
            b=fake_bus();b.active=b.left_drive.active=True;b.require_center_before_relax=True
            def feedback(*args):
                for side in ('left','right'):
                    for i in range(1,9):
                        q=([*LEFT_CENTER,LEFT_GRIPPER_TARGET][i-1] if side=='left' else 0.)
                        b.states[side,i]=(q,2,now[0])
            feedback()
            def history():return deque([(now[0]-.64,LEFT_CENTER.copy()),(now[0],LEFT_CENTER.copy())])
            b.dual_center_history={'left':history(),'right':deque()};b.snare_center_history=history()
            def stop():
                now[0]+=.15
                b.left_joint6_session=b.left_drive.left_joint6_session=None
            session=SimpleNamespace(finished=True,stop=Mock(side_effect=stop))
            b.left_joint6_session=b.left_drive.left_joint6_session=session
            with patch('camera_playback.recording_only.refresh_feedback',side_effect=feedback) as refresh:
                with self.assertRaisesRegex(RuntimeError,'Center not settled yet'):b.disable_centered_side('left')
            refresh.assert_called_once_with(b,.08);session.stop.assert_called_once()
            self.assertFalse(b.dual_center_history['left']);self.assertFalse(b.snare_center_history)
            b.sockets['left'].sock.send.assert_not_called()
            with self.assertRaisesRegex(RuntimeError,'Center not settled yet'):b.disable_centered_side('left')
            now[0]+=.65;feedback()
            b.dual_center_history['left']=history();b.snare_center_history=history()
            b.disable_centered_side('left')
            self.assertEqual(b.sockets['left'].sock.send.call_count,24)
            self.assertIn('left',b.center_disabled)
