"""Slow spawn must query fresh encoders before returning to the active GUI."""
from pathlib import Path
from types import SimpleNamespace
import tempfile
from camera_playback.smooth_recording_worker import PHASES
import unittest
from unittest.mock import Mock,patch
from centering.motors import Motors
from camera_playback.smooth_recording_worker import RecordingSession

class RecordingSpawnFeedbackTests(unittest.TestCase):
    def test_real_transport_refreshes_after_spawn_and_stops_child_if_refresh_fails(self):
        for fail in (False,True):
            with self.subTest(fail=fail),tempfile.TemporaryDirectory() as p:
                bus=Motors.__new__(Motors)
                bus.active=True;bus.control_side='right';bus.control_gripper=True
                bus.isolated_motors=set();bus.right_joint7_session=None;bus.locks=[9,10]
                recording=SimpleNamespace(side='right',smooth_motion=SimpleNamespace(metadata={}))
                events=[];ctx=Mock();ctx.Process.return_value.start.side_effect=lambda:events.append('spawn')
                def refresh(b,seconds):
                    self.assertIs(b,bus);self.assertEqual(seconds,.08);events.append('fresh query')
                    if fail:raise RuntimeError('Fresh feedback unavailable')
                with patch('camera_playback.smooth_recording_worker.mp.get_context',return_value=ctx), \
                     patch('camera_playback.smooth_recording_worker.DupFd'), \
                     patch('camera_playback.recording_only.refresh_feedback',side_effect=refresh), \
                     patch.object(RecordingSession,'stop') as stop:
                    if fail:
                        with self.assertRaisesRegex(RuntimeError,'Fresh feedback'):RecordingSession(recording,bus,directory=Path(p))
                        stop.assert_called_once()
                    else:
                        RecordingSession(recording,bus,directory=Path(p));stop.assert_not_called()
                self.assertEqual(events,['spawn','fresh query'])

    def test_live_worker_teardown_does_not_claim_feedback_ownership(self):
        s=RecordingSession.__new__(RecordingSession)
        s.process=Mock();s.process.is_alive.return_value=True
        for phase in PHASES:
            s.progress=[PHASES.index(phase),0.]
            expected=phase not in ('STARTING','ENDPOINT VERIFIED','ERROR')
            self.assertEqual(s.owns_feedback,expected,phase)
        s.process.is_alive.return_value=False
        s.progress=[PHASES.index('PLAYBACK'),0.]
        self.assertFalse(s.owns_feedback)

    def test_gui_resumes_queries_when_live_worker_replies_stop(self):
        from camera_playback.playback_transport import PlaybackMotors
        from tests.test_left_center_hold import fake_bus
        for age,suppressed in ((.05,True),(.11,False)):
            with self.subTest(age=age),patch('time.monotonic',return_value=100.):
                b=fake_bus();b.playback_session=SimpleNamespace(owns_feedback=True,center_return=False)
                b.states={(side,i):(0.,0,100.-age) for side in ('left','right') for i in range(1,9)}
                b.last_query=99.8
                seen=[]
                with patch.object(Motors,'poll',side_effect=lambda:seen.append(b.last_query)):
                    b.poll()
                self.assertEqual(seen,[100. if suppressed else 99.8])
