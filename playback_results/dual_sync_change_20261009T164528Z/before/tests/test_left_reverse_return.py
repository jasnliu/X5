"""Left reverse return and right-arm independence. All transports are simulated."""
from collections import deque
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from camera_playback.app import App
from camera_playback import dual_recording as dual
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.left_return import ReverseRecording, LeftReturnSession, LeftReturn
from camera_playback.simulation import SimulatedMotors
from centering.motors import RIGHT_GRIPPER_CLOSED
from goal_motion.app import App as JointApp
from tests import test_dual_recording as fixtures
from tests.test_left_center_hold import fake_bus


class LeftReverseReturnTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixtures.DualRecordingTests.setUpClass()
        cls.f = fixtures.DualRecordingTests()

    def test_exact_time_reversal_of_cached_curves_and_endpoints(self):
        for recording in (self.f.left, self.f.legacy):
            for fraction in (1., .4, 0.):
                duration = recording.duration_s*fraction
                reverse = ReverseRecording(recording, duration)
                np.testing.assert_array_equal(reverse.first_joints, recording.joints_at(duration)[0])
                np.testing.assert_array_equal(reverse.last_joints, recording.first_joints)
                for t in np.linspace(0, duration, 301):
                    expected = recording.joints_at(duration-t)[0]
                    np.testing.assert_array_equal(reverse.joints_at(t)[0], expected)
                    if recording is self.f.left:
                        np.testing.assert_array_equal(reverse.smooth_motion.at(t), expected)
                self.assertTrue(reverse.joints_at(duration)[1])
                np.testing.assert_array_equal(reverse.joints_at(duration+1)[0], recording.first_joints)
        for duration in (-1., float('nan'), self.f.left.duration_s+1):
            with self.assertRaises(ValueError): ReverseRecording(self.f.left, duration)
        with self.assertRaises(ValueError): ReverseRecording(self.f.right, 0.)

    def simulation(self, q):
        bus=SimulatedMotors(self.f.right.last_joints, RIGHT_GRIPPER_CLOSED)
        list(bus.center(self.f.right_g.center, RIGHT_GRIPPER_CLOSED))
        bus._left_joints = q.copy()
        bus._left_gripper = LEFT_GRIPPER_TARGET
        bus._refresh_states(time.monotonic())
        a,d=self.f.workflow(bus)
        return a,d,bus

    def test_interrupted_playback_reverses_only_played_prefix_then_centers(self):
        with patch('time.monotonic',return_value=100.) as clock, patch('goal_motion.app.rclpy.spin_once'):
            duration=self.f.legacy.duration_s*.4
            q=self.f.legacy.joints_at(duration)[0]
            a,d,bus=self.simulation(q)
            a.phase=dual.PLAYING
            d.started=100.-duration
            d.begin_return()
            self.assertAlmostEqual(d.left_return.reverse.duration_s,duration)
            np.testing.assert_allclose(bus._left_targets,q,atol=1e-14)
            seen_start=False
            for i in range(1600):
                clock.return_value+=.02
                JointApp.tick(a)
                if d.left_return.phase=='CENTER':
                    if not seen_start:
                        np.testing.assert_allclose(bus._left_joints,self.f.legacy.first_joints,atol=math.radians(.20))
                    seen_start=True
                if not bus.active:break
            self.assertTrue(seen_start)
            self.assertFalse(bus.active)
            np.testing.assert_allclose(bus._left_joints,LEFT_CENTER,atol=math.radians(.20))
            self.assertEqual(bus._left_gripper,LEFT_GRIPPER_TARGET)

    def test_final_center_is_smooth_legal_and_does_not_catch_up_after_gui_delay(self):
        a,d,bus=self.simulation(self.f.legacy.first_joints)
        r=LeftReturn(d,0.)
        r.phase='VERIFY RECORDING START';r.control=Mock()
        d.return_controls={}
        bus.left_hold_ready=lambda:True
        r.tick(100.)
        self.assertEqual(r.phase,'CENTER')
        self.assertNotIn('left',d.return_controls)
        r.tick(101.) # One-second scheduling gap must not jump a whole second.
        self.assertAlmostEqual(r.center_elapsed,.04)
        self.assertLess(np.max(np.abs(bus._left_targets-self.f.legacy.first_joints)),.001)
        now=101.
        while r.center_at is not None:
            now+=.02;r.tick(now)
            d.g.check(bus._left_targets)
        np.testing.assert_array_equal(bus._left_targets,LEFT_CENTER)
        self.assertIn('left',d.return_controls)

    def test_startup_center_without_playback_does_not_make_outward_excursion(self):
        with patch('time.monotonic',return_value=100.):
            a,d,bus=self.simulation(LEFT_CENTER)
            d.begin_return()
            self.assertIsNone(d.left_return)
            np.testing.assert_array_equal(bus._left_targets,LEFT_CENTER)

    def test_verified_right_disable_allowed_during_left_return_only(self):
        with patch('time.monotonic',return_value=100.):
            for side,returning,allowed in [('left',True,True),('left',False,False),('right',True,False)]:
                bus=fake_bus();bus.active=bus.left_drive.active=True
                for motor in range(1,9): bus.states['right',motor]=(0.,2,100.)
                history=deque([(99.39,np.zeros(7)),(100.,np.zeros(7))])
                bus.dual_center_history={'right':history,'left':deque()}
                bus.playback_session=SimpleNamespace(side=side,center_return=returning)
                if allowed:
                    bus.disable_centered_side('right')
                    self.assertIn('right',bus.center_disabled)
                    bus.sockets['left'].sock.send.assert_not_called()
                else:
                    with self.assertRaisesRegex(RuntimeError,'exclusive ownership'):
                        bus.disable_centered_side('right')
                with self.assertRaisesRegex(RuntimeError,'exclusive ownership'):
                    bus.disable_centered_side('left')

    def test_right_recovery_does_not_cancel_left_reverse_writer(self):
        a=App.__new__(App)
        a.dual=SimpleNamespace(returning=True,session=Mock(),stop=Mock())
        a.bus=SimpleNamespace(center_disabled={})
        a._stop_smooth_playback=Mock(return_value=True)
        with patch('camera_search.app.App.begin_stage') as old:
            a.begin_stage('FAULT RECENTERING',np.zeros(7))
        old.assert_called_once()
        a.dual.stop.assert_not_called()

    def test_spawned_reverse_worker_returns_to_original_start_left_commands_only(self):
        reverse=ReverseRecording(self.f.left,self.f.left.duration_s)
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            (p/'fake_initial.json').write_text(json.dumps(dict(first=reverse.first_joints.tolist(),
                right_center=self.f.right_g.center.tolist())))
            bus=SimpleNamespace(active=True,control_side='right',control_gripper=True,
                isolated_motors=set(),right_joint7_session=None)
            session=LeftReturnSession(reverse,bus,directory=p,bus_factory=fixtures.FakeLeftMotors)
            try:
                deadline=time.monotonic()+25
                while time.monotonic()<deadline:
                    result=session.poll()
                    if result['done']:break
                    time.sleep(.02)
                self.assertTrue(result['done'])
                self.assertTrue(result['result']['success'],result)
                self.assertTrue(result['result']['gains_restored'])
                commands=[json.loads(r) for r in (p/'commands.jsonl').read_text().splitlines()]
                self.assertTrue(commands)
                self.assertTrue(all(r['side']=='left' and r['motor'] in range(1,8) for r in commands))
                final=json.loads((p/'fake_left_final.json').read_text())
                np.testing.assert_allclose([final['targets'][str(i)] for i in range(1,8)],reverse.last_joints,atol=1e-6)
                self.assertEqual(final['gains'],{'1':80.,'2':80.,'3':60.,'4':60.,'5':30.,'6':30.,'7':30.})
            finally:session.stop()
