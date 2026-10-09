"""Left-first beat playback, cache and independent center/relax; no real CAN."""
from collections import deque
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import socket
import struct
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback import dual_recording as dual
from camera_playback.app import App, DEFAULT_RECORDING
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.recording_cache import load_cached_smooth_recording
from camera_playback.smooth_recording import load_smooth_recording
from camera_playback.smooth_recording_worker import LeftRecordingMotors, RecordingMotors, RecordingSession
from camera_playback.simulation import SimulatedMotors
from camera_playback.trajectory import load_playback_trajectory
from cartesian_goal.ik import CartesianIK
from centering.motors import RIGHT_GRIPPER_CLOSED, packet, parameter
from safe_zone.encoder import FRAME, EFF, joint_to_motor
from smooth_playback.trajectory import Geometry, ROOT
from tests.test_left_center_hold import fake_bus
from tests.test_smooth_recording import FakeSocket, FakeLock


def forbid_devices(event, args):
    if event == 'socket.__new__' and args[1] == socket.PF_CAN:
        raise RuntimeError('Dual recording tests prohibit physical CAN')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty', '/dev/video', '/dev/snd')):
        raise RuntimeError('Dual recording tests prohibit physical devices')


import sys
sys.addaudithook(forbid_devices)


class LeftFakeSocket(FakeSocket):
    def feedback(self, motor):
        if self.side == 'left':
            q = self.targets[motor] if motor < 8 else LEFT_GRIPPER_TARGET
        else:
            q = self.config['right_center'][motor-1] if motor < 8 else RIGHT_GRIPPER_CLOSED
        raw = joint_to_motor(self.side, motor, q) if motor < 8 else -q
        count = int(round((raw+12.57)/25.14*65535))
        data = struct.pack('>HHHH', count, 32768, 32768, 250)
        self.responses.append(FRAME.pack(EFF | 2 << 24 | 2 << 22 | motor << 8 | 0xfd, 8, data))


class FakeLeftMotors(LeftRecordingMotors):
    def __init__(self, directory, handles):
        config = json.loads((directory/'fake_initial.json').read_text())
        with patch('camera_playback.smooth_recording_worker.socket.socket',
                   side_effect=lambda *a: LeftFakeSocket(directory, config)):
            super().__init__(directory, [FakeLock(), FakeLock()])


class DualRecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.right_g = Geometry()
        cls.g = dual.left_geometry(cls.right_g.model)
        cls.path = dual.DEFAULT_LEFT_RECORDING
        cls.original_bytes = cls.path.read_bytes()
        cls.args = (cls.path, cls.g.model, cls.g.zone, cls.g.lower, cls.g.upper, cls.g.center, .4)
        cls.left = load_smooth_recording(*cls.args, side='left')
        cls.legacy = load_playback_trajectory(*cls.args, side='left', playback_speed=.8)
        g = cls.right_g
        cls.right = load_playback_trajectory(ROOT/'recordings/record3.json',
            g.model, g.zone, g.lower, g.upper, g.center, .4, playback_speed=.8)

    def workflow(self, bus):
        a = SimpleNamespace(bus=bus, test_mode=True, phase='CENTERING',
            center_goal=self.right_g.center, lower=self.right_g.lower, upper=self.right_g.upper,
            playback_trajectory=self.right, control=object(), setup=None,
            status=Mock(), _stop_smooth_playback=Mock(return_value=True),
            relax=Mock(), begin_stage=Mock())
        a.ik = CartesianIK(self.right_g.model,self.right_g.zone,a.lower,a.upper,.4,
            'right', 'openarmx_right_hand_tcp',a.center_goal)
        d = dual.DualRecording.__new__(dual.DualRecording)
        d.app, d.g, d.recording = a, self.g, self.legacy
        d.job = d.session = d.cleanup_error = None
        d.left_done = d.returning = False
        a.dual = d
        return a, d

    def test_defaults_and_left_schema_are_correct(self):
        self.assertEqual(DEFAULT_RECORDING, ROOT/'recordings/record3.json')
        self.assertEqual(self.path, ROOT/'left_recordings/record1.json')
        self.assertEqual(self.left.side, 'left')
        np.testing.assert_array_equal(self.left.first_joints, json.loads(self.original_bytes)['samples'][0]['positions_rad'])
        np.testing.assert_array_equal(self.left.last_joints, json.loads(self.original_bytes)['samples'][-1]['positions_rad'])
        self.assertLessEqual(self.left.smooth_motion.metadata['max_joint_deviation_deg'], 1.5)
        self.assertEqual(self.path.read_bytes(), self.original_bytes)

    def test_left_cache_is_exact_and_cannot_accept_right_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            cold = load_cached_smooth_recording(*self.args, side='left', cache_dir=directory)
            with patch('camera_playback.smooth_recording.load_smooth_recording', side_effect=AssertionError('Expected cache hit')) as loader:
                warm = load_cached_smooth_recording(*self.args, side='left', cache_dir=directory)
            loader.assert_not_called()
            self.assertEqual(warm.geometry.side, 'left')
            for t in np.linspace(0, cold.duration_s, 2001):
                np.testing.assert_array_equal(cold.joints_at(t)[0], warm.joints_at(t)[0])
            with self.assertRaisesRegex(ValueError, 'schema'):
                load_cached_smooth_recording(*self.args, side='right', cache_dir=directory)
            with self.assertRaisesRegex(ValueError, 'schema'):
                load_cached_smooth_recording(ROOT/'recordings/record3.json', *self.args[1:], side='left', cache_dir=directory)

    def test_left_plays_first_gripper_fixed_right_waits_then_endpoint_holds(self):
        with patch('time.monotonic', return_value=100.) as clock:
            bus = SimulatedMotors(self.right_g.center, RIGHT_GRIPPER_CLOSED)
            list(bus.center(self.right_g.center, RIGHT_GRIPPER_CLOSED))
            a, d = self.workflow(bus)
            for i in range(150):
                clock.return_value += .02; bus.poll()
            self.assertTrue(bus.left_center_ready())
            d.begin_left()
            self.assertEqual(a.phase, dual.APPROACH)
            for _ in range(1500):
                clock.return_value += .02; bus.poll(); d.tick(clock.return_value)
                np.testing.assert_array_equal(bus._joints, self.right_g.center)
                self.assertEqual(bus._left_gripper, LEFT_GRIPPER_TARGET)
                if d.left_done:
                    break
                a.begin_stage.assert_not_called()
            self.assertTrue(d.left_done)
            a.begin_stage.assert_called_once()
            self.assertEqual(a.begin_stage.call_args.args[0], 'MOVING TO RECORDING START')
            np.testing.assert_array_equal(a.begin_stage.call_args.args[1], self.right.first_joints)
            np.testing.assert_allclose(bus._left_joints, self.legacy.last_joints, atol=1e-12)
            # Subsequent right motion must not drag left back to its old center.
            a.phase = 'RECORDING PLAYBACK'
            bus.set_positions(self.right.first_joints)
            for _ in range(100):
                clock.return_value += .02; bus.poll()
            np.testing.assert_array_equal(bus._left_targets, self.legacy.last_joints)

    def test_center_dispatches_both_then_relaxes_each_only_at_own_center(self):
        with patch('time.monotonic', return_value=100.) as clock:
            bus = SimulatedMotors(self.right_g.center, RIGHT_GRIPPER_CLOSED)
            list(bus.center(self.right_g.center, RIGHT_GRIPPER_CLOSED))
            bus._left_joints = self.legacy.last_joints.copy()
            bus._left_gripper = LEFT_GRIPPER_TARGET
            bus._joints = self.right.last_joints.copy()
            bus._refresh_states(clock.return_value)
            a, d = self.workflow(bus)
            d.begin_return()
            np.testing.assert_array_equal(bus._left_targets, LEFT_CENTER)
            np.testing.assert_array_equal(bus._targets, self.right_g.center)
            first_disabled = None
            for _ in range(1500):
                clock.return_value += .02; bus.poll(); d.tick(clock.return_value)
                if len(bus.center_disabled) == 1:
                    first_disabled = next(iter(bus.center_disabled))
                    self.assertTrue(bus.active)
                    a.relax.assert_not_called()
                if not bus.active:
                    break
            self.assertIsNotNone(first_disabled)
            self.assertEqual(set(bus.center_disabled), {'left','right'})
            self.assertNotEqual(*bus.center_disabled.values())
            np.testing.assert_allclose(bus._left_joints, LEFT_CENTER, atol=math.radians(.20))
            np.testing.assert_allclose(bus._joints, self.right_g.center, atol=math.radians(.20))
            a.relax.assert_called_once()

    def test_per_side_disable_does_not_release_other_arm(self):
        bus = fake_bus(); bus.active = bus.left_drive.active = True
        now = time.monotonic()
        for i, q in enumerate([*LEFT_CENTER, LEFT_GRIPPER_TARGET], 1):
            bus.states['left', i] = (q, 2, now)
        bus.dual_center_history = {'left':deque([(now-.61, LEFT_CENTER.copy()), (now, LEFT_CENTER.copy())]), 'right':deque()}
        bus.disable_centered_side('left')
        self.assertTrue(bus.active)
        bus.sockets['right'].sock.send.assert_not_called()
        self.assertEqual(bus.sockets['left'].sock.send.call_count, 24)
        self.assertEqual(set(bus.center_disabled), {'left'})
        with self.assertRaisesRegex(RuntimeError, 'not settled'):
            bus.disable_centered_side('right')

    def test_worker_only_writes_selected_left_joints_never_gripper_or_other_arm(self):
        bus = RecordingMotors.__new__(RecordingMotors)
        bus.control_side = 'left'
        for side, frame in [('right', parameter(1,0x7016,.1)), ('left',parameter(8,0x7016,.1)),
                            ('left',packet(3,1)), ('left',packet(4,1)),
                            ('left',parameter(1,0x7005,5,True)), ('left',parameter(1,0x7018,1.))]:
            with self.subTest(side=side,frame=frame), self.assertRaises(RuntimeError):
                bus._send(side,frame)

    def test_stop_joins_left_worker_before_center_targets(self):
        a,d = self.workflow(Mock())
        d.session = Mock()
        events=[]
        d.session.stop.side_effect=lambda: events.append('joined') or {'gains_restored':True}
        a.bus.playback_session=d.session
        a.bus.fresh.return_value=False
        with self.assertRaisesRegex(RuntimeError,'Fresh feedback'):
            d.begin_return()
        self.assertEqual(events,['joined'])
        a.bus.begin_dual_center.assert_not_called()
        self.assertIsNone(a.bus.playback_session)

    def test_failed_left_worker_never_starts_right(self):
        a,d = self.workflow(Mock())
        a.phase=dual.PLAYING
        d.session=Mock()
        d.session.stop.return_value={'gains_restored':False,'error':'unverified gains'}
        d.fault('left failure')
        self.assertEqual(a.phase,dual.FAULT)
        self.assertIsNotNone(d.cleanup_error)
        self.assertFalse(d.ready)
        a.begin_stage.assert_not_called()
        a.relax.assert_not_called()

    def test_app_run_blocked_without_left_pass_and_return_hook_is_dual(self):
        a = App.__new__(App)
        a.dual=Mock(ready=False);a.status=Mock()
        a.start()
        self.assertIn('left recording',a.status.set.call_args.args[0])
        a.dual.ready=True
        a.center_goal=self.right_g.center
        a.begin_stage('CENTER RELAX RECENTERING',a.center_goal)
        a.dual.begin_return.assert_called_once()

    def test_spawned_left_worker_exact_endpoint_restored_gains_no_other_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            (p/'fake_initial.json').write_text(json.dumps({'first':self.left.first_joints.tolist(),
                'right_center':self.right_g.center.tolist()}))
            bus=SimpleNamespace(active=True,control_side='right',control_gripper=True,
                isolated_motors=set(),right_joint7_session=None)
            session=RecordingSession(self.left,bus,directory=p,bus_factory=FakeLeftMotors)
            try:
                deadline=time.monotonic()+25
                while time.monotonic()<deadline:
                    result=session.poll()
                    if result['done']:break
                    time.sleep(.02)
                self.assertTrue(result['done'])
                self.assertTrue(result['result']['success'],result['result'])
                self.assertTrue(result['result']['gains_restored'])
                commands=[json.loads(line) for line in (p/'commands.jsonl').read_text().splitlines()]
                self.assertTrue(commands)
                self.assertTrue(all(r['side']=='left' and r['motor'] in range(1,8) for r in commands))
                final=json.loads((p/'fake_left_final.json').read_text())
                np.testing.assert_allclose([final['targets'][str(i)] for i in range(1,8)],self.left.last_joints,atol=1e-6)
                self.assertEqual(final['gains'],{'1':80.,'2':80.,'3':60.,'4':60.,'5':30.,'6':30.,'7':30.})
            finally:
                session.stop()


if __name__=='__main__':unittest.main()
