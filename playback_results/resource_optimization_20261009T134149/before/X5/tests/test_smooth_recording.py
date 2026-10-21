"""Offline integration/transport tests. Fake sockets only; never move hardware."""
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App, CameraSearchApp
from camera_playback.smooth_recording import load_smooth_recording, RECORD3_DURATION_RATIO
from camera_playback.smooth_recording_worker import RecordingMotors, RecordingSession
from centering.motors import packet, parameter, RIGHT_GRIPPER_CLOSED
from safe_zone.encoder import FRAME, EFF, joint_to_motor, request_frame
from smooth_playback.trajectory import Geometry, ROOT, build


def forbid_can(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.PF_CAN:
        raise RuntimeError('Smooth integration tests prohibit physical CAN sockets')


sys.addaudithook(forbid_can)  # Also installed in spawned test children.


class FakeLock:
    def detach(self):
        return os.open('/dev/null', os.O_RDONLY)


class FakeSocket:
    def __init__(self, directory, config):
        self.directory, self.config = directory, config
        self.responses = deque()
        self.gains = dict(enumerate([80., 80., 60., 60., 30., 30., 30.], 1))
        self.mode = int(config.get('mode', 5))
        self.targets = dict(enumerate(config['first'], 1))
        self.targets[8] = RIGHT_GRIPPER_CLOSED
        self.gain_reduced = False

    def setsockopt(self, *args): pass
    def setblocking(self, *args): pass
    def bind(self, address): self.side = 'right' if address[0] == 'can0' else 'left'

    def feedback(self, motor):
        q = self.targets[motor] if self.side == 'right' else 0.
        raw = joint_to_motor(self.side, motor, q) if motor < 8 else -q
        count = int(round((raw+12.57)/25.14*65535))
        state = 2 if self.side == 'right' else 0
        data = struct.pack('>HHHH', count, 32768, 32768, 250)
        self.responses.append(FRAME.pack(EFF | 2 << 24 | state << 22 | motor << 8 | 0xfd, 8, data))

    def send(self, frame):
        cid, _, data = FRAME.unpack(frame)
        kind, motor = (cid >> 24) & 31, cid & 255
        index = int.from_bytes(data[:2], 'little')
        if kind == 2:
            assert frame == request_frame(motor)
            self.feedback(motor)
        elif kind == 17:
            if index == 0x701e:
                result = struct.pack('<H2xf', index, self.gains[motor])
            else:
                assert index == 0x7005
                result = struct.pack('<H2xB3x', index, self.mode)
            self.responses.append(FRAME.pack(EFF | 17 << 24 | motor << 8 | 0xfd, 8, result))
        elif kind == 18:
            value = struct.unpack('<f', data[4:])[0]
            if index == 0x701e:
                if self.config.get('fail_restore') and self.gain_reduced and value > 10.:
                    raise RuntimeError('Injected gain restoration failure')
                self.gains[motor] = value
                if value == 10.: self.gain_reduced = True
            elif index == 0x7016:
                from safe_zone.encoder import encoder_to_joint
                self.targets[motor] = encoder_to_joint(self.side, motor, value)
            else:
                assert index == 0x7017
        else:
            raise AssertionError(f'Forbidden fake command {kind}')
        return 16

    def recv(self, count):
        if not self.responses:
            raise BlockingIOError()
        return self.responses.popleft()

    def close(self):
        (self.directory/f'fake_{self.side}_final.json').write_text(json.dumps(
            dict(gains=self.gains, targets=self.targets, mode=self.mode)))


class FakeRecordingMotors(RecordingMotors):
    def __init__(self, directory, handles):
        config = json.loads((directory/'fake_initial.json').read_text())
        with patch('camera_playback.smooth_recording_worker.socket.socket',
                   side_effect=lambda *a: FakeSocket(directory, config)):
            super().__init__(directory, [FakeLock(), FakeLock()])


class SmoothRecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.g = Geometry()
        cls.recordings = {}
        for name in ('record1', 'record2', 'record3'):
            cls.recordings[name] = load_smooth_recording(
                ROOT/'recordings'/f'{name}.json', cls.g.model, cls.g.zone,
                cls.g.lower, cls.g.upper, cls.g.center, .4)
        cls.recording = cls.recordings['record3']

    def app(self, hardware_test=False):
        app = App.__new__(App)
        app.hardware = True
        app.hardware_test_mode, app.test_mode = hardware_test, False
        app.playback_trajectory = self.recording
        app.smooth_playback_session = None
        app.smooth_playback_cleanup_error = None
        app.bus = Mock(active=True)
        app.status = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app.hill_ik = SimpleNamespace(position=lambda q: np.zeros(3))
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3), speed=.4)
        app._program_failure = Mock()
        return app

    def test_record3_is_same_tested_physical_trajectory(self):
        _, methods = build(self.g)
        original = methods['paced_precise']
        integrated = self.recording.smooth_motion
        self.assertEqual(integrated.duration, 4.8)
        for t in np.linspace(0., 4.8, 2001):
            np.testing.assert_allclose(integrated.at(t), original.at(t), atol=2e-14, rtol=0)
        for key in ('command_hz', 'position_gain_cap', 'precise_endpoint', 'firmware_joint_speeds_rad_s'):
            self.assertEqual(integrated.metadata[key], original.metadata[key])

    def test_all_selected_recordings_remain_supported_and_endpoint_exact(self):
        for name, recording in self.recordings.items():
            with self.subTest(recording=name):
                source = json.loads(recording.source.read_text())
                np.testing.assert_array_equal(recording.joints_at(0)[0], source['samples'][0]['positions_rad'])
                np.testing.assert_array_equal(recording.joints_at(recording.duration_s)[0], source['samples'][-1]['positions_rad'])
                self.assertAlmostEqual(recording.duration_s/recording.original_duration_s, RECORD3_DURATION_RATIO)
                self.assertLessEqual(recording.smooth_motion.metadata['max_joint_deviation_deg'], 1.5)
                self.assertLessEqual(recording.smooth_motion.metadata['max_tcp_deviation_mm'], 10.)

    def test_single_frame_and_stationary_recordings(self):
        for count in (1, 3):
            source = json.loads(self.recording.source.read_text())
            sample = source['samples'][0]
            source['samples'] = [dict(sample, time_s=i*.1) for i in range(count)]
            source['sample_count'] = count
            with tempfile.TemporaryDirectory() as directory:
                p = Path(directory)/'still.json'
                p.write_text(json.dumps(source))
                recording = load_smooth_recording(p, self.g.model, self.g.zone,
                    self.g.lower, self.g.upper, self.g.center, .4)
                for t in (0., .02, 1.):
                    np.testing.assert_allclose(recording.joints_at(t)[0], sample['positions_rad'], atol=1e-14)

    def test_physical_modes_use_new_loader_but_test_and_preview_do_not(self):
        for hardware, test, hardware_test in ((True, False, False), (True, False, True),
                                               (True, True, False), (False, False, False)):
            app = self.app(hardware_test)
            app.hardware, app.test_mode, app.bus = hardware, test, None
            app.model, app.zone = self.g.model, self.g.zone
            app.lower, app.upper, app.center_goal = self.g.lower, self.g.upper, self.g.center
            for name in ('_show_recording_loading', '_hide_recording_loading', 'recording_status',
                         'recording_label', '_refresh_buttons'):
                setattr(app, name, Mock())
            with patch('camera_playback.app.load_smooth_recording', return_value=self.recording) as smooth, \
                 patch('camera_playback.app.load_playback_trajectory', return_value=self.recording) as old:
                self.assertTrue(app.load_recording('selected.json'))
            self.assertEqual(smooth.call_count, int(hardware and not test))
            self.assertEqual(old.call_count, int(not hardware or test))

    def test_spawn_handoff_primes_fresh_feedback_before_resuming_watchdog(self):
        from camera_playback.playback_transport import PlaybackMotors
        app = self.app()
        app.bus = PlaybackMotors.__new__(PlaybackMotors)
        session = Mock()
        with patch('camera_playback.app.RecordingSession', return_value=session), \
             patch('camera_playback.app.recording_only_control.refresh_feedback') as refresh:
            app._begin_playback(1.)
        self.assertIs(app.bus.playback_session, session)
        refresh.assert_called_once_with(app.bus, .10)

    def test_both_modes_handoff_to_same_settling_and_alignment_without_relax(self):
        for hardware_test in (False, True):
            app = self.app(hardware_test)
            session = Mock()
            result = dict(success=True, gains_restored=True, cleanup_error=None)
            session.stop.return_value = result
            session.poll.return_value = dict(done=False, elapsed=1., phase='PLAYBACK', result=None)
            app.begin_stage = Mock()
            app._begin_playback_alignment = Mock()
            with patch('camera_playback.app.RecordingSession', return_value=session) as factory:
                app._begin_playback(1.)
            factory.assert_called_once_with(self.recording, app.bus)
            self.assertIsNone(app.control)
            app._advance_playback(2.)
            app.bus.set_positions.assert_not_called()
            app.bus.set_right_arm_speed.assert_not_called()
            app.begin_stage.assert_not_called()
            session.poll.return_value = dict(done=True, elapsed=4.8, phase='ENDPOINT VERIFIED', result=result)
            app._advance_playback(6.)
            session.stop.assert_called_once()
            self.assertEqual(app.begin_stage.call_args.args[0], 'SETTLING RECORDING END')
            np.testing.assert_array_equal(app.begin_stage.call_args.args[1], self.recording.last_joints)
            app.phase = 'SETTLING RECORDING END'
            app.complete_stage(7.)
            app._begin_playback_alignment.assert_called_once_with(7.)
            app.bus.set_right_arm_speed.assert_called_once_with(.4)
            app.bus.center.assert_not_called()
            app.bus.relax.assert_not_called()

    def test_error_or_unrestored_gains_never_advance_to_alignment(self):
        for result in (dict(success=False, error='lost feedback', gains_restored=True),
                       dict(success=True, cleanup_error='readback failed', gains_restored=False)):
            app = self.app()
            app.playback_visual_samples = []
            session = Mock()
            session.stop.return_value = result
            session.poll.return_value = dict(done=True, elapsed=1., phase='ERROR', result=result)
            app.smooth_playback_session = session
            app.begin_stage = Mock()
            app._advance_playback(2.)
            app._program_failure.assert_called_once()
            app.begin_stage.assert_not_called()

    def test_recovery_stage_joins_worker_before_parent_targets(self):
        app = self.app()
        order = []
        app.smooth_playback_session = Mock(stop=lambda: (order.append('stop') or dict(gains_restored=True)))
        with patch.object(CameraSearchApp, 'begin_stage', side_effect=lambda *a: order.append('stage')):
            app.begin_stage('ZONE RECENTERING', self.g.center)
        self.assertEqual(order, ['stop', 'stage'])

    def test_close_joins_worker_before_parent_bus_cleanup(self):
        app = self.app()
        order = []
        app.root = SimpleNamespace(mainloop=lambda: order.append('mainloop'))
        app.smooth_playback_session = Mock(stop=lambda: (order.append('stop') or dict(gains_restored=True)))
        app.audio_receiver = Mock()
        app.hihat = None
        def parent_run():
            app.root.mainloop()
            order.append('parent cleanup')
        with patch.object(CameraSearchApp, 'run', side_effect=parent_run):
            app.run()
        self.assertEqual(order, ['mainloop', 'stop', 'parent cleanup'])

    def test_transport_cannot_enable_disable_change_modes_or_touch_gripper_left(self):
        bus = RecordingMotors.__new__(RecordingMotors)
        for side, frame in [('right', packet(3, 1)), ('right', packet(4, 1)),
                            ('right', parameter(1, 0x7005, 5, True)),
                            ('right', parameter(8, 0x7016, 0.)),
                            ('left', parameter(1, 0x7016, 0.)),
                            ('right', parameter(1, 0x7018, 6.)),
                            ('right', packet(6, 1))]:
            with self.assertRaises(RuntimeError):
                bus._send(side, frame)
        with self.assertRaises(RuntimeError): bus.center()
        with self.assertRaises(RuntimeError): bus.relax()

    def session(self, directory, **config):
        directory = Path(directory)
        (directory/'fake_initial.json').write_text(json.dumps(dict(first=self.recording.first_joints.tolist(), **config)))
        bus = SimpleNamespace(active=True, control_side='right', control_gripper=True,
                              isolated_motors=set(), right_joint7_session=None)
        return RecordingSession(self.recording, bus, directory=directory, bus_factory=FakeRecordingMotors)

    def wait(self, session, phase=None, timeout=20.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            update = session.poll()
            if update['done'] or update['phase'] == phase:
                return update
            time.sleep(.01)
        self.fail('Worker did not reach expected state')

    def test_spawned_worker_runs_exact_method_without_gui_clock(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self.session(directory)
            try:
                update = self.wait(session, 'PLAYBACK')
                self.assertFalse(update['done'], update)
                # Simulate a 250ms occupied GUI. The child must keep streaming.
                until = time.monotonic()+.25
                while time.monotonic() < until: sum(range(10000))
                update = self.wait(session)
                self.assertTrue(update['result']['success'], update)
                self.assertTrue(update['result']['gains_restored'])
                self.assertLess(update['result']['endpoint']['max_error_deg'], .05)
                with open(Path(directory)/'trace.csv') as trace:
                    rows = list(csv.DictReader(trace))
                stamps = np.array([float(row['monotonic_s']) for row in rows if row['phase'] == 'PLAYBACK'])
                self.assertGreater(len(stamps), 750)
                self.assertLess(np.median(np.diff(stamps)), .007)
                self.assertAlmostEqual(stamps[-1]-stamps[0], 4.8, delta=.04)
                commands = [json.loads(line) for line in (Path(directory)/'commands.jsonl').read_text().splitlines()]
                self.assertTrue(all(row['side'] == 'right' and 1 <= row['motor'] <= 7
                                    and row['kind'] in (17, 18) for row in commands))
                final = json.loads((Path(directory)/'fake_right_final.json').read_text())
                self.assertEqual(list(final['gains'].values()), [80., 80., 60., 60., 30., 30., 30.])
                np.testing.assert_allclose([final['targets'][str(i)] for i in range(1, 8)],
                                           self.recording.last_joints, atol=1e-7)
            finally:
                session.stop()

    def test_spawned_worker_cancel_stops_and_restores_without_center_or_relax(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self.session(directory)
            try:
                self.assertFalse(self.wait(session, 'PLAYBACK')['done'])
                result = session.stop()
                self.assertFalse(result['success'])
                self.assertTrue(result['gains_restored'], result)
                self.assertIsNone(result['cleanup_error'])
                self.assertFalse(session.process.is_alive())
            finally:
                session.stop()

    def test_spawned_worker_rejects_bad_mode_without_gain_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self.session(directory, mode=0)
            try:
                result = self.wait(session)['result']
                self.assertFalse(result['success'])
                self.assertIn('requires existing CSP', result['error'])
                commands = [json.loads(line) for line in (Path(directory)/'commands.jsonl').read_text().splitlines()]
                self.assertTrue(all(row['kind'] == 17 for row in commands))
            finally:
                session.stop()

    def test_spawned_worker_reports_unrestored_gains_instead_of_success(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self.session(directory, fail_restore=True)
            try:
                result = self.wait(session)['result']
                self.assertFalse(result['success'])
                self.assertFalse(result['gains_restored'])
                self.assertIn('gain restoration failure', result['cleanup_error'])
            finally:
                session.stop()

    def test_spawned_worker_gui_watchdog(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self.session(directory)
            try:
                self.assertFalse(self.wait(session, 'PLAYBACK')['done'])
                time.sleep(2.7)  # Intentionally no heartbeat.
                result = self.wait(session)['result']
                self.assertFalse(result['success'])
                self.assertIn('heartbeat lost', result['error'])
                self.assertTrue(result['gains_restored'])
            finally:
                session.stop()


if __name__ == '__main__':
    unittest.main()
