"""Regression tests: no arm devices, only fake buses and a serial pseudo-terminal."""
import fcntl
import os
import pty
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App
from camera_playback.emergency import stop_writer
from camera_playback.hihat import HiHatController
from camera_playback.hihat_calibration_runtime import HiHatCalibrationRuntime
from camera_playback.mit_center_return import MitCenterReturn
from camera_playback.playback_transport import RetrySocket
from centering.motors import packet, parameter
from safe_zone.encoder import request_frame
from smooth_playback.trajectory import Geometry
from tests import test_right_center_restore as reference


class ReturnFaultTests(unittest.TestCase):
    def test_recoverable_dual_return_fault_keeps_original_right_recovery_and_j7_goal(self):
        app = App.__new__(App)
        app.phase = 'CENTER RELAX RECENTERING'
        app.bus = SimpleNamespace(active=True,
            mit_center_return=MitCenterReturn(.4, 0., .3, 100.))
        app.dual = SimpleNamespace(returning=True, fault=Mock())
        app._cancel_hihat_calibration = Mock()
        app._stop_smooth_playback = Mock(return_value=True)
        app._stop_hardware_test_session = Mock()
        app._cancel_continuous_striking = Mock()
        app._restore_playback_speed = Mock(return_value=True)
        app._restore_strike_speed = Mock(return_value=True)
        with patch('camera_search.app.App.fail') as original_recovery:
            app.fail('Simulated transient return feedback failure')
        app.dual.fault.assert_not_called()
        original_recovery.assert_called_once_with('Simulated transient return feedback failure')
        self.assertEqual(app.bus.mit_center_return.goal, 0.)
        self.assertTrue(app.dual.returning)

    def test_real_stale_status_still_faults_at_original_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = SimpleNamespace(state='ready', service_running=True,
                telemetry={'angle': 90}, telemetry_at=0., calibrated=True,
                ready=lambda: True, query_status=Mock(), release_all=Mock())
            app = SimpleNamespace(hihat=controller, bus=SimpleNamespace(active=True),
                phase='CENTER RELAX RECENTERING', RETURN_PHASES=App.RETURN_PHASES,
                status=Mock(), fail=Mock(), result_status=Mock(), result_label=Mock())
            runtime = HiHatCalibrationRuntime(app, now=0., directory=directory)
            try:
                runtime.engine.phase = 'READY'
                runtime.engine.selected_angle = 90
                runtime.tick(1.5)
                self.assertTrue(runtime.ready)
                runtime.tick(1.501)
                self.assertIn('status became stale', runtime.engine.failure)
                self.assertFalse(runtime.ready)
                app.fail.assert_not_called()
                controller.query_status.assert_not_called()  # Service is the query owner.
            finally:
                runtime.close()

    def test_peripheral_fault_does_not_cancel_return_but_remains_latched(self):
        for returning in (False, True):
            with self.subTest(returning=returning), tempfile.TemporaryDirectory() as directory:
                app = SimpleNamespace(hihat=Mock(), bus=SimpleNamespace(active=True),
                    phase='CENTER RELAX RECENTERING' if returning else 'RECORDING PLAYBACK',
                    RETURN_PHASES=App.RETURN_PHASES, result_status=Mock(), result_label=Mock(),
                    status=Mock(), fail=Mock())
                runtime = HiHatCalibrationRuntime(app, directory=directory)
                try:
                    runtime.engine.failure = 'Calibrated ESP32 status became stale'
                    runtime._report_failure()
                    runtime._report_failure()
                    self.assertEqual(app.hihat_fault_detail, runtime.engine.failure)
                    if returning:
                        app.fail.assert_not_called()
                        app.hihat.release_all.assert_called_once()
                    else:
                        app.fail.assert_called_once()
                finally:
                    runtime.close()

    def test_mit_fault_then_retry_restores_center_not_frozen_hold(self):
        fixture = reference.RightCenterRestoreTests()
        geometry = Geometry()
        with patch('time.monotonic', return_value=100.):
            app, dual, _ = fixture.workflow(geometry, True)
            q = geometry.center.copy(); q[6] -= .4
            fixture.feedback(app, q, 100.)
            app.bus.require_center_before_relax = True
            app.bus.right_joint7_mode_readback = Mock(return_value=0)
            dual.fault('injected arm-return interruption')
            old = app.bus.mit_center_return
            self.assertEqual(old.goal, q[6])
            dual.begin_return()
            new = app.bus.mit_center_return
            self.assertIsNot(new, old)
            self.assertEqual(new.goal, geometry.center[6])
            self.assertEqual(new.position, q[6])
            self.assertEqual(new.at, 100.)
            self.assertGreater(new.update(q[6], 100.02).position, q[6])
            self.assertTrue(dual.returning)

    def test_status_and_heartbeat_continue_without_gui_ticks(self):
        master, slave = pty.openpty()
        fcntl.fcntl(master, fcntl.F_SETFL, os.O_NONBLOCK)
        controller = HiHatController(os.ttyname(slave))
        stop = threading.Event()
        received = bytearray()
        def firmware():
            while not stop.wait(.005):
                try:
                    data = os.read(master, 4096)
                except BlockingIOError:
                    continue
                received.extend(data)
                for byte in data:
                    if byte == ord('Q'):
                        os.write(master, b'HIHAT v=2 min=90 max=115 angle=90 counts=134 pos=0 target=0 released=1 opened=1 fault=0\n')
        worker = threading.Thread(target=firmware)
        try:
            self.assertTrue(controller.connect())
            controller.state = 'ready'
            controller.initialized = controller.firmware_verified = True
            controller.calibrated = True
            worker.start()
            controller.start_service()
            # Simulate a long synchronous path check: no Tk tick or manual poll.
            time.sleep(1.8)
            self.assertIsNotNone(controller.telemetry_at)
            self.assertLess(time.monotonic()-controller.telemetry_at, .7)
            self.assertGreaterEqual(received.count(b'H'), 10)
            self.assertGreaterEqual(received.count(b'Q'), 3)
            self.assertFalse(any(c in received for c in b'BCOA'))
        finally:
            controller.close()
            stop.set()
            if worker.ident is not None:
                worker.join(1.)
            os.close(master); os.close(slave)
        self.assertFalse(controller.service_running)


class EmergencyTests(unittest.TestCase):
    def test_transport_latch_blocks_resumed_motion_but_allows_disable_and_feedback(self):
        sock = RetrySocket(Mock(send=Mock(return_value=16)))
        sock.emergency_latched = True
        for frame in (packet(3, 1), parameter(1, 0x7016, .2)):
            with self.assertRaisesRegex(RuntimeError, 'blocked after emergency'):
                sock.send(frame)
        sock.send(packet(4, 1))
        sock.send(request_frame(1))
        self.assertEqual(sock.sock.send.call_count, 2)

    def app(self):
        app = App.__new__(App)
        app.single_run_mode = True
        app.bus = SimpleNamespace(active=True, relax=Mock(), mit_center_return=object(),
                                  right_joint7_session=None, playback_session=None)
        app.root, app.status = Mock(), Mock()
        app.phase = 'DUAL RECORDING SAFE HOLD'
        app._cancel_recording_preflight = Mock()
        app.center_relax = Mock()
        return app

    def test_emergency_ignores_center_feedback_and_cleanup_latch(self):
        app = self.app()
        app.dual = SimpleNamespace(session=None, cleanup_error='gain restoration failed',
                                   returning=True, stop=Mock(side_effect=AssertionError('not emergency')))
        app.hihat = Mock(release_all=Mock(side_effect=OSError('disconnected')))
        app.hihat_calibration = Mock(close=Mock(side_effect=OSError('log closed')))
        app.emergency_relax()
        app.bus.relax.assert_called_once()
        app.dual.stop.assert_not_called()
        app.center_relax.assert_not_called()
        self.assertIsNone(app.bus.mit_center_return)
        self.assertTrue(app.emergency_latched)
        self.assertTrue(app.bus.emergency_relax_reason)
        app.start()
        self.assertIn('restart required', app.status.set.call_args.args[0])

    def test_owned_writers_exit_before_disable(self):
        app = self.app()
        events = []
        process = Mock(is_alive=Mock(return_value=False))
        process.join.side_effect = lambda *_: events.append('joined')
        app.smooth_playback_session = SimpleNamespace(process=process, cancel=Mock())
        app.bus.playback_session = app.smooth_playback_session
        app.bus.relax.side_effect = lambda: events.append('disabled')
        app.emergency_relax()
        self.assertEqual(events, ['joined', 'disabled'])
        self.assertIsNone(app.smooth_playback_session)

    def test_unresponsive_worker_escalates_with_bounded_joins(self):
        process = Mock(is_alive=Mock(side_effect=[True, True, False]))
        stop_writer(SimpleNamespace(process=process, cancel=Mock()))
        process.terminate.assert_called_once()
        process.kill.assert_called_once()
        self.assertEqual([c.args[0] for c in process.join.call_args_list], [.15, .3, .3])

    def test_live_writer_prevents_disable_race(self):
        app = self.app()
        app.smooth_playback_session = SimpleNamespace(process=Mock(is_alive=Mock(return_value=True)), cancel=Mock())
        app.emergency_relax()
        app.bus.relax.assert_not_called()
        self.assertEqual(app.phase, 'FAULT')
        self.assertIn('PHYSICAL POWER', app.status.set.call_args.args[0])

    def test_repeated_ctrl_c_is_not_another_center_request(self):
        app = self.app()
        app.request_safe_close()
        app.request_safe_close()
        app.center_relax.assert_called_once()
        app.bus.relax.assert_called_once()
        app.root.quit.assert_called_once()
        self.assertTrue(app.force_close)

    def test_power_loss_does_not_make_shutdown_wait_forever(self):
        app = self.app()
        app.bus.relax.side_effect = OSError('CAN offline')
        app.close_deadline = 1.
        with patch('time.monotonic', return_value=36.):
            app._safe_close_tick()
        app.bus.relax.assert_called_once()
        app.root.quit.assert_called_once()
        self.assertTrue(app.force_close)
        self.assertTrue(app.bus.active)  # Never claim hardware disabled on error.

    def test_stall_uses_independent_emergency_not_center(self):
        app = self.app()
        app.relax('MAJOR FAULT: STALL: motor 7')
        app.center_relax.assert_not_called()
        app.bus.relax.assert_called_once()


if __name__ == '__main__':
    unittest.main()
