"""Background validation and fresh-before-enable guards; no physical devices."""
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App
from camera_playback.recording_preflight import RecordingPreflight
from camera_playback.hihat_calibration import HiHatCalibration
from camera_playback.hihat_calibration_runtime import HiHatCalibrationRuntime


class WorkerTests(unittest.TestCase):
    def test_slow_validation_returns_immediately_and_never_calls_gui(self):
        entered, release = threading.Event(), threading.Event()
        main_thread = threading.get_ident()
        def loader(*args, progress, **kwargs):
            self.assertNotEqual(threading.get_ident(), main_thread)
            progress('checking geometry')
            entered.set()
            self.assertTrue(release.wait(3))
            return 'validated'
        start = time.monotonic()
        job = RecordingPreflight(playback_speed=.8, loader=loader)
        self.addCleanup(release.set)
        self.assertLess(time.monotonic()-start, .2)
        self.assertTrue(entered.wait(1))
        self.assertEqual(job.poll(), [('progress', 'checking geometry')])
        self.assertEqual(job.poll(), [])
        release.set(); job.thread.join(2)
        self.assertEqual(job.poll(), [('result', 'validated')])

    def test_failure_is_queued_not_raised_on_ui_or_worker(self):
        job = RecordingPreflight(playback_speed=.8, loader=Mock(side_effect=ValueError('bad path')))
        job.thread.join(2)
        self.assertEqual(job.poll(), [('error', 'bad path')])

    def test_cancel_discards_late_result(self):
        release = threading.Event()
        def loader(*args, **kwargs):
            release.wait(2)
            return 'late'
        job = RecordingPreflight(playback_speed=.8, loader=loader)
        job.cancel(); release.set(); job.thread.join(2)
        self.assertEqual(job.poll(), [])


class AppTests(unittest.TestCase):
    def fixture(self):
        a = App.__new__(App)
        a.hybrid_enabled = a.hardware = True
        a.test_mode = a.hardware_test_mode = a.recording_only = False
        a.close_requested = False
        a.recording_preflight = None
        a.hihat_calibration = SimpleNamespace(busy=False, ready=True)
        a.bus = Mock(active=False); a.bus.fresh.return_value = True
        a.model = a.zone = object()
        a.lower = np.full(7,-3.); a.upper = np.full(7,3.)
        a.center_goal = np.zeros(7)
        a.ik = SimpleNamespace(speed=.4)
        for name in ('root','status','_refresh_buttons','_show_recording_loading',
                     '_hide_recording_loading','_install_recording','recording_status',
                     'recording_label'):
            setattr(a,name,Mock())
        a.playback_trajectory = 'old'
        return a

    def test_calibrated_then_load_disables_run_until_queued_completion(self):
        a=self.fixture(); job=Mock()
        with patch('camera_playback.app.RecordingPreflight',return_value=job):
            self.assertTrue(a.load_recording('record3.json'))
        self.assertIsNone(a.playback_trajectory)
        a.start()
        self.assertIn('validation',a.status.set.call_args.args[0])
        self.assertFalse(a.load_recording('record2.json'))
        job.poll.return_value=[('progress','checking'),('result','new')]
        a._poll_recording_preflight('record3.json')
        self.assertIsNone(a.recording_preflight)
        a._install_recording.assert_called_once_with('new')

    def test_validation_error_cannot_reuse_old_path_or_open_modal_dialog(self):
        a=self.fixture(); job=Mock()
        with patch('camera_playback.app.RecordingPreflight',return_value=job):
            a.load_recording('bad.json')
        job.poll.return_value=[('error','invalid recording')]
        with patch('camera_playback.app.messagebox.showerror') as dialog:
            a._poll_recording_preflight('bad.json')
        dialog.assert_not_called()
        self.assertIsNone(a.playback_trajectory)
        a._install_recording.assert_not_called()

    def test_cancel_and_close_discard_completion(self):
        a=self.fixture(); job=a.recording_preflight=Mock()
        a.close_requested=True
        a._poll_recording_preflight('record3.json')
        job.cancel.assert_called_once()
        job.poll.assert_not_called()
        a._install_recording.assert_not_called()

    def start_fixture(self):
        a=self.fixture()
        a.phase='PLAYBACK PREFLIGHTED'; a.ik_refresh_deadline=12.
        a.hihat_start_query_at=None
        a.receiver=a.audio_receiver=Mock()
        a._camera_ready_for_start=Mock(return_value=True)
        a._sound_ready_for_start=Mock(return_value=True)
        a._hihat_ready_for_start=Mock(return_value=True)
        a.hihat=Mock(telemetry_at=9.9)
        a.hihat.ready.return_value=True; a.hihat_fault_detail=None
        return a

    def test_no_arm_enable_until_response_newer_than_run_request(self):
        a=self.start_fixture()
        with patch('camera_playback.app.time.monotonic',return_value=10.) as clock, \
             patch('camera_playback.app.JointGoalApp.start') as enable:
            a._start_preflighted_recording()
            a.hihat.query_status.assert_called_once()
            enable.assert_not_called()
            a._start_preflighted_recording()  # old telemetry is not a reply
            enable.assert_not_called()
            a.hihat.telemetry_at=10.02
            clock.return_value=10.03
            a._start_preflighted_recording()
            enable.assert_called_once_with(a)

    def test_missing_reply_times_out_without_arm_enable(self):
        a=self.start_fixture(); a.hihat_start_query_at=10.
        with patch('camera_playback.app.time.monotonic',return_value=12.01), \
             patch('camera_playback.app.JointGoalApp.start') as enable:
            a._start_preflighted_recording()
        enable.assert_not_called()
        self.assertEqual(a.phase,'READY')
        self.assertIn('did not all become ready',a.status.set.call_args.args[0])

    def test_future_timestamp_is_not_a_fresh_start_reply(self):
        a=self.start_fixture(); a.hihat_start_query_at=10.
        a.hihat.telemetry_at=11.
        with patch('camera_playback.app.time.monotonic',return_value=10.1), \
             patch('camera_playback.app.JointGoalApp.start') as enable:
            a._start_preflighted_recording()
        enable.assert_not_called()


class StatusSafetyTests(unittest.TestCase):
    def fixture(self, stamp):
        c=Mock(state='ready',telemetry_at=stamp,telemetry={'angle':90})
        c.ready.return_value=True
        app=SimpleNamespace(hihat=c,hihat_sound_monitor=SimpleNamespace(receiver=Mock()),
            bus=SimpleNamespace(active=False),result_status=Mock(),result_label=Mock(),status=Mock())
        r=HiHatCalibrationRuntime.__new__(HiHatCalibrationRuntime)
        r.app=app; r.closed=False; r.controller_ready_at=0.; r.next_query=0.
        r.failure_reported=False
        r.engine=HiHatCalibration(c)
        r.engine.phase='READY';r.engine.angle=r.engine.selected_angle=90
        return r

    def test_real_missing_status_after_calibration_still_faults(self):
        r=self.fixture(0.)
        r.tick(1.51)
        self.assertEqual(r.engine.phase,'FAULT')
        self.assertEqual(r.engine.failure,'Calibrated ESP32 status became stale')
        r.app.hihat.query_status.assert_called_once()
        # A fresh reply cannot silently clear a genuine latched fault.
        r.app.hihat.telemetry_at=1.52
        r.tick(1.52)
        self.assertEqual(r.engine.phase,'FAULT')

    def test_queried_current_status_preserves_selected_angle(self):
        r=self.fixture(10.)
        r.tick(10.1)
        self.assertTrue(r.ready)
        self.assertEqual(r.engine.selected_angle,90)
        self.assertIsNone(r.engine.failure)


if __name__=='__main__': unittest.main()
