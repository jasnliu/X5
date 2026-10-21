"""Bass integration without CAN, a real serial device, microphone or motor."""
import fcntl
import math
import os
import pty
import select
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.bass_drum import BassDrumScheduler
from camera_playback.app import App
from camera_playback.hihat import HiHatController
from camera_playback.hihat_sync import TimingFault
from camera_playback.hihat_sync_runtime import HiHatSyncRuntime
from camera_playback.hybrid_strike import HybridStatus
from camera_playback import hybrid_workflow as hw
from test_hybrid_workflow import app_fixture


class BassSchedulerTests(unittest.TestCase):
    def test_all_quarters_selected_bpm_no_pickup_extras_duplicates_or_drift(self):
        for bpm in (20, 60, 87.5, 100, 120, 180):
            with self.subTest(bpm=bpm):
                period, epoch, rows = 60 / bpm, 10., []
                send = Mock()
                scheduler = BassDrumScheduler(send, period, rows.append)
                scheduler.tick(9., None)
                scheduler.tick(epoch - period / 3, epoch)  # Ride pickup.
                send.assert_not_called()
                for index in range(16):
                    target = epoch + index * period
                    now = target + .002 + (index % 3) * .001
                    send.return_value = now
                    scheduler.tick(now, epoch)
                    scheduler.tick(now, epoch)  # Duplicate GUI observation.
                    scheduler.tick(target + period * 2 / 3, epoch)  # Swing extra.
                    self.assertEqual(send.call_count, index + 1)
                    self.assertAlmostEqual(rows[-1]['target'], target)
                self.assertEqual([r['beat'] for r in rows], [1, 2, 3, 4] * 4)
                self.assertTrue(all(r['advance'] == 0. for r in rows))

    def test_invalid_or_overlapping_pulse_period_rejected(self):
        for period in (0., -.1, .05, math.inf, math.nan):
            with self.assertRaises(ValueError):
                BassDrumScheduler(Mock(), period)

    def test_late_deadline_never_sends_a_catch_up_burst(self):
        send = Mock(return_value=10.1)
        scheduler = BassDrumScheduler(send, .6)
        with self.assertRaisesRegex(TimingFault, 'refusing catch-up'):
            scheduler.tick(10.1, 10.)
        send.assert_not_called()

    def test_delivery_failure_or_invalid_timestamp_propagates(self):
        for actual in (10.1, 9.9, math.nan, math.inf):
            scheduler = BassDrumScheduler(lambda: actual, .6)
            with self.assertRaisesRegex(TimingFault, 'serial delivery'):
                scheduler.tick(10., 10.)
        scheduler = BassDrumScheduler(Mock(side_effect=OSError('serial failed')), .6)
        with self.assertRaisesRegex(OSError, 'serial failed'):
            scheduler.tick(10., 10.)


class BassRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.workflow_app = app_fixture()  # Load tuning before mocking timeline-file I/O.
        self.now = 10.
        self.rows = []
        self.app = SimpleNamespace(
            root=Mock(), hihat=Mock(), continuous_strike_active=True,
            continuous_stop_requested=False, audio_receiver=Mock(),
            hihat_sound_monitor=SimpleNamespace(receiver=Mock()),
            _request_continuous_stop=Mock())
        self.runtime = HiHatSyncRuntime(self.app)
        self.runtime.log = self.rows.append
        for target, options in (
            ('time.monotonic', dict(side_effect=lambda: self.now)),
            ('Path.open', dict(new=Mock())), ('Path.mkdir', dict(new=Mock())),
        ):
            patcher = patch('camera_playback.hihat_sync_runtime.' + target, **options)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.addCleanup(self.runtime.stop)

    def start(self, bpm=100, advance=0.):
        self.runtime.calibrated_advance = advance
        self.runtime.start(60 / bpm)

    def tick(self, now, *, observe=True):
        self.now = now
        if observe:
            self.runtime.observe(HybridStatus(swing=True), now)
        self.runtime.tick()

    def seed_pickup(self, bottom=10.1):
        self.now = bottom + .01
        self.runtime.observe(HybridStatus(swing=True, count=1, bottoms=1,
                                         bottom_at=bottom), self.now)
        self.runtime.tick()
        return bottom + self.runtime.engine.period / 3

    def test_calibration_never_kicks_then_swing_waits_for_pickup_and_beat_one(self):
        self.runtime.start_calibration(11.)
        self.tick(11.)
        self.tick(11.375)
        self.app.hihat.send_kick.assert_not_called()
        self.assertIsNone(self.runtime.bass_drum)
        self.runtime.engine.locked = True
        self.runtime.finish_calibration()
        self.runtime.start(.6)
        self.tick(11.5)  # No ride grid yet.
        epoch = self.seed_pickup(11.6)
        self.tick(epoch - .001)
        self.app.hihat.send_kick.assert_not_called()
        self.tick(epoch)
        self.app.hihat.send_kick.assert_called_once_with()

    def test_unchecked_keeps_hihat_grid_and_can_be_enabled_on_next_run(self):
        self.app.bass_drum_enabled = False
        self.start()
        epoch = self.seed_pickup()
        for i in range(8):
            self.tick(epoch + i * .6)
        self.assertIsNone(self.runtime.bass_drum)
        self.app.hihat.send_kick.assert_not_called()
        self.assertEqual([c.args[0] for c in self.app.hihat.send_state.call_args_list],
                         [False, True] * 4)
        self.assertFalse([r for r in self.rows if r['kind'] == 'bass_option'][-1]['enabled'])
        self.runtime.stop()
        self.app.bass_drum_enabled = True
        self.now = 20.
        self.start()
        epoch = self.seed_pickup(20.1)
        self.tick(epoch)
        self.app.hihat.send_kick.assert_called_once_with()

    def test_signed_hihat_advance_never_shifts_bass_and_open_close_stays_13_24(self):
        for advance in (-.1, 0., .1):
            with self.subTest(advance=advance):
                self.now = 10.
                self.rows.clear()
                self.app.hihat.reset_mock()
                self.start(bpm=87.5, advance=advance)
                epoch = self.seed_pickup()
                period = self.runtime.engine.period
                # Visit both clocks' due times and every triplet extra.
                targets = [epoch + i * period for i in range(8)]
                hat_due = [t - (advance if i else 0.) for i, t in enumerate(targets)]
                extras = [targets[i] + period * 2 / 3 for i in (1, 3, 5)]
                for now in sorted(set(targets + hat_due + extras)):
                    self.tick(now + 1e-9)
                    self.tick(now + 1e-9)
                bass = [r for r in self.rows if r['kind'] == 'bass_command']
                hats = [r for r in self.rows if r['kind'] == 'command']
                self.assertEqual(len(bass), 8)
                self.assertEqual(len(hats), 8)
                for row, target in zip(bass, targets):
                    self.assertAlmostEqual(row['at'], target)
                self.assertEqual([r['closed'] for r in hats], [False, True] * 4)
                self.assertEqual([r['advance'] for r in hats], [0.] + [advance] * 7)
                self.assertEqual(self.app.hihat.send_kick.call_count, 8)

    def test_missed_pickup_can_seed_from_first_main_beat_without_duplicate(self):
        self.start()
        self.now = 10.5
        status = HybridStatus(swing=True, main_count=1, main_at=10.5)
        self.runtime.observe(status, self.now)
        self.runtime.tick()
        self.runtime.observe(status, self.now)
        self.runtime.tick()
        self.app.hihat.send_kick.assert_called_once()

    def test_stop_and_restart_discard_old_bass_grid(self):
        self.start()
        epoch = self.seed_pickup()
        self.tick(epoch)
        self.runtime.stop()
        self.tick(epoch + .6)
        self.app.hihat.send_kick.assert_called_once()
        self.assertIsNone(self.runtime.bass_drum)
        self.start(bpm=120)
        self.tick(12.)
        self.app.hihat.send_kick.assert_called_once()
        epoch = self.seed_pickup(12.1)
        self.tick(epoch)
        self.assertEqual(self.app.hihat.send_kick.call_count, 2)
        self.assertEqual([r['beat'] for r in self.rows if r['kind'] == 'bass_command'], [1, 1])

    def test_stop_request_inactive_run_and_stale_status_cancel_bass_before_send(self):
        for condition in ('stop', 'inactive', 'stale'):
            with self.subTest(condition=condition):
                self.app.continuous_stop_requested = False
                self.app.continuous_strike_active = True
                self.now = 10.
                self.start()
                epoch = self.seed_pickup()
                if condition == 'stop':
                    self.app.continuous_stop_requested = True
                elif condition == 'inactive':
                    self.app.continuous_strike_active = False
                self.tick(epoch, observe=False)
                self.assertFalse(self.runtime.active)
                self.assertIsNone(self.runtime.bass_drum)
                self.app.hihat.send_kick.assert_not_called()
        self.app._request_continuous_stop.assert_called_once()
        self.app.hihat.release_all.assert_not_called()

    def test_bass_serial_failure_cancels_both_schedules_via_controlled_stop(self):
        self.start()
        epoch = self.seed_pickup()
        self.app.hihat.send_kick.side_effect = RuntimeError('serial failed')
        self.tick(epoch)
        self.app._request_continuous_stop.assert_called_once()
        self.assertFalse(self.runtime.active)
        self.assertIsNone(self.runtime.bass_drum)
        self.tick(epoch + .6)
        self.app.hihat.send_kick.assert_called_once()
        self.app.hihat.release_all.assert_not_called()

    def test_bass_deadline_fault_even_when_shifted_hihat_is_not_due(self):
        self.start(advance=-.2)
        epoch = self.seed_pickup()
        self.tick(epoch)
        self.tick(epoch + .7)  # Bass .1 s late, but hi-hat not due until +.8.
        self.app.hihat.send_kick.assert_called_once()
        self.app._request_continuous_stop.assert_called_once()
        self.assertIn('Bass drum deadline missed', self.app._request_continuous_stop.call_args.args[0])
        self.assertFalse(self.runtime.active)

    def exercise_workflow_serial(self, enabled):
        """Actual workflow -> scheduler -> serial controller -> PTY bytes."""
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        fcntl.fcntl(master, fcntl.F_SETFL, os.O_NONBLOCK)
        controller = HiHatController(os.ttyname(slave))
        self.addCleanup(controller.close)
        self.assertTrue(controller.connect(now=10.))
        controller.tick(10.31)
        os.read(master, 256)  # Startup H/S probe, never K.
        os.write(master, b'STOP -- all motors released.\n')
        controller.tick(10.32)
        self.assertTrue(controller.ready())
        controller.calibrated = True
        controller.start_sequence()
        self.app.hihat = controller
        a = self.workflow_app
        a._bass_drum_enabled = enabled
        a.continuous_strike_active, a.strike_active = True, False
        a.continuous_strike_degrees, a._strike_bpm = 10.5, 100.
        a.hihat, a.hihat_sync = controller, self.runtime
        a.root = self.app.root
        a.audio_receiver = self.app.audio_receiver
        a.hihat_sound_monitor = self.app.hihat_sound_monitor
        self.runtime.app = a
        self.runtime.calibrated_advance = 0.
        with patch.object(hw.snare, 'start'):
            hw.start_swing(a, a.swing_events)
        self.assertIn('bass on 1/2/3/4' if enabled else 'bass OFF (motor 1 idle)',
                      a.status.set.call_args.args[0])
        epoch = self.seed_pickup()
        for index in range(8):
            self.tick(epoch + index * .6 + 1e-9)
        expected = b'OKBK' * 4 if enabled else b'OB' * 4
        actual = b''
        # PTY delivery can split reads, especially without a following K byte.
        while len(actual) < len(expected) and select.select([master], [], [], .25)[0]:
            actual += os.read(master, 256)
        self.assertEqual(actual, expected)
        hw.request_stop(a)
        self.tick(epoch + 4.8)
        self.assertEqual(os.read(master, 256), b'O')
        self.assertFalse(controller.sequence_active)
        self.assertIsNone(self.runtime.bass_drum)

    def test_real_workflow_uses_shared_serial_and_stops_future_bass(self):
        self.exercise_workflow_serial(True)

    def test_unchecked_workflow_sends_no_motor1_bytes_but_retains_hihat(self):
        self.exercise_workflow_serial(False)


class BassOptionTests(unittest.TestCase):
    def test_default_enabled_and_setup_choice_committed(self):
        a = App.__new__(App)
        self.assertTrue(a.bass_drum_enabled)
        a.bass_drum_selected = Mock()
        a.bass_drum_selected.get.return_value = False
        a._apply_selected_bass_drum()
        self.assertFalse(a.bass_drum_enabled)
        a.bass_drum_selected.get.return_value = True
        self.assertFalse(a.bass_drum_enabled)  # Mutable widget is not runtime state.
        a._apply_selected_bass_drum()
        self.assertTrue(a.bass_drum_enabled)

    def test_active_run_locks_checkbox_without_reading_changed_variable(self):
        a = App.__new__(App)
        a.phase = 'SWING STRIKE RETURNING'; a.bus = Mock(active=True)
        a._bass_drum_enabled = False
        a.bpm_entry = Mock(); a.tempo_hint = Mock()
        a.bass_drum_checkbox = Mock(); a.bass_drum_selected = Mock()
        self.assertTrue(a._refresh_tempo_controls())
        a.bass_drum_checkbox.config.assert_called_once_with(state='disabled')
        a.bass_drum_selected.get.assert_not_called()
        self.assertFalse(a.bass_drum_enabled)

    def test_setup_text_reflects_selected_option_and_tempo(self):
        a = App.__new__(App); a._strike_bpm = 87.5
        template = 'swing + quarter-note bass at 100 BPM'
        self.assertEqual(a._beat_setup_text(template),'swing + quarter-note bass at 87.5 BPM')
        a._bass_drum_enabled = False
        self.assertEqual(a._beat_setup_text(template),'swing (bass off) at 87.5 BPM')


if __name__ == '__main__':
    unittest.main()
