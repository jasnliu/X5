"""No hardware: finite angle search, delayed audio and startup integration."""
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.hihat import HiHatController
from camera_playback.hihat_calibration import HiHatCalibration, ANGLES
from camera_playback.hihat_calibration_runtime import HiHatCalibrationRuntime


class Controller:
    counts_for = staticmethod(HiHatController.counts_for)

    def __init__(self):
        self.now = 0.
        self.commands = []
        self.angle_ack = None
        self.target_degrees = 100
        self.calibrated = False
        self.state, self.detail = 'ready', ''
        self.telemetry = dict(angle=100, counts=149, pos=0, target=0, released=1, opened=1, fault=0)
        self.telemetry_at = 0.
        self.auto_move = True

    def ready(self):
        return self.state == 'ready'

    def query_status(self):
        self.commands.append((self.now, 'Q'))

    def set_calibration_angle(self, angle):
        assert angle in ANGLES
        self.commands.append((self.now, f'A{angle}'))
        self.telemetry.update(angle=angle, counts=self.counts_for(angle))
        self.angle_ack = (angle, self.counts_for(angle), self.now)

    def calibration_edge(self, closed):
        self.commands.append((self.now, 'B' if closed else 'O'))
        target = self.telemetry['counts'] if closed else 0
        self.telemetry.update(target=target, released=int(not closed), opened=int(not closed))
        if self.auto_move:
            self.telemetry['pos'] = target

    def release_all(self):
        self.commands.append((self.now, 'S'))
        self.telemetry.update(target=0, released=1)


class SearchTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller()
        self.events = []
        self.e = HiHatCalibration(self.c, self.events.append)
        self.e.start(0.)

    def step(self, dt=.05, *, progress=True, audio=True, fresh=True):
        self.c.now = round(self.c.now + dt, 6)
        if fresh:
            self.c.telemetry_at = self.c.now
        if progress:
            self.e.feed([dict(kind='progress', instrument='hihat', finalized_at=self.c.now-.6)], self.c.now)
        self.e.tick(self.c.now, audio_ready=audio)

    def until(self, predicate, **options):
        for _ in range(1200):
            if predicate():
                return
            self.step(**options)
        self.fail('state transition timed out: ' + self.e.phase)

    def hit(self, onset=None, instrument='hihat'):
        self.e.feed([dict(kind='hit', instrument=instrument,
                         event_at=self.c.now if onset is None else onset,
                         detected_at=self.c.now, score=.8, normality_score=75.)], self.c.now)

    def test_six_attempts_no_sound_fault_at_115_never_120(self):
        self.until(lambda: self.e.phase == 'FAULT')
        self.assertEqual([r['angle'] for r in self.events if r['kind'] == 'close'], list(ANGLES))
        self.assertIn('115°', self.e.failure)
        self.assertFalse(self.e.ready)
        self.assertFalse(self.c.calibrated)
        self.assertEqual(self.c.telemetry['pos'], 0)
        self.assertEqual(self.c.commands[-1][1], 'S')

    def test_first_hit_latches_but_always_finishes_two_second_hold_and_open(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED' and self.e.angle == 95)
        self.hit()
        self.assertFalse(self.e.ready)
        self.until(lambda: self.e.ready)
        self.assertEqual(self.e.selected_angle, 95)
        self.assertEqual(self.c.target_degrees, 95)
        self.assertTrue(self.c.calibrated)
        closed = next(r for r in self.events if r['kind'] == 'closed' and r['angle'] == 95)
        opened_command = next(r for r in self.events if r['kind'] == 'open' and r['angle'] == 95)
        opened = next(r for r in self.events if r['kind'] == 'opened' and r['angle'] == 95)
        selected = next(r for r in self.events if r['kind'] == 'selected')
        self.assertGreaterEqual(opened_command['at'] - closed['at'], 2.)
        self.assertGreaterEqual(selected['at'] - opened['at'], 2.)
        self.assertEqual([r['angle'] for r in self.events if r['kind'] == 'close'], [90,95])

    def test_late_notification_after_open_still_selects_onset_of_closure(self):
        self.until(lambda: self.e.phase == 'HOLDING OPEN' and self.e.angle == 90)
        self.hit(self.e.close_at + .2)
        self.until(lambda: self.e.ready)
        self.assertEqual(self.e.selected_angle, 90)

    def test_hit_at_maximum_is_success_not_fault(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED' and self.e.angle == 115)
        self.hit()
        self.until(lambda: self.e.ready)
        self.assertEqual(self.e.selected_angle, 115)
        self.assertIsNone(self.e.failure)

    def test_normal_passive_coast_after_verified_zero_return_is_not_false_fault(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.hit()
        self.until(lambda: self.e.phase == 'OPENING')
        self.c.telemetry['pos'] = -19  # Actual observed passive coast after release.
        self.until(lambda: self.e.ready)
        self.assertEqual(self.e.selected_angle, 90)
        self.assertIsNone(self.e.failure)

    def test_released_alone_cannot_fake_a_verified_open_return(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.until(lambda: self.e.phase == 'OPENING')
        self.c.telemetry['opened'] = 0
        self.until(lambda: self.e.failure is not None)
        self.assertIn('return', self.e.failure)

    def test_ride_old_future_and_opening_hits_do_not_select(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.hit(instrument='ride')
        self.hit(self.e.close_at - 1)
        self.hit(self.c.now + 5)
        self.until(lambda: self.e.phase == 'HOLDING OPEN')
        self.hit(self.e.open_at)
        self.hit()
        self.assertIsNone(self.e.hit)
        self.until(lambda: self.e.angle == 95)
        self.assertFalse(self.e.ready)

    def test_missing_progress_is_a_fault_not_a_miss_or_deeper_trial(self):
        self.until(lambda: self.e.phase == 'FAULT', progress=False)
        self.assertIn('finalize', self.e.failure)
        self.assertEqual(self.e.angle, 90)

    def test_slow_progress_keeps_open_before_next_trial(self):
        self.until(lambda: self.e.phase == 'HOLDING OPEN', progress=False)
        for _ in range(45): self.step(progress=False)
        self.assertEqual(self.e.phase, 'HOLDING OPEN')
        self.assertEqual(self.e.angle, 90)
        self.e.feed([dict(kind='progress', instrument='hihat', finalized_at=self.e.open_at)], self.c.now)
        self.step(progress=False)
        self.assertEqual(self.e.angle, 95)

    def test_cancel_returns_to_zero_and_cannot_restart(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.e.abort('cancelled', self.c.now)
        self.assertEqual(self.e.phase, 'FAULT OPENING')
        self.until(lambda: self.e.phase == 'FAULT')
        self.assertEqual(self.c.commands[-1][1], 'S')
        self.assertEqual(self.c.telemetry['pos'], 0)
        with self.assertRaises(RuntimeError): self.e.start(self.c.now)

    def test_audio_failure_stops_increasing_and_opens(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.step(audio=False)
        self.until(lambda: self.e.phase == 'FAULT', audio=False)
        self.assertIn('detector', self.e.failure)
        self.assertEqual(self.e.angle, 90)

    def test_encoder_loss_and_failed_return_are_bounded(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.step(dt=.7, fresh=False)
        self.step(fresh=False)
        self.assertEqual(self.e.phase, 'FAULT')
        self.assertEqual(self.c.commands[-1][1], 'S')

    def test_missing_angle_ack_never_closes(self):
        self.until(lambda: self.e.phase == 'CONFIGURING')
        self.c.angle_ack = None
        self.until(lambda: self.e.phase == 'FAULT')
        self.assertIn('acknowledgment', self.e.failure)
        self.assertNotIn('B', [command for _,command in self.c.commands])

    def test_serial_unplug_cannot_prevent_terminal_fault(self):
        self.until(lambda: self.e.phase == 'HOLDING CLOSED')
        self.c.state = 'error'
        self.c.release_all = Mock(side_effect=OSError('serial unplugged'))
        self.step()
        self.assertEqual(self.e.phase, 'FAULT')
        self.assertIsNotNone(self.e.failure)
        self.assertTrue(any(e['kind'] == 'release_unconfirmed' for e in self.events))

    def test_stalled_closure_aborts_instead_of_indefinite_hold(self):
        self.until(lambda: self.e.phase == 'CONFIGURING')
        self.c.auto_move = False
        self.until(lambda: self.e.phase == 'FAULT')
        self.assertIn('reach', self.e.failure)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.c = Controller()
        self.app = SimpleNamespace(hihat=self.c, bus=SimpleNamespace(active=False),
            hihat_sound_monitor=SimpleNamespace(receiver=SimpleNamespace(state='ready',ready=lambda now: True)),
            hihat_fault_detail=None, result_status=Mock(), result_label=Mock(), status=Mock(), fail=Mock())
        self.r = HiHatCalibrationRuntime(self.app, now=0., directory=self.temp.name)
        self.addCleanup(self.r.close)

    def step(self, now):
        self.c.now = self.c.telemetry_at = now
        self.r.tick(now)

    def test_first_ready_tick_starts_without_button_countdown_or_notification(self):
        with patch('threading.Thread') as thread, patch('urllib.request.urlopen') as network:
            self.step(.1)
            thread.assert_not_called()
            network.assert_not_called()
        self.assertEqual(self.r.engine.phase, 'INITIAL OPENING')
        self.assertIn((.1, 'O'), self.c.commands)
        self.assertTrue(self.r.busy)
        self.app.fail.assert_not_called()

    def test_controller_and_protocol_readiness_are_still_required(self):
        self.c.state = 'starting'
        self.step(.1)
        self.assertEqual(self.c.commands, [])
        self.c.state = 'ready'
        status, self.c.telemetry = self.c.telemetry, None
        self.step(.2)
        self.assertEqual(self.r.engine.phase, 'WAITING')
        self.assertTrue(all(cmd == 'Q' for _,cmd in self.c.commands))
        self.c.telemetry = status
        self.step(.3)
        self.assertEqual(self.r.engine.phase, 'INITIAL OPENING')
        self.assertIn((.3, 'O'), self.c.commands)

    def test_cancel_before_ready_never_moves_later(self):
        self.r.cancel()
        self.step(20.)
        self.assertEqual(self.c.commands, [])

    def test_missing_firmware_capability_faults_without_close(self):
        self.c.telemetry = None
        self.step(.1)
        self.step(3.2)
        self.assertIn('firmware', self.r.engine.failure)
        self.assertTrue(all(cmd == 'Q' for _,cmd in self.c.commands))

    def test_failure_uses_existing_arm_safe_failure_path_once(self):
        self.app.bus.active = True
        self.app.hihat_sound_monitor.receiver.state = 'error'
        self.step(1.)
        self.step(2.)
        self.app.fail.assert_called_once()

    def test_waits_for_warm_model_without_attempts(self):
        self.app.hihat_sound_monitor.receiver.ready = lambda now: False
        self.step(30.)
        self.assertEqual(self.r.engine.phase, 'WAITING')
        self.assertTrue(all(cmd == 'Q' for _,cmd in self.c.commands))
        self.step(91.)
        self.assertIn('startup timed out', self.r.engine.failure)


if __name__ == '__main__': unittest.main()
