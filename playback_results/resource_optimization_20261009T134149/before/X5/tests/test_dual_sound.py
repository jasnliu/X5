"""Offline sound isolation: hi-hat never selects a depth or changes readiness."""
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.audio import AudioReceiver, AudioSender
from camera_playback.audio_bridge import forward_audio_message, validate_instrument
from camera_playback.app import App
from camera_playback.sound_monitor import HiHatSoundMonitor
from camera_playback.hybrid_strike import HybridStatus
from camera_playback.mit_strike import Sample
from test_hybrid_workflow import app_fixture


class DualSoundTests(unittest.TestCase):
    def test_model_events_cannot_cross_bridge_instruments(self):
        hit = dict(event='HIT', event_monotonic=10., score=.99, normality_score=90.)
        hihat = dict(hit, instrument='hihat', label='hihat_close')
        sender = Mock()
        with self.assertRaisesRegex(RuntimeError, 'different instrument'):
            forward_audio_message(hihat, sender, 11.)
        sender.hit.assert_not_called()
        forward_audio_message(hihat, sender, 11., 'hihat')
        sender.hit.assert_called_once_with(10., 11., .99, 90.)
        for message in (hit, dict(hihat, label='cymbal_hit')):
            with self.assertRaises(RuntimeError):validate_instrument(message, 'hihat')
        validate_instrument({}, 'ride')  # legacy ride output still valid

    def test_wrong_socket_cannot_change_ride_hit_readiness_or_watermark(self):
        with tempfile.TemporaryDirectory() as tmp:
            receiver = AudioReceiver(Path(tmp)/'ride.sock')
            wrong = AudioSender(receiver.path, instrument='hihat')
            ride = AudioSender(receiver.path)
            now = time.monotonic()
            try:
                wrong.status('ready', 'wrong model')
                wrong.hit(now-.1, now, .99, 90.)
                wrong.progress(now-.1, now)
                self.assertEqual(receiver.poll(), [])
                self.assertFalse(receiver.ready())
                self.assertIsNone(receiver.finalized_at)
                ride.status('ready', 'ride ready')
                ride.hit(now-.1, now, .8, 75.)
                ride.progress(now-.1, now)
                self.assertEqual([m['kind'] for m in receiver.poll()], ['status','hit','progress'])
                self.assertTrue(receiver.ready())
            finally:wrong.close();ride.close();receiver.close()

    def test_legacy_ride_cannot_enter_hihat_panel(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = AudioReceiver(Path(tmp)/'hihat.sock', instrument='hihat')
            ride = AudioSender(r.path)
            try:
                ride.hit(10., 11., .9, 90.)
                self.assertEqual(r.poll(), [])
            finally:ride.close();r.close()

    def test_hihat_hit_does_not_select_depth_but_ride_still_does(self):
        a = app_fixture()
        a.hybrid_session.status = HybridStatus(phase='coast', count=1,
            peak_drop=.07, released_at=10., sample=Sample(-.07, -.5, 0, 2, 10.05), at=10.05)
        a._process_sound_hit(dict(instrument='hihat', event_at=10.06))
        self.assertIsNone(a.strike_hit_pending)
        self.assertIsNone(a.strike_attempt_started_at)
        a.hybrid_session.request.assert_not_called()
        a._process_sound_hit(dict(instrument='ride', event_at=10.06))
        self.assertEqual(a.strike_hit_pending['degrees'], 5)

    def monitor(self, receiver):
        m = object.__new__(HiHatSoundMonitor)
        m.disabled=False;m.receiver=receiver;m.count=0
        m.last_hit=m.last_received_at=None
        return m

    def test_panel_counts_flashes_and_keeps_last_detection(self):
        r = SimpleNamespace(state='ready', detail='', ready=Mock(return_value=True),
            poll=Mock(return_value=[dict(kind='hit', instrument='hihat', score=.98, normality_score=81.)]))
        m=self.monitor(r)
        text, color, flash = m.poll_display(12.)
        self.assertIn('CLOSURE DETECTED', text)
        self.assertIn('Closures: 1', text)
        self.assertTrue(flash)
        r.poll.return_value=[]
        text, _, flash = m.poll_display(12.7)
        self.assertIn('READY', text)
        self.assertIn('0.980', text)
        self.assertFalse(flash)
        r.ready.return_value=False
        self.assertIn('STALE', m.poll_display(14.)[0])

    def test_fresh_real_heartbeat_is_not_treated_as_future_or_stale(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=AudioReceiver(Path(tmp)/'hihat.sock',instrument='hihat')
            s=AudioSender(r.path,instrument='hihat')
            try:
                m=self.monitor(r)
                s.status('ready','hihat v1')
                self.assertIn('READY',m.poll_display()[0])
            finally:s.close();r.close()

    def test_optional_socket_setup_failure_is_display_only(self):
        root=Mock()
        with (patch('camera_playback.sound_monitor.AudioReceiver',side_effect=OSError('cannot bind')),
              patch('camera_playback.sound_monitor.tk.StringVar'),
              patch('camera_playback.sound_monitor.tk.Label')):
            m=HiHatSoundMonitor(root,'unused.sock',before=Mock())
            self.assertIsNone(m.receiver)
            self.assertIn('beat unaffected',m.text.set.call_args.args[0])
            m.close()

    def test_hihat_errors_are_display_only_and_ride_remains_startable(self):
        m=self.monitor(SimpleNamespace(state='error', detail='optional detector lost',
                                       poll=Mock(return_value=[])))
        self.assertIn('beat unaffected', m.poll_display(12.)[0])
        a=app_fixture();a.sound_fault_detail=None
        a.audio_receiver=SimpleNamespace(ready=Mock(return_value=True))
        a.hihat_sound_monitor=m
        self.assertTrue(a._sound_ready_for_start())
        m.closed=False;m.root=Mock();m.text=Mock();m.label=Mock();m.normal_background='white'
        m.receiver.poll.side_effect=RuntimeError('optional malformed packet')
        m.tick()  # must not reach root's hardware callback-failure handler
        self.assertIn('beat unaffected', m.text.set.call_args.args[0])
        m.root.after.assert_called_once_with(50, m.tick)
        self.assertTrue(a._sound_ready_for_start())


if __name__ == '__main__':unittest.main()
