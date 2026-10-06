"""Regression: capture time, delayed ST7 delivery, and safe no-hit decisions."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from camera_playback.audio import AudioReceiver, AudioSender
from camera_playback.audio_bridge import forward_audio_message
from camera_playback.app import STRIKE_SOUND_WAIT_PHASE
from camera_playback.hybrid_strike import HybridStatus
from camera_playback.mit_strike import Sample
from test_hybrid_workflow import app_fixture

path = Path(__file__).resolve().parents[2]/'st7/cymbal_detector/audio_clock.py'
spec = importlib.util.spec_from_file_location('st7_audio_clock', path)
clock = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clock)


class AudioTimingTests(unittest.TestCase):
    def test_adc_time_ignores_stream_startup_and_callback_queue_delay(self):
        # Stream opened 200 ms after the text notification. Callback is 100 ms
        # later; ADC time, not either text/notification time, is authoritative.
        self.assertAlmostEqual(clock.block_start_monotonic(100.3, 40.1, 40.), 100.2)
        self.assertAlmostEqual(clock.event_monotonic(101., 12800, 16000, .1), 100.3)
        with self.assertRaises(ValueError): clock.block_start_monotonic(100., 0., 0.)

    def test_bridge_uses_exact_capture_onset_not_listener_or_notification(self):
        sender = Mock()
        forward_audio_message(dict(event='HIT', event_monotonic=100.1,
            time_since_start=61.92, notification_monotonic=102., score=.75,
            normality_score=59.2), sender, 102.3)
        sender.hit.assert_called_once_with(100.1, 102.3, .75, 59.2)
        with self.assertRaisesRegex(RuntimeError, 'ADC-monotonic'):
            forward_audio_message(dict(event='HIT', time_since_start=61.92,
                score=.75, normality_score=59.2), sender, 102.3)

    def test_real_datagrams_keep_hit_before_finalized_watermark(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = AudioReceiver(Path(tmp)/'audio.sock')
            s = AudioSender(Path(tmp)/'audio.sock')
            try:
                with patch('camera_playback.audio.time.monotonic', return_value=102.):
                    forward_audio_message(dict(event='HIT', event_monotonic=100.1,
                        score=.75, normality_score=59.2), s, 102.)
                    forward_audio_message(dict(event='AUDIO_PROGRESS',
                        finalized_monotonic=100.6, captured_monotonic=101.), s, 102.)
                    messages = r.poll()
                self.assertEqual([m['kind'] for m in messages], ['hit', 'progress'])
                self.assertEqual(r.finalized_at, 100.6)
                self.assertEqual(messages[0]['event_at'], 100.1)
            finally:
                s.close(); r.close()

    def waiting(self):
        a = app_fixture()
        a.phase = STRIKE_SOUND_WAIT_PHASE
        a.strike_attempt_returned_at = 10.4
        a.strike_sound_deadline = 13.4
        a.audio_receiver = SimpleNamespace(finalized_at=10.1)
        a._program_failure = Mock()
        a.hybrid_session.status = HybridStatus(phase='hold', ready=True, count=1,
            completed=1, released_at=10., returned_at=10.4, peak_drop=.09,
            sample=Sample(0, 0, .745, 2, 10.4), at=10.4)
        return a

    def test_delayed_hit_after_old_timeout_still_selects_original_attempt(self):
        a = self.waiting()
        a._advance_hybrid_audio_wait(12.)
        self.assertEqual(a.strike_index, 0)
        self.assertEqual(a.phase, STRIKE_SOUND_WAIT_PHASE)
        a._process_sound_hit(dict(event_at=10.1, detected_at=12.))
        self.assertTrue(a.continuous_strike_active)
        self.assertEqual(a.continuous_strike_degrees, 5)

    def test_no_hit_wait_requires_finalization_and_backlog_fails_safely(self):
        a = self.waiting()
        a._finish_no_hit_attempt = Mock()
        a._advance_hybrid_audio_wait(12.)
        a._finish_no_hit_attempt.assert_not_called()
        a.audio_receiver.finalized_at = 10.5
        a._advance_hybrid_audio_wait(12.1)
        a._finish_no_hit_attempt.assert_called_once()
        a = self.waiting()
        a._advance_hybrid_audio_wait(13.4)
        a._program_failure.assert_called_once()
        self.assertEqual(a.strike_index, 0)


if __name__ == '__main__':
    unittest.main()
