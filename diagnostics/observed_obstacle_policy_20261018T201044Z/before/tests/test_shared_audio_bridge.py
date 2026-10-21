"""Offline multiplexing, resource and lifecycle contracts; no physical devices."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from camera_playback.shared_audio_bridge import Router, shared_command
from camera_playback.resource_preflight import check_memory


class SharedAudioTests(unittest.TestCase):
    def setUp(self):
        self.senders = {name: Mock() for name in ('ride', 'hihat', 'snare')}
        self.router = Router(self.senders)
        self.labels = dict(ride='cymbal_hit', hihat='hihat_close', snare='snare_hit')

    def message(self, name, **fields):
        return dict(instrument=name, label=self.labels[name], **fields)

    def ready(self, name, now=10.):
        self.router.consume(self.message(name, status='listening',
            timestamp_clock='portaudio_adc_monotonic', normality={'enabled': True}), now)

    def test_same_explicit_bundles_one_interpreter(self):
        with patch('pathlib.Path.is_file', return_value=True):
            cmd = shared_command(Path('/test/st7'), 'pipewire', self.senders)
        self.assertEqual(cmd[1:3], ['-m', 'cymbal_detector.shared_listener'])
        bundles = [cmd[i+1:i+5] for i, arg in enumerate(cmd) if arg == '--model']
        for bundle, name, version in zip(bundles, self.senders, ['ride_v4','hihat_v2','snare_v2']):
            self.assertEqual(bundle[0], name)
            self.assertEqual(bundle[1:], [f'/test/st7/models/{version}/{asset}'
                             for asset in ('config.yaml','best.pt','normality/reference.npz')])

    def test_tagged_hits_stay_on_their_own_control_and_graph_sender(self):
        for name in self.senders:
            self.router.consume(self.message(name, event='HIT', event_monotonic=10.,
                                            score=.9, normality_score=80.), 11.)
            self.senders[name].hit.assert_called_once_with(10., 11., .9, 80.)
        self.assertTrue(all(s.hit.call_count == 1 for s in self.senders.values()))

    def test_readiness_needs_matching_listener_warmup_and_fresh_progress(self):
        for name in self.senders:
            self.ready(name)
        self.router.consume(self.message('ride', event='AUDIO_PROGRESS',
                            finalized_monotonic=12., captured_monotonic=12.4), 12.5)
        self.router.heartbeat(12.5)
        self.assertEqual(self.senders['ride'].status.call_args.args[0], 'ready')
        self.assertEqual(self.senders['snare'].status.call_args.args[0], 'starting')
        self.router.heartbeat(15.)
        self.assertEqual(self.senders['ride'].status.call_args.args[0], 'starting')

    def test_wrong_labels_unknown_instruments_and_missing_clocks_fail(self):
        for message in [dict(instrument='snare', label='hihat_close'),
                        dict(instrument='other'), self.message('ride', status='listening', normality={})]:
            with self.assertRaises(RuntimeError):
                self.router.consume(message, 10.)

    def test_shared_failure_notifies_every_control_receiver(self):
        self.router.status('error', 'shared child exited')
        for sender in self.senders.values():
            sender.status.assert_called_once_with('error', 'shared child exited')

    def test_memory_gate_uses_available_not_free_or_swap(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'meminfo'
            path.write_text('MemFree: 1 kB\nMemAvailable: 3145728 kB\nSwapFree: 0 kB\n')
            check_memory(path)
            path.write_text('MemAvailable: 1048576 kB\nSwapFree: 99999999 kB\n')
            with self.assertRaisesRegex(RuntimeError, 'No controllers started'):
                check_memory(path)
            path.write_text('MemTotal: 10000000 kB\n')
            with self.assertRaisesRegex(RuntimeError, 'Cannot verify'):
                check_memory(path)
