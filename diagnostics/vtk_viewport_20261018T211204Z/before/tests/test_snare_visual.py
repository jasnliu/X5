"""Snare graph is read-only; dedicated calibration input stays instrument-isolated."""
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.audio import AudioReceiver, AudioSender
from camera_playback.audio_bridge import detector_command, forward_audio_message, route_tonor, validate_instrument
from camera_playback.audio_waveform import WaveformApp
from camera_playback.waveform import COLORS, LABELS, MARKER_LABELS, MARKER_DASHES, MARKER_ROWS, WaveformTimeline
from camera_playback.hihat_sync import HiHatSynchronizer
from camera_playback.hybrid_strike import HybridStatus
from camera_playback.mit_strike import Sample
from test_hybrid_workflow import app_fixture

ROOT=Path(__file__).resolve().parents[1]


class SnareVisualTests(unittest.TestCase):
    def event(self):
        return dict(event='HIT',instrument='snare',label='snare_hit',
                    event_monotonic=10.,score=.96,normality_score=81.)

    def test_exact_new_bundles_selected_for_all_three_instruments(self):
        for instrument,version in [('ride','ride_v4'),('hihat','hihat_v2'),('snare','snare_v2')]:
            # Command selection is independent of a local ST7 installation.
            # The live smoke separately loads the real installed model bundles.
            with patch('pathlib.Path.is_file',return_value=True):
                command=detector_command(ROOT.parent/'st7','pipewire',instrument)
            self.assertEqual(command[command.index('--config')+1],str(ROOT.parent/'st7/models'/version/'config.yaml'))
            self.assertEqual(command[command.index('--checkpoint')+1],str(ROOT.parent/'st7/models'/version/'best.pt'))
            self.assertEqual(command[command.index('--normality-artifact')+1],str(ROOT.parent/'st7/models'/version/'normality/reference.npz'))

    def test_snare_bridge_requires_matching_label_and_forwards_original_onset(self):
        sender=Mock()
        forward_audio_message(self.event(),sender,11.,'snare')
        sender.hit.assert_called_once_with(10.,11.,.96,81.)
        for event in ({},dict(self.event(),label='hihat_close'),dict(self.event(),instrument='ride')):
            with self.assertRaises(RuntimeError):validate_instrument(event,'snare')
        for instrument in ('ride','hihat'):
            with self.assertRaises(RuntimeError):forward_audio_message(self.event(),sender,11.,instrument)

    def test_snare_status_hits_and_progress_cannot_enter_either_control_receiver(self):
        with tempfile.TemporaryDirectory() as tmp:
            for instrument in ('ride','hihat'):
                receiver=AudioReceiver(Path(tmp)/instrument,instrument)
                snare=AudioSender(receiver.path,'snare')
                now=time.monotonic()
                try:
                    for state in ('starting','ready','error','stopped'):snare.status(state,'visual-only')
                    snare.hit(now-.1,now,.96,81.)
                    snare.progress(now-.2,now)
                    self.assertEqual(receiver.poll(),[])
                    self.assertEqual(receiver.state,'starting')
                    self.assertFalse(receiver.ready())
                    self.assertIsNone(receiver.finalized_at)
                finally:snare.close();receiver.close()

    def test_snare_graph_receiver_rejects_other_instruments_but_renders_snare(self):
        with tempfile.TemporaryDirectory() as tmp:
            receiver=AudioReceiver(Path(tmp)/'graph','snare')
            try:
                for instrument in ('ride','hihat','snare'):
                    sender=AudioSender(receiver.path,instrument)
                    sender.hit(10.,11.,.96,81.);sender.close()
                messages=receiver.poll()
                self.assertEqual(len(messages),1)
                self.assertEqual(messages[0]['instrument'],'snare')
                graph=WaveformTimeline()
                self.assertTrue(graph.add_hit(messages[0],11.))
                self.assertEqual(graph.markers[0].event_at,10.)
                self.assertEqual(graph.counts['snare'],1)
                self.assertEqual(graph.delays['snare'],1.)
            finally:receiver.close()

    def test_snare_cannot_select_ride_depth_or_retune_hihat(self):
        a=app_fixture()
        a.hybrid_session.status=HybridStatus(phase='coast',count=1,peak_drop=.07,
            released_at=10.,sample=Sample(-.07,-.5,0,2,10.05),at=10.05)
        a._process_sound_hit(dict(instrument='snare',event_at=10.06))
        self.assertIsNone(a.strike_hit_pending)
        a.hybrid_session.request.assert_not_called()
        for calibration in (False,True):
            sync=HiHatSynchronizer(Mock(),calibration=calibration);sync.start(10.)
            for name in ('ride','hihat','snare'):
                sync.feed(name,[dict(kind='hit',instrument='snare',event_at=10.06)],11.)
            self.assertEqual(sum(len(e) for e in sync.events.values()),0)
            self.assertEqual(sync.advance,0.)
            self.assertFalse(sync.locked)

    def test_snare_is_purple_exact_label_and_independent_overlap_row(self):
        self.assertEqual(LABELS['snare'],'snare')
        self.assertEqual(MARKER_LABELS['snare'],'snare')
        self.assertEqual(COLORS['snare'],'#9333ea')
        self.assertEqual(len(set(COLORS.values())),3)
        self.assertEqual(len(set(MARKER_DASHES.values())),3)
        self.assertEqual(len(set(MARKER_ROWS.values())),3)

    def test_snare_source_selection_never_changes_default_routing(self):
        with patch('camera_playback.audio_bridge.subprocess.run',return_value=SimpleNamespace(stdout='1\ttonor-test\tPipeWire\n')) as run:
            self.assertEqual(route_tonor('tonor-test',change_default=False),'tonor-test')
            self.assertEqual(run.call_count,1)
            self.assertEqual(run.call_args.args[0],['pactl','list','short','sources'])

    def test_snare_socket_setup_failure_is_visual_only(self):
        with patch('camera_playback.audio_waveform.WaveformWindow'), \
             patch('camera_playback.audio_waveform.AudioReceiver',side_effect=[Mock(),Mock(),OSError('snare socket unavailable')]):
            app=WaveformApp(Mock(),'ride','hihat',snare_socket='snare')
            self.assertEqual(set(app.receivers),{'ride','hihat'})
            self.assertIn('snare markers unavailable',app.errors[0])
            app.close()
