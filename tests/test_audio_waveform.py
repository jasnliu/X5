"""Visual-only waveform bounds, timing, event fan-out and launch isolation."""
import ast
import json
from pathlib import Path
import runpy
import socket
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.audio import AudioReceiver
from camera_playback.audio_bridge import forward_audio_message
from camera_playback.audio_visual import MirroredAudioSender
from camera_playback.audio_waveform import MicrophoneCapture, configure_capture_source
from camera_playback.waveform import WaveformTimeline, capture_start, COLORS, MAX_MARKERS

ROOT=Path(__file__).resolve().parents[1]


def hit(instrument='ride', onset=100., delivered=100.8):
    return dict(kind='hit', instrument=instrument, event_at=onset, detected_at=delivered,
                score=.95, normality_score=80.)


class TimelineTests(unittest.TestCase):
    def test_late_marker_uses_original_onset_and_scrolls_with_audio(self):
        t=WaveformTimeline(10.)
        self.assertTrue(t.add_hit(hit(),100.85))
        m=t.markers[0]
        self.assertEqual(m.event_at,100.)
        self.assertAlmostEqual(t.fraction(m.event_at,100.85),.915)
        self.assertAlmostEqual(t.fraction(m.event_at,101.85),.815)
        self.assertAlmostEqual(t.delays['ride'],.8)

    def test_same_onset_keeps_both_instruments_and_correct_colors(self):
        t=WaveformTimeline()
        t.add_hit(hit(),101.)
        t.add_hit(hit('hihat',delivered=100.6),101.)
        self.assertEqual(len(t.markers),2)
        self.assertEqual(COLORS,dict(ride='#dc2626',hihat='#2563eb'))
        self.assertFalse(t.add_hit(hit(),101.))

    def test_out_of_order_delayed_hits_prune_by_onset_not_receipt_order(self):
        t=WaveformTimeline()
        t.add_hit(hit(onset=105.,delivered=105.2),106.)
        t.add_hit(hit('hihat',onset=100.,delivered=105.8),106.)
        t.prune(111.)
        self.assertEqual([m.event_at for m in t.markers],[105.])

    def test_invalid_future_and_expired_markers_are_not_drawn_at_now(self):
        t=WaveformTimeline()
        for msg in (hit(onset=80.), hit(onset=float('nan')), hit(onset=110.),
                    hit(delivered=110.),hit('other'),hit(onset=True)):
            self.assertFalse(t.add_hit(msg,101.),msg)

    def test_waveform_and_markers_have_hard_memory_bounds(self):
        t=WaveformTimeline()
        for i in range(5000):
            t.add_audio(i*.02,np.zeros(320))
        self.assertLessEqual(len(t.blocks),550)
        for i in range(1000):
            t.add_hit(hit(onset=100.+i*.001,delivered=101.1),102.)
        self.assertEqual(len(t.markers),MAX_MARKERS)
        t.prune(120.)
        self.assertEqual(len(t.blocks),0)
        self.assertEqual(len(t.markers),0)

    def test_min_max_envelope_preserves_single_sample_transients(self):
        t=WaveformTimeline()
        samples=np.zeros(320,dtype=np.float32);samples[17]=.9;samples[18]=-.8
        t.add_audio(100.,samples)
        low,high=t.envelope(101.,1000)
        self.assertAlmostEqual(np.max(high),.9,places=6)
        self.assertAlmostEqual(np.min(low),-.8,places=6)
        self.assertTrue(np.isinf(high[0]))  # Uncaptured time is not drawn as silence.

    def test_absolute_capture_gaps_are_not_compressed(self):
        t=WaveformTimeline()
        t.add_audio(100.,np.ones(320))
        t.add_audio(102.,np.ones(320))
        low,_=t.envelope(103.,1000)
        self.assertTrue(np.isfinite(low[700:703]).any())
        self.assertTrue(np.isinf(low[750:850]).all())
        self.assertTrue(np.isfinite(low[900:903]).any())

    def test_adc_mapping_matches_st7_including_callback_delay(self):
        source=ROOT.parent/'st7/cymbal_detector/audio_clock.py'
        namespace=runpy.run_path(str(source))
        for values in ((100.3,40.1,40.),(500.,300.,299.91)):
            self.assertEqual(capture_start(*values),namespace['block_start_monotonic'](*values))
        self.assertAlmostEqual(capture_start(100.3,40.1,40.),100.2)
        for values in ((1.,0.,0.),(float('nan'),1.,1.),(5.,10.,8.)):
            with self.assertRaises(ValueError):capture_start(*values)


class CopyTests(unittest.TestCase):
    def pair(self,tmp,instrument='ride'):
        control=AudioReceiver(Path(tmp)/'control',instrument)
        visual=AudioReceiver(Path(tmp)/'visual',instrument)
        sender=MirroredAudioSender(control.path,instrument,visual.path)
        for obj in (control,visual,sender):self.addCleanup(obj.close)
        return control,visual,sender

    def test_original_control_payload_and_visual_hit_are_identical(self):
        with tempfile.TemporaryDirectory() as tmp:
            control,visual,sender=self.pair(tmp)
            forward_audio_message(dict(event='HIT',event_monotonic=100.,score=.95,normality_score=80.),sender,100.8)
            a,b=control.poll(),visual.poll()
            self.assertEqual(a,b)
            self.assertEqual(a[0]['event_at'],100.)
            self.assertEqual(a[0]['detected_at'],100.8)
            now=time.monotonic()
            sender.status('ready','original');sender.progress(now-.5,now)
            self.assertEqual([r['kind'] for r in control.poll()],['status','progress'])
            self.assertEqual([r['kind'] for r in visual.poll()],['status'])
            # Cleanup before TemporaryDirectory removes socket names.
            control.close();visual.close();sender.close()

    def test_closed_missing_or_failed_viewer_never_changes_control_delivery(self):
        with tempfile.TemporaryDirectory() as tmp:
            control,visual,sender=self.pair(tmp)
            visual.close()
            sender.hit(100.,100.8,.95,80.)
            self.assertEqual(len(control.poll()),1)
            real=sender.visual_socket
            for error in (BlockingIOError(),ConnectionRefusedError(),OSError('broken visual socket')):
                sender.visual_socket=Mock();sender.visual_socket.sendto.side_effect=error
                sender.hit(100.,100.8,.95,80.)
                self.assertEqual(len(control.poll()),1)
            sender.visual_socket=real
            control.close();sender.close()

    def test_unread_visual_queue_drops_copies_without_blocking_control(self):
        with tempfile.TemporaryDirectory() as tmp:
            control,visual,sender=self.pair(tmp)
            sender.visual_socket.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,2048)
            start=time.monotonic()
            for _ in range(100):
                sender.hit(100.,100.8,.95,80.)
                self.assertEqual(len(control.poll()),1)
            self.assertLess(time.monotonic()-start,1.)
            self.assertLess(len(visual.poll()),100)
            control.close();visual.close();sender.close()

    def test_hihat_copy_cannot_enter_ride_receiver(self):
        with tempfile.TemporaryDirectory() as tmp:
            control,visual,sender=self.pair(tmp,'hihat')
            sender.hit(100.,100.7,.95,80.)
            self.assertEqual(visual.poll()[0]['instrument'],'hihat')
            self.assertEqual(control.poll()[0]['instrument'],'hihat')
            control.close();visual.close();sender.close()


class CaptureTests(unittest.TestCase):
    def test_capture_queue_bounded_and_keeps_adc_times_after_drop(self):
        c=MicrophoneCapture()
        for i in range(150):
            with patch('camera_playback.audio_waveform.time.monotonic',return_value=100.+i*.02):
                c.callback(np.ones((320,1)),320,SimpleNamespace(currentTime=20.,inputBufferAdcTime=19.9),False)
        blocks=c.poll()
        self.assertEqual(len(blocks),100)
        self.assertEqual(c.dropped_blocks,50)
        self.assertAlmostEqual(blocks[-1][0],102.88)

    def test_bad_adc_clock_and_overflow_do_not_invent_audio(self):
        c=MicrophoneCapture()
        c.callback(np.ones((320,1)),320,SimpleNamespace(currentTime=0.,inputBufferAdcTime=0.),False)
        c.callback(np.ones((320,1)),320,SimpleNamespace(currentTime=20.,inputBufferAdcTime=19.9),'overflow')
        self.assertEqual(c.poll(),[])
        self.assertEqual(c.dropped_blocks,2)

    def test_capture_routing_is_read_only_and_explicit_not_default_source_change(self):
        with patch('camera_playback.audio_waveform.subprocess.run',return_value=SimpleNamespace(stdout='1\ttonor-test\tPipeWire\n')) as run, \
             patch.dict('os.environ',{},clear=True):
            self.assertEqual(configure_capture_source('tonor-test'),'tonor-test')
            run.assert_called_once_with(['pactl','list','short','sources'],check=True,capture_output=True,text=True)


class LaunchTests(unittest.TestCase):
    def graph(self,mode):
        processes=[];handlers=[];actions=[]
        modules={n:ModuleType(n) for n in ('launch','launch.actions','launch.event_handlers','launch.events','launch_ros','launch_ros.actions')}
        def process(**kw):
            p=SimpleNamespace(**kw);processes.append(p);return p
        def description(items):
            actions.extend(items);return SimpleNamespace(add_action=actions.append)
        def handler(**kw):
            handlers.append(kw);return kw
        modules['launch'].LaunchDescription=description
        modules['launch'].LaunchService=Mock(return_value=Mock(run=Mock(return_value=0)))
        modules['launch.actions'].ExecuteProcess=process
        modules['launch.actions'].EmitEvent=Mock()
        modules['launch.actions'].RegisterEventHandler=lambda x:x
        modules['launch.event_handlers'].OnProcessExit=handler
        modules['launch.events'].Shutdown=Mock()
        modules['launch_ros.actions'].Node=lambda **kw:SimpleNamespace(**kw)
        with patch.dict(sys.modules,modules),patch.object(sys,'argv',['launch',*mode]), \
             patch('pathlib.Path.is_file',return_value=True),patch('camera_playback.audio_bridge.detector_command'):
            with self.assertRaises(SystemExit):runpy.run_path(str(ROOT/'launch_right_camera_playback.py'),run_name='__main__')
        return actions,handlers

    def test_visual_process_is_unwatched_and_copies_use_separate_sockets(self):
        actions,handlers=self.graph(['--hardware'])
        viewer=next(x for x in actions if 'camera_playback.audio_waveform' in getattr(x,'cmd',[]))
        self.assertFalse(any(h['target_action'] is viewer for h in handlers))
        bridges=[x for x in actions if 'camera_playback.audio_bridge' in getattr(x,'cmd',[])]
        self.assertEqual(len(bridges),2)
        for bridge,flag in zip(bridges,('--ride-socket','--hihat-socket')):
            self.assertEqual(bridge.cmd[bridge.cmd.index('--visual-socket')+1],viewer.cmd[viewer.cmd.index(flag)+1])
            self.assertNotEqual(bridge.cmd[bridge.cmd.index('--socket')+1],bridge.cmd[bridge.cmd.index('--visual-socket')+1])
        app=next(x for x in actions if 'camera_playback.app' in getattr(x,'cmd',[]))
        self.assertNotIn('--visual-socket',app.cmd)

    def test_test_mode_still_has_no_capture_or_waveform_process(self):
        actions,_=self.graph(['--test'])
        self.assertFalse(any('camera_playback.audio_waveform' in getattr(x,'cmd',[]) for x in actions))

    def test_viewer_has_no_control_imports_or_audio_file_writer(self):
        for name in ('audio_waveform.py','waveform.py','audio_visual.py'):
            tree=ast.parse((ROOT/'camera_playback'/name).read_text())
            imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
            self.assertFalse(any(any(word in module for word in ('motors','app','hybrid','hihat','wave','soundfile'))
                                 for module in imports if module not in ('waveform',)))


if __name__=='__main__':unittest.main()
