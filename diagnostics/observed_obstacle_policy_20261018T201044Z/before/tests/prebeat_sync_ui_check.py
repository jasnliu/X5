"""Real Tk hardware-mode calibration/handoff widgets with all device I/O mocked.

Only the post-search endpoint is injected; this is not physical or full-startup
verification. Timers are manually clocked. No CAN/serial/video/audio opens.
"""
from dataclasses import replace
import math
from pathlib import Path
import socket
import sys
import tempfile
import time
from unittest.mock import Mock, patch


def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden')
    if event=='open' and isinstance(args[0],str) and args[0].startswith(('/dev/tty','/dev/video','/dev/snd')):
        raise AssertionError('Physical devices forbidden')

sys.addaudithook(audit)
import numpy as np
import rclpy
from camera_playback.app import App
from camera_playback.hybrid_strike import HybridStatus, load_tuning
from camera_playback.mit_strike import Sample
from camera_playback.simulation import SimulatedMotors
from camera_playback import hybrid_workflow as hw
from camera_playback.tempo import SYNC_PERIOD
from types import SimpleNamespace

app=None
rclpy.init(args=[])
try:
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)
        bus=SimulatedMotors(np.array([0.]*6+[1.4]))
        with patch('goal_motion.app.Motors',return_value=bus), \
             patch('camera_playback.app.HiHatController'), \
             patch('camera_playback.app.HiHatCalibrationRuntime'):
            app=App(True,str(path/'camera.sock'),str(path/'audio.sock'),left_recording_path=None)
        app.root.withdraw()
        app.bpm_text.set('110')
        assert 'hi-hat timing calibration (80 bpm)' in app.header.cget('text')
        assert 'hi-hat at 110 BPM' in app.header.cget('text')
        bus.active=True
        anchor=app.center_goal.copy()
        bus._joints=bus._targets=anchor.copy()
        app.arm=Mock(return_value=anchor)
        app.hybrid_anchor=anchor
        app.hybrid_tuning=load_tuning()
        app.hybrid_session=session=Mock()
        session.request.return_value=3
        app.phase='STRIKE WAITING FOR SOUND'
        now=time.monotonic()
        session.status=HybridStatus(phase='hold',ready=True,at=now,
            sample=Sample(anchor[6],0,.745,2,now))
        target=anchor.copy();target[6]-=math.radians(10.5)
        left=Mock(template=SimpleNamespace(curve=SimpleNamespace(down=.146)),finished=True)
        bus.left_joint6_session=left
        assert not hasattr(app, 'snare_sync')
        with patch.object(hw.snare,'start') as snare:
            app._begin_continuous_striking(10.5,target)
            assert session.request.call_args.args[0]=='calibrate'
            assert app.phase==hw.SYNC_CALIBRATION and app.sync_calibrating
            assert '80 BPM' in app.status.get()
            assert '80 BPM SYNC CALIBRATION' in app.center_relax_button.cget('text')
            assert app.bpm_entry.cget('state')=='disabled'
            snare.assert_not_called()
            engine=app.hihat_sync.engine;epoch=engine.epoch
            for index in range(2):
                target=epoch+index*SYNC_PERIOD
                for edge in (target,target+SYNC_PERIOD/2):
                    with patch('camera_playback.hihat_sync_runtime.time.monotonic',return_value=edge):
                        engine.tick(edge,dict(ride=True,hihat=True))
                now=target+.46
                for instrument in ('ride','hihat'):
                    engine.feed(instrument,[dict(kind='hit',instrument=instrument,
                        event_at=target+(.005 if instrument=='hihat' else 0.)),
                        dict(kind='progress',instrument=instrument,finalized_at=now)],now)
                engine.evaluate(now,dict(ride=True,hihat=True))
            assert engine.complete
            session.status=replace(session.status,swing=True,calibrating=True,
                at=now,sample=Sample(anchor[6],0,.745,2,now))
            with patch('camera_playback.hihat_sync_runtime.time.monotonic',return_value=now):hw.tick(app,now)
            assert app.phase==hw.SYNC_RETURN
            snare.assert_not_called()
            assert session.request.call_args.args==('finish',)
            now+=.4
            session.status=replace(session.status,request_id=3,swing=False,
                at=now,sample=Sample(anchor[6],0,.745,2,now))
            with patch('camera_playback.hihat_sync_runtime.time.monotonic',return_value=now):hw.tick(app,now)
            assert not app.sync_calibrating
            assert session.request.call_args.args==('swing',math.radians(10.5),app.swing_events)
            assert app.hihat_sync.engine.locked and app.hihat_sync.engine.advance==0.
            assert app.hihat_sync.engine.period==60/110
            left.start_calibration.assert_not_called()
            left.set_calibration_advance.assert_not_called()
            assert '110 BPM' in app.center_relax_button.cget('text')
            snare.assert_called_once_with(app)
            # Real visible Stop binding must cancel both future edge schedules.
            app.center_relax_button.invoke()
            assert session.request.call_args.args==('finish',)
            assert app.phase==hw.FINISHING
            assert bus.active and not app.hihat_sync.active
            assert app.continuous_stop_requested
        print('PASS: real Tk shows one 80 BPM hi-hat calibration, two acoustic matches, '
              'direct full-return handoff to 110 BPM swing, no snare calibration, and controlled Stop; '
              'all devices mocked, no physical hardware accessed')
finally:
    if app is not None:
        app._stop_hihat_sequence()
        hw.close(app)
        app._cancel_recording_preflight()
        app._cancel_planning()
        app.planner_executor.shutdown(wait=True,cancel_futures=True)
        app.receiver.close();app.audio_receiver.close()
        app.hihat_sound_monitor.close()
        if app.snare_audio_receiver is not None: app.snare_audio_receiver.close()
        app.root.destroy();app.node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
