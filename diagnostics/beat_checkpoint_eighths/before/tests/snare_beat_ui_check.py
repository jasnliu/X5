"""Actual Tk/ROS main beat --test, eight measures and stop mid-snare. NO devices."""
import json
from pathlib import Path
import socket
import sys
import tempfile
import time
from unittest.mock import patch
import numpy as np
import rclpy


def audit(event, args):
    if event == 'socket.__new__' and args[1] == socket.PF_CAN:
        raise AssertionError('Physical CAN forbidden')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty','/dev/video','/dev/snd')):
        raise AssertionError('Physical devices forbidden')


sys.addaudithook(audit)
from camera_playback.app import App, DEFAULT_RECORDING
from camera_playback import dual_recording as dual
from camera_playback.left_hold import LEFT_CENTER
from tests.test_snare_beat import Choices

rclpy.init(args=[])
app = None
hits, phases = [], []
try:
    with tempfile.TemporaryDirectory() as directory:
        with patch('goal_motion.app.Motors', side_effect=AssertionError('No motor constructor')), \
             patch('camera_playback.app.HiHatController', side_effect=AssertionError('No serial constructor')):
            app = App(False, str(Path(directory)/'camera.sock'), str(Path(directory)/'audio.sock'),
                      recording_path=DEFAULT_RECORDING, test_mode=True)
        app.root.withdraw()
        app.dual.job.thread.join(60); app.dual.poll_load()
        assert app.dual.ready, app.dual.label.get()
        assert app.dual.snare_template.target == np.radians(11.)
        for callback in app.root.tk.call('after', 'info'): app.root.after_cancel(callback)
        with patch('time.monotonic', return_value=time.monotonic()) as clock, \
             patch('camera_playback.snare_beat.random.Random', return_value=Choices((4, 1, 2, 3))):
            app.start(); app._start_preflighted_recording()
            stop = False
            last_release = None
            for index in range(12000):
                clock.return_value += .01
                app.tick()
                if not phases or phases[-1] != app.phase:
                    phases.append(app.phase)
                assert app.phase not in {'FAULT', dual.FAULT, 'ZONE RECENTERING'}, app.status.get()
                if app.phase == 'WAITING FOR LOAD': app.continue_motion()
                sim = app.bus.snare_simulation
                if sim is not None:
                    s = sim.status
                    if s.released_at is not None and s.released_at != last_release:
                        hits.append(dict(measure=s.measure, beat=s.beat, scheduled=s.scheduled_at,
                                         release=s.released_at))
                        last_release = s.released_at
                    if s.measure == 8 and s.moving and not stop:
                        app._request_continuous_stop()
                        assert app.continuous_strike_active, 'Must finish full snare return first'
                        stop = True
                if stop and app.phase == 'RELAXED': break
            else: raise AssertionError('UI did not complete: '+app.status.get())
            assert [(h['measure'], h['beat']) for h in hits] == list(enumerate((4, 1, 2, 3)*2, 1)), hits
            for h in hits:
                assert abs(h['scheduled']-h['release']-app.dual.snare_template.curve.down) < 1e-9
            assert phases.index(dual.PLAYING) < phases.index('RECORDING PLAYBACK')
            assert not app.bus.active
            assert set(app.bus.center_disabled) == {'left', 'right'}
            np.testing.assert_allclose(app.bus._left_joints, LEFT_CENTER, atol=1e-10)
            np.testing.assert_allclose(app.bus._joints, app.center_goal, atol=1e-10)
            assert app.bus.snare_simulation is None
        Path('diagnostics/snare_beat_integration/simulation_ui_trace.json').write_text(json.dumps(
            dict(hits=hits, phases=phases, result='PASS: reference simulation only; zero physical I/O'), indent=2)+'\n')
        print('PASS: eight measures, all four choices including 4 -> 1; exact 11 degree reference; stop mid-snare completes return; both centered then relaxed; NO devices')
finally:
    if app is not None:
        app._cancel_recording_preflight(); app._cancel_planning()
        app.planner_executor.shutdown(wait=True, cancel_futures=True)
        app.receiver.close(); app.audio_receiver.close()
        if getattr(app, 'hihat_sound_monitor', None): app.hihat_sound_monitor.close()
        app.root.destroy(); app.node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
