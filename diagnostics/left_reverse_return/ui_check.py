"""Actual Tk/ROS UI in pure simulation; never opens CAN, cameras or serial."""
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
        raise RuntimeError('UI test prohibits CAN')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty','/dev/video','/dev/snd')):
        raise RuntimeError('UI test prohibits physical devices')


sys.addaudithook(audit)
from camera_playback.app import App, DEFAULT_RECORDING
from camera_playback import dual_recording as dual
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET

trace = []
with tempfile.TemporaryDirectory() as directory:
    rclpy.init(args=[])
    app = None
    try:
        with patch('goal_motion.app.Motors', side_effect=AssertionError('No real motor constructor')), \
             patch('camera_playback.app.HiHatController', side_effect=AssertionError('No serial constructor')):
            app = App(False, str(Path(directory)/'camera.sock'), str(Path(directory)/'audio.sock'),
                      recording_path=DEFAULT_RECORDING, test_mode=True)
        app.root.withdraw()
        app.dual.job.thread.join(30)
        app.dual.poll_load()
        assert app.dual.ready, app.dual.label.get()
        assert app.dual.recording.source == dual.DEFAULT_LEFT_RECORDING
        assert app.playback_trajectory.source == DEFAULT_RECORDING
        assert app.dual.button.cget('text') == 'SELECT LEFT RECORDING'
        assert 'RIGHT' in app.recording_button.cget('text')
        assert app.dual.button.winfo_manager() == 'pack'
        # A wrong-arm replacement must clear the old pass and block Run.
        assert app.dual.load(DEFAULT_RECORDING)
        app.dual.job.thread.join(10); app.dual.poll_load()
        assert not app.dual.ready
        app.start()
        assert not app.bus.active
        assert 'left recording' in app.status.get()
        assert app.dual.load(dual.DEFAULT_LEFT_RECORDING)
        app.dual.job.thread.join(30); app.dual.poll_load()
        assert app.dual.ready
        for callback in app.root.tk.call('after', 'info'):
            app.root.after_cancel(callback)
        last = None
        with patch('time.monotonic', return_value=time.monotonic()) as clock:
            app.start()
            app._start_preflighted_recording()
            requested_return = False
            phases = []
            reverse_phases = []
            for index in range(6000):
                clock.return_value += .02
                app.tick()
                left_return = app.dual.left_return
                if left_return is not None and left_return.phase not in reverse_phases:
                    reverse_phases.append(left_return.phase)
                    trace.append({'t':clock.return_value, 'left_return':left_return.phase,
                                  'left_joints':app.bus._left_joints.tolist()})
                if app.phase != last:
                    phases.append(app.phase)
                    trace.append({'t':clock.return_value, 'phase':app.phase, 'status':app.status.get()})
                    print(app.phase, app.status.get(), flush=True)
                    last = app.phase
                assert app.phase not in {'FAULT', dual.FAULT, 'ZONE RECENTERING'}, app.status.get()
                if app.phase == 'WAITING FOR LOAD':
                    app.continue_motion()
                if app.phase in dual.LEFT_PHASES:
                    assert not app.dual.load(DEFAULT_RECORDING), 'Cannot replace selection while powered'
                    np.testing.assert_allclose(app.bus._joints, app.center_goal, atol=1e-10)
                    assert abs(app.bus._left_gripper-LEFT_GRIPPER_TARGET) < 1e-10
                if app.dual.left_done and not requested_return:
                    np.testing.assert_allclose(app.bus._left_joints, app.dual.recording.last_joints, atol=1e-10)
                if app.phase == 'SETTLING RECORDING END' and not requested_return:
                    # Exercise the actual Center button command, before test strikes.
                    app.center_relax_button.invoke()
                    if not app.dual.returning:  # The periodic refresh may not have enabled this widget yet.
                        app.center_relax()
                    requested_return = True
                    np.testing.assert_array_equal(app.bus._left_targets, app.dual.recording.last_joints)
                    np.testing.assert_array_equal(app.bus._targets, app.center_goal)
                if requested_return and app.phase == 'RELAXED':
                    break
            else:
                raise AssertionError('Simulated sequence did not finish: '+app.status.get())
            assert phases.index(dual.PLAYING) < phases.index('RECORDING PLAYBACK')
            assert not any('STRIKE' in p or 'HILL' in p for p in phases)
            assert set(app.bus.center_disabled) == {'left', 'right'}
            assert not app.bus.active
            assert reverse_phases == ['APPROACH REVERSE START', 'REVERSE RECORDING',
                                      'VERIFY RECORDING START', 'CENTER'], reverse_phases
            np.testing.assert_allclose(app.bus._left_joints, LEFT_CENTER, atol=1e-10)
            np.testing.assert_allclose(app.bus._joints, app.center_goal, atol=1e-10)
        print('PASS: actual Tk selectors/defaults; left → right playback; left reverse → start → center → relax; right independently centers/relaxes; no physical devices',flush=True)
    finally:
        Path('/home/jason/Proyectos3/X5/diagnostics/left_reverse_return/ui_trace.json').write_text(json.dumps(trace,indent=2)+'\n')
        if app is not None:
            app._cancel_recording_preflight()
            app._cancel_planning()
            app.planner_executor.shutdown(wait=True, cancel_futures=True)
            app.receiver.close();app.audio_receiver.close()
            monitor=getattr(app,'hihat_sound_monitor',None)
            if monitor:monitor.close()
            app.root.destroy();app.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
