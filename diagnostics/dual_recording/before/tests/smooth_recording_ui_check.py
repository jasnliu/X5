"""Real Tk physical-mode preflight with forbidden CAN and mocked device owners.

Run once without arguments and once with --hardwaretest, in separate processes.
No Run/Continue action is issued and no real hardware constructor is permitted.
"""
from pathlib import Path
import socket
import sys
import tempfile
from unittest.mock import Mock, patch

import rclpy

from camera_playback.app import App, ROOT
from camera_playback.smooth_recording import SmoothRecording


def audit(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.PF_CAN:
        raise RuntimeError('Offline UI check forbids physical CAN')


sys.addaudithook(audit)

with tempfile.TemporaryDirectory() as directory:
    hardware_test = '--hardwaretest' in sys.argv
    recording_only = '--recording-only' in sys.argv
    bus = Mock(active=False, fresh=Mock(return_value=False))
    hihat = Mock(detail='offline mocked controller', state='disconnected',
                 ready=Mock(return_value=False))
    app = None
    rclpy.init(args=[])
    try:
        with patch('goal_motion.app.Motors', return_value=bus) as motors, \
             patch('camera_playback.app.HiHatController', return_value=hihat):
            app = App(True, str(Path(directory)/'camera.sock'),
                      str(Path(directory)/'audio.sock'),
                      recording_path=ROOT/'recordings/record3.json',
                      hardware_test_mode=hardware_test, recording_only=recording_only)
        app.root.withdraw()
        assert motors.call_count == 1
        assert app.hardware and not app.test_mode
        assert isinstance(app.playback_trajectory, SmoothRecording)
        assert app.playback_trajectory.duration_s == 4.8
        assert 'paced_precise 200 Hz' in app.recording_status.get()
        assert app.phase == 'READY'
        assert app.smooth_playback_session is None
        bus.center.assert_not_called()
        bus.set_positions.assert_not_called()
        bus.set_gripper.assert_not_called()
        if hardware_test or recording_only:
            assert app.hihat is None
        else:
            assert app.hihat is hihat
        if hardware_test:
            assert app.hardware_test_frame.winfo_manager() == ('' if recording_only else 'pack')
        if recording_only:
            assert app.emergency_button.cget('text') == 'STOP: CENTER THEN RELAX'
            assert 'CENTER → RELAX' in app.continue_button.cget('text')
        print('PASS: real Tk', '--hardwaretest' if hardware_test else '--hardware',
              'preflights record3 with paced_precise; no physical devices or motion')
    finally:
        if app is not None:
            app.bus = None
            app._cancel_planning()
            app.planner_executor.shutdown(wait=False, cancel_futures=True)
            app.receiver.close()
            app.audio_receiver.close()
            if app.hihat is not None:
                app.hihat.close()
            app.root.destroy()
            app.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
