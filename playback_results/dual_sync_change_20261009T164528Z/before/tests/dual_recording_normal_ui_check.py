"""Normal beat UI construction/preflight only; physical devices forbidden."""
import socket
import sys
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch


def audit(event, args):
    if event == 'socket.__new__' and args[1] == socket.PF_CAN:
        raise AssertionError('CAN forbidden')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty', '/dev/video', '/dev/snd')):
        raise AssertionError('Physical devices forbidden')


sys.addaudithook(audit)
import rclpy
from camera_playback.app import App, DEFAULT_RECORDING
from camera_playback.dual_recording import DEFAULT_LEFT_RECORDING
from camera_playback.smooth_recording import SmoothRecording
from tests.test_left_center_hold import fake_bus
from safe_zone.encoder import FRAME

bus = fake_bus()
sockets = {side: sock.sock for side, sock in bus.sockets.items()}
hihat = Mock(connect=Mock(return_value=False), detail='NO-MOTION TEST: serial owner mocked')
rclpy.init(args=[])
app = None
try:
    with tempfile.TemporaryDirectory() as directory:
        with patch('goal_motion.app.Motors', return_value=bus), patch('camera_playback.app.HiHatController', return_value=hihat):
            app = App(True, str(Path(directory)/'camera.sock'), str(Path(directory)/'audio.sock'),
                      recording_path=DEFAULT_RECORDING)
        app.root.withdraw()
        # Never dispatch the control/calibration timers in this test.
        app.recording_preflight.thread.join(180)
        app._poll_recording_preflight('record3.json')
        app.dual.job.thread.join(180)
        app.dual.poll_load()
        assert app.single_run_mode and app.hybrid_enabled and app.dual.ready
        assert isinstance(app.playback_trajectory, SmoothRecording)
        assert isinstance(app.dual.recording, SmoothRecording)
        assert app.playback_trajectory.source == DEFAULT_RECORDING
        assert app.dual.recording.source == DEFAULT_LEFT_RECORDING
        assert app.playback_trajectory.side == 'right' and app.dual.recording.side == 'left'
        assert app.continue_button.cget('text') == 'RUN: CENTER BOTH → LEFT → RIGHT → HYBRID SWING'
        assert app.continue_button.cget('state') == 'disabled'
        assert app.dual.button.winfo_manager() == app.recording_button.winfo_manager() == 'pack'
        assert not app.bus.active
        for sock in sockets.values():
            sock.send.assert_not_called()
        assert len(hihat.method_calls) == 1 and hihat.method_calls[0][0] == 'connect'
        print('PASS: normal UI loads both smooth defaults, shows separate selectors, retains readiness gate; zero motor packets; no physical devices')
        assert app.emergency_button.cget('text') == 'EMERGENCY RELAX (NO CENTER)'
        # Invoke the actual Tk command using ONLY the injected fake sockets.
        # No encoder/center evidence is supplied; off-center disable must work.
        app.bus.active = app.bus.left_drive.active = True
        app.emergency_button.invoke()
        assert app.emergency_latched and not app.bus.active
        for sock in sockets.values():
            assert sock.send.call_count == 24
            assert all((FRAME.unpack(call.args[0])[0] >> 24) & 31 == 4
                       for call in sock.send.call_args_list)
        print('PASS: actual Emergency Relax button sends only 24 disable frames per fake arm, no centering targets; restart latched')
finally:
    if app is not None:
        app._cancel_recording_preflight()
        app._cancel_planning()
        app.planner_executor.shutdown(wait=True, cancel_futures=True)
        app.receiver.close()
        app.audio_receiver.close()
        app.hihat_sound_monitor.close()
        app.root.destroy()
        app.node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
