"""Real Tk cache integration in all launcher modes; device owners are mocked.

Run with the ROS environment sourced. Never press Run/Start, never run control
timers, and forbid physical CAN/device opens. Only calculation workers and Tk
idle drawing are allowed while checking cold/warm recording installation.
"""
from pathlib import Path
import socket
import sys
import tempfile
import time
from unittest.mock import Mock, patch


def audit(event, args):
    if event == 'socket.__new__' and args[1] == socket.PF_CAN:
        raise RuntimeError('Offline cache UI check forbids physical CAN')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(
            ('/dev/tty', '/dev/video', '/dev/snd', '/dev/bus/usb')):
        raise RuntimeError('Offline cache UI check forbids physical devices')


sys.addaudithook(audit)

import rclpy
from camera_playback.app import App, ROOT
from camera_playback.smooth_recording import SmoothRecording
from camera_playback.trajectory import PlaybackTrajectory


with tempfile.TemporaryDirectory() as directory, \
     patch.dict('os.environ', {'XDG_CACHE_HOME': directory}):
    rclpy.init(args=[])
    try:
        for mode in ('hardware', 'hardwaretest', 'preview', 'test'):
            app = None
            bus = Mock(active=False, fresh=Mock(return_value=False))
            hihat = Mock(detail='offline mocked controller', state='disconnected',
                         ready=Mock(return_value=False), connect=Mock(return_value=False))
            expected_hit = mode in ('hardwaretest', 'test')
            physical_mode = mode in ('hardware', 'hardwaretest')
            module = 'camera_playback.recording_cache.' + (
                'smooth_recording.load_smooth_recording' if physical_mode
                else 'trajectory.load_playback_trajectory')
            # A warm UI load must not invoke the original expensive loader.
            from contextlib import nullcontext
            loader_guard = patch(module, side_effect=AssertionError('Expected cache hit')) \
                if expected_hit else nullcontext()
            try:
                with patch('goal_motion.app.Motors', return_value=bus), \
                     patch('camera_playback.app.HiHatController', return_value=hihat), \
                     loader_guard:
                    app = App(physical_mode, str(Path(directory) / (mode + '-camera.sock')),
                              str(Path(directory) / (mode + '-audio.sock')),
                              recording_path=None, test_mode=mode == 'test',
                              hardware_test_mode=mode == 'hardwaretest')
                    app.root.withdraw()
                    # Never dispatch startup, calibration or control timers.
                    for callback in app.root.tk.call('after', 'info'):
                        app.root.after_cancel(callback)
                    start = time.perf_counter()
                    assert app.load_recording(ROOT / 'recordings/record1.json', show_dialog=False)
                    if app.recording_preflight is not None:
                        assert app.playback_trajectory is None
                        app.recording_preflight.thread.join(30)
                        assert not app.recording_preflight.thread.is_alive()
                        app._poll_recording_preflight('record1.json')
                    elapsed = time.perf_counter() - start
                    expected_type = SmoothRecording if physical_mode else PlaybackTrajectory
                    assert type(app.playback_trajectory) is expected_type
                    assert app.recording_preflight is None
                    assert 'Recording: record1.json' in app.recording_status.get()
                    assert not app._recording_loading_text
                    assert app.phase == 'READY'
                    assert app.smooth_playback_session is None
                    if physical_mode:
                        assert app.start_button.cget('state') == 'disabled'
                    bus.center.assert_not_called()
                    bus.set_positions.assert_not_called()
                    bus.set_gripper.assert_not_called()
                    print(f'PASS {mode}: {"cached" if expected_hit else "full preflight"} '
                          f'{elapsed:.3f}s; correct trajectory installed; no motion', flush=True)
            finally:
                if app is not None:
                    app.bus = None
                    app._cancel_recording_preflight()
                    app._cancel_planning()
                    app.planner_executor.shutdown(wait=True, cancel_futures=True)
                    app.receiver.close()
                    app.audio_receiver.close()
                    if app.hihat is not None:
                        app.hihat.close()
                    monitor = getattr(app, 'hihat_sound_monitor', None)
                    if monitor is not None and monitor.receiver is not None:
                        monitor.receiver.close()
                    app.root.destroy()
                    app.node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
