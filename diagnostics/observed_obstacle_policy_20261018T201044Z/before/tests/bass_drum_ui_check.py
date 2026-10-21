"""Actual hardware-mode Tk checkbox with all physical devices forbidden/mocked.

No motion is run. Phase changes below exercise setup/locked/relaxed UI states.
"""
from pathlib import Path
import socket
import sys
import tempfile
import time
from unittest.mock import patch


def audit(event, args):
    if event == 'socket.__new__' and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty', '/dev/video', '/dev/snd')):
        raise AssertionError('Physical devices forbidden')

sys.addaudithook(audit)
import numpy as np
import rclpy
from PIL import ImageGrab
from camera_playback.app import App
from camera_playback.simulation import SimulatedMotors

ROOT = Path(__file__).resolve().parents[1]
app = None
rclpy.init(args=[])
try:
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        bus = SimulatedMotors(np.array([0.] * 6 + [1.4]))
        with patch('goal_motion.app.Motors', return_value=bus), \
             patch('camera_playback.app.HiHatController'), \
             patch('camera_playback.app.HiHatCalibrationRuntime'):
            app = App(True, str(path/'camera.sock'), str(path/'audio.sock'), left_recording_path=None)
        app.root.geometry('+40+40')
        app.phase = 'READY'; bus.active = False
        app._refresh_buttons()
        box = app.bass_drum_checkbox
        assert box.cget('text') == 'Use bass drum (motor 1) in swing beat'
        assert box.winfo_manager() == 'grid'
        assert box.cget('state') == 'normal'
        assert app.bass_drum_selected.get() and app.bass_drum_enabled
        box.invoke()
        assert not app.bass_drum_selected.get() and not app.bass_drum_enabled
        assert '(bass off)' in app.header.cget('text')
        app.bpm_text.set('110')
        assert app.strike_bpm == 110 and not app.bass_drum_enabled
        assert '(bass off) at 110 BPM' in app.header.cget('text')

        # Cancel app timers before processing window-manager paint events.
        # This is UI-only: no application motion/startup loop is entered.
        for identifier in app.root.tk.call('after', 'info'):
            app.root.after_cancel(identifier)
        app.root.lift()
        app.root.attributes('-topmost', True)
        app.root.update_idletasks()
        app.root.update()
        time.sleep(.4)  # Allow compositor window-opening animation to finish.
        app.root.update()
        frame = app.tempo_frame
        area = (frame.winfo_rootx(), frame.winfo_rooty(),
                frame.winfo_rootx()+min(frame.winfo_width(), 1000),
                frame.winfo_rooty()+frame.winfo_height())
        output = ROOT/'diagnostics/bass_checkbox_20261010'
        output.mkdir(parents=True, exist_ok=True)
        ImageGrab.grab(bbox=area).save(output/'checkbox_off.png')

        for phase in ('PLAYBACK PREFLIGHTED', 'CENTERING', 'HIHAT SYNC CALIBRATION',
                      'SWING STRIKE RETURNING', 'CENTER RELAX RECENTERING'):
            app.phase = phase; bus.active = True
            app._refresh_tempo_controls()
            assert box.cget('state') == 'disabled'
            box.invoke()
            assert not app.bass_drum_selected.get() and not app.bass_drum_enabled
        app.bass_drum_selected.set(True)  # Programmatic edits cannot change this run.
        assert not app.bass_drum_enabled
        app.bass_drum_selected.set(False)
        app.phase = 'RELAXED'; bus.active = False
        app._refresh_buttons()
        assert box.cget('state') == 'normal' and not app.bass_drum_enabled
        box.invoke()
        assert app.bass_drum_enabled
        assert '+ quarter-note bass at 110 BPM' in app.header.cget('text')
        print('PASS: real Tk checkbox defaults ON, toggles OFF, reflects BPM/status, locks throughout the run, and unlocks after relax; no hardware accessed')
finally:
    if app is not None:
        app._stop_hihat_sequence()
        app._cancel_recording_preflight(); app._cancel_planning()
        app.planner_executor.shutdown(wait=True, cancel_futures=True)
        app.receiver.close(); app.audio_receiver.close(); app.hihat_sound_monitor.close()
        if app.snare_audio_receiver is not None: app.snare_audio_receiver.close()
        app.root.destroy(); app.node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
