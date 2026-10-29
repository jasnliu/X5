"""Native generated-beat encoder panel check with fake motors; NO hardware I/O."""
import json
import math
from pathlib import Path
import socket
import sys
import time
import tkinter as tk
from unittest.mock import patch


def no_hardware(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden')
    if event == 'open' and args and isinstance(args[0], str) and args[0].startswith(
            ('/dev/ttyUSB', '/dev/ttyACM', '/dev/serial/', '/dev/video', '/dev/snd/')):
        raise AssertionError('Physical device forbidden')


sys.addaudithook(no_hardware)
import rclpy
from PIL import ImageGrab
from camera_playback.generated_app import GeneratedApp
from safe_zone.gripper_feedback import motor8_feedback


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', type=Path, default=Path(
        'planner_runs/generated-beat-20261028T114902-81cf47ce/plan-b5cbbf61/mission.json'))
    parser.add_argument('--report-dir', type=Path, default=Path('diagnostics/beat_encoder_display_20261028'))
    args = parser.parse_args(); args.report_dir.mkdir(parents=True, exist_ok=True)
    report = dict(passed=False, hardware_access=False, physical_motion=False)
    rclpy.init(args=[]); app = None

    def pump():
        end = time.monotonic() + .2
        while time.monotonic() < end:
            app.root.update(); time.sleep(.005)

    try:
        with patch('centering.motors.Motors.__init__', side_effect=AssertionError('No CAN owner')), \
             patch('camera_playback.hihat.HiHatController.connect', side_effect=AssertionError('No ESP32')):
            app = GeneratedApp(False, '/tmp/beat-encoder-test-camera.sock', '/tmp/beat-encoder-test-audio.sock',
                               simulate_mission=args.mission, auto_preview=False)
            app.bus.motor8_feedback = {s: motor8_feedback(c) for s, c in (('right', 33411), ('left', 35774))}
            pump()
            assert app.stage == 'READY' and not app.bus.active and app.motion.stage == 'READY'
            before = app.angles.get()
            app.bus.q['left'][4] += math.radians(1.5)  # Fake feedback only; no Start or command.
            pump()
            text = app.angles.get()
            assert text != before
            assert text.split('\nLeft encoder degrees:')[0] == before.split('\nLeft encoder degrees:')[0]
            for side in ('left', 'right'):
                row = text.split(side.capitalize() + ' encoder degrees:\n')[1].splitlines()[0]
                assert row == '  '.join(f'{i}: {math.degrees(q):6.1f}' for i, q in enumerate(app.bus.q[side], 1))
            assert '33411 / 65535' in text and '35774 / 65535' in text
            label = next(w for w in app.root.winfo_children() if isinstance(w, tk.Label)
                         and str(w.cget('textvariable')) == str(app.angles))
            assert label.winfo_ismapped() and label.winfo_height() >= label.winfo_reqheight()
            assert label.winfo_width() >= label.winfo_reqwidth()
            assert label.winfo_y() + label.winfo_height() <= app.root.winfo_height()
            assert str(app.emergency_button['state']) == str(app.emergency_relax_button['state']) == 'normal'
            assert not app.bus.active and not app.bus.calls
            app.root.lift(); pump()
            x, y = app.root.winfo_rootx(), app.root.winfo_rooty()
            ImageGrab.grab(bbox=(x, y, x + app.root.winfo_width(), y + app.root.winfo_height())).save(
                args.report_dir / 'panel.png')
            report.update(passed=True, encoder_text=text, fake_feedback_updates=True, no_simulated_commands=True,
                          panel_size=[app.root.winfo_width(), app.root.winfo_height()],
                          screen_size=[app.root.winfo_screenwidth(), app.root.winfo_screenheight()],
                          encoder_label_fully_visible=True, emergency_buttons_available=True)
    finally:
        if app:
            app.request_safe_close(); app.audio_receiver.close(); app.receiver.close(); app.hihat_sound_monitor.close()
            app.planner_executor.shutdown(wait=False, cancel_futures=True)
            app.root.destroy(); app.node.destroy_node()
        rclpy.shutdown()
        (args.report_dir / 'ui_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
