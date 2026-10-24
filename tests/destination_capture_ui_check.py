"""Native Tk + ROS checks with synthetic encoders. No CAN/camera access.

Run with a display and sourced ROS:
PYTHONPATH=".:$PYTHONPATH" /usr/bin/python3 tests/destination_capture_ui_check.py.
"""
import json
import os
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from destination_capture.app import App, parse_args
from drum_planner.common import endpoint
from safe_zone.gripper_feedback import EncoderState, motor8_feedback
from motion_recording.recording import joint_names, gripper_name


def exercise(side, folder):
    mode = {'failure': False, 'offset': .1, 'joint7': None}
    opened, closed, messages = [], [], []

    class FakeObserver:
        def __init__(self, actual_side, bus):
            assert actual_side == side
            assert bus == ('can0' if side == 'right' else 'can1')
            opened.append((actual_side, bus))
        def sample(self):
            if mode['failure']:
                raise RuntimeError('synthetic feedback outage')
            # Change only J1 between valid captures. Offsetting every joint by
            # .2 made the old left "success" fixture exceed J2's 0.1292 limit.
            values = {name: .01 * i for i, name in enumerate(joint_names(side))}
            values[joint_names(side)[0]] = mode['offset']
            if mode['joint7'] is not None:
                values[joint_names(side)[6]] = mode['joint7']
            values[gripper_name(side)] = .005
            return EncoderState(values, motor8_feedback={side: motor8_feedback(32900)})
        def close(self): closed.append(True)

    args = parse_args(['--' + side, '--hardware', '--output-root', str(folder)])
    app = App(args, observer_factory=FakeObserver)
    listener = Node('destination_test_listener_' + side)
    subscription = listener.create_subscription(JointState, '/joint_states', messages.append, 10)

    def pump_until(predicate, seconds=4):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.root.update()
            rclpy.spin_once(listener, timeout_sec=0)
            if predicate(): return
            time.sleep(.01)
        raise AssertionError(f'{side}: timed out: {app.status.get()} / {app.notice.get()}')

    try:
        pump_until(lambda: 'READY' in app.status.get() and len(messages) > 0)
        app.capture_button.invoke()
        first = app.saved_path
        assert first.name == ('destinationR1.json' if side == 'right' else 'destinationL1.json')
        assert app.saved_count == 1 and 'Saved:' in app.notice.get()
        assert float(endpoint(first, side=side)[0]) == .1
        original = first.read_bytes()
        # The reported R4 angle must show an actionable error, never create a
        # file or advance numbering, and remain retryable in the same window.
        before = {p.name: p.read_bytes() for p in app.store.folder.iterdir()}
        bad_angle = 1.5192945754177156
        mode['joint7'] = bad_angle
        pump_until(lambda: app.last_state[joint_names(side)[6]] == bad_angle)
        app.capture_button.invoke()
        assert f'Not saved: {side.title()} J7 out of range' in app.notice.get()
        assert '87.05' in app.notice.get() and '80.21' in app.notice.get()
        assert app.notice_label['fg'] == 'firebrick'
        assert app.saved_count == 1 and app.saved_path == first
        assert app.store.next_number() == 2 and '2.json' in app.counter.get()
        assert {p.name: p.read_bytes() for p in app.store.folder.iterdir()} == before
        assert app.capture_button['state'] == 'normal'
        app.root.update()
        assert (app.notice_label.winfo_y() + app.notice_label.winfo_height()
                <= app.root.winfo_height()), 'Rejection text is cut off'
        if os.environ.get('X5_DESTINATION_UI_SCREENSHOTS'):
            from PIL import ImageGrab
            output = Path(os.environ['X5_DESTINATION_UI_SCREENSHOTS'])
            output.mkdir(parents=True, exist_ok=True)
            app.root.lift();app.root.update();time.sleep(.1)
            x, y = app.root.winfo_rootx(), app.root.winfo_rooty()
            ImageGrab.grab(bbox=(x, y, x + app.root.winfo_width(),
                                 y + app.root.winfo_height())).save(output / f'{side}-range-error.png')
        mode['joint7'] = None
        mode['offset'] = .2
        pump_until(lambda: app.last_state[joint_names(side)[0]] == .2 and
                   any(m.position[m.name.index(joint_names(side)[0])] == .2 for m in messages))
        assert app.saved_count == 1  # Returning in range must not autosave.
        app.capture_button.invoke()
        assert app.saved_count == 2
        assert app.saved_path.name.endswith('2.json')
        assert app.notice_label['fg'] == 'darkgreen'
        assert float(endpoint(app.saved_path, side=side)[0]) == .2
        assert first.read_bytes() == original
        assert app.capture_button['state'] == 'normal'

        # Feedback faults do not kill the UI or queue a delayed save.
        mode['failure'] = True
        pump_until(lambda: 'unavailable' in app.status.get())
        app.capture_button.invoke()
        assert 'Not saved' in app.notice.get() and app.saved_count == 2
        mode['failure'] = False
        pump_until(lambda: 'READY' in app.status.get())
        assert app.saved_count == 2
        app.capture_button.invoke()
        assert app.saved_count == 3

        # Disk error is retryable using the same button and encoder connection.
        with patch.object(app.store, 'save', side_effect=PermissionError('synthetic disk error')):
            app.capture_button.invoke()
        assert 'synthetic disk error' in app.notice.get() and app.saved_count == 3
        app.capture_button.invoke()
        assert app.saved_count == 4
        assert app.capture_button['state'] == 'normal'
        assert 'Next:' in app.counter.get()
        # Exercise actual WM-close callback; run() performs the normal cleanup.
        app.root.after(100, app.close)
        app.run()
        assert not app.feedback.thread.is_alive()
        assert len(opened) == len(closed)
        print(f'PASS {side}: range error/no save/retry, captures 1–4, feedback recovery, disk retry, close', flush=True)
    finally:
        if not app.closed:
            app.close()
            app.feedback.close()
            app.root.destroy()
            app.node.destroy_node()
        listener.destroy_node()


def offline(folder):
    with patch('destination_capture.feedback.SingleArmObserver', side_effect=AssertionError('No CAN allowed')):
        app = App(parse_args(['--right', '--output-root', str(folder)]))
        app.root.update()
        assert app.feedback is None
        app.capture_button.invoke()
        assert 'Offline preview cannot capture' in app.notice.get()
        assert not list(folder.rglob('*.json'))
        app.root.after(100, app.close)
        app.run()
        print('PASS offline: no observer, no synthetic destinations, normal close', flush=True)


if __name__ == '__main__':
    rclpy.init(args=[])
    try:
        # A second guard ensures the fake-reader tests never touch real SocketCAN.
        with tempfile.TemporaryDirectory(prefix='x5-destination-ui-') as directory, \
                patch('safe_zone.encoder.socket.socket', side_effect=AssertionError('Hardware forbidden')):
            folder = Path(directory)
            offline(folder / 'offline')
            for side in ('right', 'left'):
                exercise(side, folder / 'captures')
    finally:
        rclpy.shutdown()
