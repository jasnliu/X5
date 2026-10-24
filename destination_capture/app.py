"""Small Tk capture panel and ROS joint-state publisher; no motor control."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from motion_recording.recording import joint_names
from safe_zone.geometry import Model
from safe_zone.gripper_feedback import gripper_feedback_text
from .feedback import Feedback, fresh
from .store import DestinationStore

ROOT = Path(__file__).resolve().parents[1]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='One-click, query-only arm destination capture with RViz')
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--right', dest='arm', action='store_const', const='right')
    modes.add_argument('--left', dest='arm', action='store_const', const='left')
    parser.add_argument('--hardware', action='store_true', help='Read disabled motors; never enables or moves them')
    parser.add_argument('--right-can', default='can0')
    parser.add_argument('--left-can', default='can1')
    parser.add_argument('--output-root', type=Path, default=ROOT / 'destinations',
                        help='Parent folder for right/left destinations (default: X5/destinations)')
    args = parser.parse_args(argv)
    args.can = args.right_can if args.arm == 'right' else args.left_can
    return args


class App:
    def __init__(self, args, *, observer_factory=None):
        self.args = args
        self.side = args.arm
        self.model = Model(ROOT / 'model/openarmx.urdf')
        self.store = DestinationStore(args.output_root, self.side)
        self.feedback = Feedback(self.side, args.can, observer_factory) if args.hardware else None
        self.node = Node(f'{self.side}_destination_capture')
        self.publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self.last_state = None
        self.saved_path = None
        self.saved_count = 0
        self.closed = False
        self.root = tk.Tk()
        suffix = '' if args.hardware else ' — OFFLINE PREVIEW'
        self.root.title(f'OpenArmX {self.side.upper()} destination capture' + suffix)
        self.root.geometry('640x390')
        self.root.minsize(640, 390)
        self.status = tk.StringVar()
        self.notice = tk.StringVar(value='No destinations captured this session.')
        self.angles = tk.StringVar()
        self.counter = tk.StringVar()
        tk.Label(self.root, text=f'{self.side.upper()} ARM DESTINATION CAPTURE',
                 font=('Sans', 16, 'bold')).pack(padx=16, pady=(14, 8))
        tk.Label(self.root, text='Manually position the relaxed arm, then click Capture destination.\n'
                 'This program only reads encoders; support the arm yourself.',
                 justify='left').pack(fill='x', padx=16, pady=4)
        self.capture_button = tk.Button(self.root, text='Capture destination',
                                       font=('Sans', 14, 'bold'), command=self.capture)
        self.capture_button.pack(fill='x', padx=16, pady=10)
        tk.Label(self.root, textvariable=self.status, anchor='w', wraplength=600,
                 justify='left').pack(fill='x', padx=16)
        tk.Label(self.root, textvariable=self.counter, anchor='w').pack(fill='x', padx=16, pady=5)
        self.notice_label = tk.Label(self.root, textvariable=self.notice, wraplength=600,
                                    anchor='w', justify='left', fg='darkgreen')
        self.notice_label.pack(fill='x', padx=16, pady=5)
        tk.Label(self.root, textvariable=self.angles, anchor='w', justify='left',
                 font=('Monospace', 10)).pack(fill='x', padx=16, pady=5)
        tk.Label(self.root, text='RViz: drag to orbit · middle-drag to pan · scroll to zoom').pack(
            side='bottom', pady=10)
        self.refresh_counter()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        if self.feedback:
            self.feedback.thread.start()
        self.tick()

    def refresh_counter(self):
        self.counter.set(f'Saved this session: {self.saved_count}   '
                         f'Next: {self.store.prefix}{self.store.next_number()}.json')

    def capture(self):
        # Read directly from the worker, not a possibly delayed Tk display queue.
        snapshot, error = self.feedback.read() if self.feedback else (None, None)
        try:
            if not self.args.hardware:
                raise ValueError('Offline preview cannot capture. Restart with --hardware to read the arm.')
            if error or not fresh(snapshot):
                raise ValueError(error or 'No fresh encoder feedback yet. Wait for READY and click again.')
            age = time.monotonic() - snapshot.completed
            self.saved_path = self.store.save(
                snapshot.state, self.model, captured_at_utc=snapshot.captured_at_utc,
                feedback_age_s=age, batch_duration_s=snapshot.batch_duration_s, bus=self.args.can)
            self.saved_count += 1
            self.notice.set(f'Saved: {self.saved_path}')
            self.notice_label.config(fg='darkgreen')
            print(f'Saved destination: {self.saved_path}', flush=True)
            self.refresh_counter()
        except Exception as exc:
            self.notice.set(f'Not saved: {exc}')
            self.notice_label.config(fg='firebrick')
            print(f'Destination not saved: {exc}', flush=True)
        # No dialogs, disabled buttons, pending capture, or restart requirement.

    def tick(self):
        if self.closed:
            return
        snapshot, error = self.feedback.read() if self.feedback else (None, None)
        ready = error is None and fresh(snapshot)
        if not self.args.hardware:
            self.status.set('OFFLINE PREVIEW — no CAN access; capture does not save synthetic poses.')
        elif error:
            self.status.set(f'Feedback unavailable — retrying: {error}')
        elif ready:
            self.status.set(f'READY — reading only {self.side} arm on {self.args.can}; capture checks joint limits.')
        else:
            self.status.set('Waiting for fresh encoder feedback — RViz shows the last known pose.')
        if ready:
            self.last_state = snapshot.state
        state = {name: 0. for name in self.model.names}
        if self.last_state is not None:
            state.update(self.last_state)
        # Do not continually stamp a stale measured pose as fresh ROS feedback.
        if ready or not self.args.hardware:
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name, msg.position = list(state), list(state.values())
            self.publisher.publish(msg)
        values = [math.degrees(state[name]) for name in joint_names(self.side)]
        self.angles.set('  '.join(f'J{i+1}: {q:+.2f}°' for i, q in enumerate(values[:4])) + '\n' +
                        '  '.join(f'J{i+5}: {q:+.2f}°' for i, q in enumerate(values[4:])) + '\n' +
                        gripper_feedback_text(getattr(self.last_state, 'motor8_feedback', {}).get(self.side)))
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(50, self.tick)

    def close(self):
        if not self.closed:
            self.closed = True
            self.root.quit()

    def run(self):
        try:
            self.root.mainloop()
        finally:
            if self.feedback:
                self.feedback.close()
            self.root.destroy()
            self.node.destroy_node()


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=[])
    try:
        app = App(args)
        signal.signal(signal.SIGINT, lambda *_: app.close())
        signal.signal(signal.SIGTERM, lambda *_: app.close())
        app.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
