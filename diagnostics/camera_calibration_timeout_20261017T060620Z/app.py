"""User-driven capture GUI. Nothing connects until the Start button is pressed."""
import signal
import time
import tkinter as tk
from tkinter import messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

from safe_zone.geometry import Model
from .core import Board, URDF, HAND, camera_parameters, novel_pose
from .io import Camera, Session
from .observer import Observer, stationary_capture


class App:
    def __init__(self, args):
        self.args = args; self.board = Board(); self.model = Model(URDF)
        self.root = tk.Tk(); self.root.title('Gemini 2 / OpenArmX camera calibration — NO MOTOR CONTROL')
        self.session = Session(args.output, preview=args.command == 'preview')
        self.camera = None; self.observer = None; self.parameters = None
        self.pending = None; self.sequence = 0; self.started = None; self.failure = None
        self.status = tk.StringVar(value='Read the setup checklist, then press Start. No devices are open.')
        self.quality = tk.StringVar(value='Camera not started — press Start below')
        self.counts = tk.StringVar(value='Training: 0 / 15 minimum | Validation: 0 / 5 minimum')
        self.arm_ok = tk.BooleanVar(); self.board_ok = tk.BooleanVar(); self.camera_ok = tk.BooleanVar()
        tk.Label(self.root, text='RIGHT ARM QUERY ONLY · NO ENABLING, HOLDING, CENTERING OR GRIPPER COMMANDS',
                 fg='darkred', font=('Sans', 12, 'bold')).pack(padx=12, pady=6)
        tk.Label(self.root, text='The arm must already be disabled. Support it by hand; do not let it fall. '
                 'Keep the stick clear of drums/table.\nThe left arm is never queried or controlled. '
                 'Stop start_beat, other robot controllers and camera viewers first.', justify='left').pack(padx=12)
        self.canvas = tk.Label(self.root, text='Camera preview will appear here', width=100, height=18, bg='#222', fg='white')
        self.canvas.pack(padx=12, pady=8)
        tk.Label(self.root, textvariable=self.quality).pack()
        check = tk.Frame(self.root); check.pack(anchor='w', padx=12)
        tk.Checkbutton(check, variable=self.board_ok,
                       text='Board is flat/rigid on the final wrist/hand housing; printed squares measure 40 mm.').pack(anchor='w')
        tk.Checkbutton(check, variable=self.camera_ok,
                       text='Camera and robot base are secure; board cannot slip relative to the hand housing.').pack(anchor='w')
        if not self.session.data['preview_only']:
            tk.Checkbutton(check, variable=self.arm_ok,
                           text='Right arm is already disabled and supported; no other controller is running.').pack(anchor='w')
        row = tk.Frame(self.root); row.pack(padx=12, pady=6, fill='x')
        self.start_button = tk.Button(row, text='Start camera only' if args.command == 'preview' else 'Start camera + read-only right encoders',
                                      command=lambda: self.guard(self.start))
        self.start_button.pack(side='left', padx=3)
        self.train_button = tk.Button(row, text='Capture training [Space]', command=lambda: self.guard(lambda: self.capture('training')), state='disabled')
        self.train_button.pack(side='left', padx=3)
        self.fit_button = tk.Button(row, text='Solve / freeze training [S]', command=lambda: self.guard(self.fit), state='disabled')
        self.fit_button.pack(side='left', padx=3)
        row2 = tk.Frame(self.root); row2.pack(padx=12, pady=4)
        self.val_button = tk.Button(row2, text='Capture validation [V]', command=lambda: self.guard(lambda: self.capture('validation')), state='disabled')
        self.val_button.pack(side='left', padx=3)
        self.export_button = tk.Button(row2, text='Validate / export [E]', command=lambda: self.guard(self.export), state='disabled')
        self.export_button.pack(side='left', padx=3)
        self.undo_button = tk.Button(row2, text='Undo last training capture', command=lambda: self.guard(self.undo), state='disabled')
        self.undo_button.pack(side='left', padx=3)
        tk.Button(row2, text='Quit (no motor action)', command=self.close).pack(side='left', padx=3)
        tk.Label(self.root, textvariable=self.counts, font=('Sans', 12, 'bold')).pack(pady=4)
        tk.Label(self.root, textvariable=self.status, wraplength=980, justify='left', fg='#143e60').pack(padx=12, pady=6)
        tk.Label(self.root, text='Move by hand between poses. Vary position AND wrist tilt about two axes. '
                 'After pressing Capture, hold completely still for 2 seconds.\n'
                 'After Solve, collect at least 5 NEW poses for independent validation. '
                 'Calibration is not permission to run a collision-avoidance trajectory.', justify='left').pack(padx=12, pady=4)
        tk.Label(self.root, text=f'Session: {self.session.path}', wraplength=1000, fg='#555').pack(padx=12, pady=6)
        for key, function in [('s', self.fit), ('v', lambda: self.capture('validation')),
                              ('e', self.export), ('<space>', lambda: self.capture('training'))]:
            self.root.bind(key, lambda event, f=function: self.key_action(event, f))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        signal.signal(signal.SIGINT, lambda *_: self.close())
        signal.signal(signal.SIGTERM, lambda *_: self.close())
        self.root.after(60, self.tick)

    def key_action(self, event, function):
        # Space on a focused checkbox/button should not also capture a pose.
        if event.widget.winfo_class() not in ('Checkbutton', 'Button'):
            self.guard(function)

    def guard(self, function):
        try:
            function()
        except Exception as e:
            self.status.set(str(e))
            messagebox.showwarning('Calibration — no motor state was changed', str(e), parent=self.root)

    def start(self):
        if self.camera is not None:
            raise ValueError('Already started; close and start a new session to reconnect')
        if not self.session.data['preview_only']:
            if not all(v.get() for v in (self.board_ok, self.camera_ok, self.arm_ok)):
                raise ValueError('Confirm all three setup checks before connecting to the right encoders')
            self.session.data['confirmations'] = dict(square_measured_mm=40, board_rigid_to_hand=True,
                                                       camera_rigid_to_base=True, right_arm_disabled_supported=True)
            self.session.save()
        self.started = time.monotonic()
        self.camera = Camera(self.session.path, self.args.serial, self.args.fps)
        self.quality.set('Connecting to Gemini 2…')
        self.start_button.configure(state='disabled')
        self.status.set('Connecting to selected Gemini 2; no motion commands will be sent.')

    def capture(self, phase):
        if self.failure:
            raise ValueError('Session stopped: ' + self.failure)
        if not self.observer or self.parameters is None or self.observer.error:
            raise ValueError('Start capture mode and wait for healthy camera/disabled-arm feedback first')
        if self.pending:
            raise ValueError('Already capturing; keep still')
        frozen = self.session.data['fit'] is not None
        if (phase == 'training' and frozen) or (phase == 'validation' and not frozen):
            raise ValueError('Training is frozen; capture validation poses' if frozen else 'Solve training before validation')
        if self.session.data['validation'] is not None:
            raise ValueError('This session is finalized. Start a new session to repeat calibration.')
        now = time.monotonic()
        self.pending = dict(phase=phase, start=now+.4, end=now+2.0, frames=[])
        self.status.set('CAPTURING: support the arm and keep it completely still for 2 seconds…')

    def finish_capture(self):
        pending = self.pending; self.pending = None
        frame, joints, evidence = stationary_capture(pending['frames'], self.observer.samples(), pending['start'], pending['end'])
        A = self.model.transforms(joints)[HAND]
        if not novel_pose(A, self.session.data['samples']):
            raise ValueError('Pose too similar to an existing capture. Move >2cm or rotate >7 degrees, safely.')
        sample = dict(phase=pending['phase'], joints_rad=joints, T_base_hand=A.tolist(),
                      frame_header=frame['header'], stationary_evidence=evidence, **frame['detection'])
        saved = self.session.append(sample, frame['image'])
        self.status.set(f"Saved {pending['phase']} pose {saved['id']}. Reposition manually, then capture again.")
        self.update_counts()

    def update_counts(self):
        samples = self.session.data['samples']
        nt = sum(s['phase'] == 'training' for s in samples)
        nv = sum(s['phase'] == 'validation' for s in samples)
        self.counts.set(f'Training: {nt} / 15 minimum | Validation: {nv} / 5 minimum'
                        + (' | FIT FROZEN' if self.session.data['fit'] else ''))

    def fit(self):
        if self.failure:
            raise ValueError('Capture stopped; inspect the fault and use saved observations offline if needed')
        if self.pending:
            raise ValueError('Wait for the capture to finish')
        self.status.set('Solving camera and board mounting transforms…'); self.root.update_idletasks()
        fit = self.session.fit(self.board)
        for b in (self.train_button, self.fit_button, self.undo_button):
            b.configure(state='disabled')
        self.val_button.configure(state='normal'); self.export_button.configure(state='normal')
        m = fit['training_metrics']
        self.status.set(f"Training fit frozen: {m['translation_mm']['rms']:.1f}mm / "
                        f"{m['rotation_deg']['rms']:.2f}deg RMS consistency. Now capture at least 5 NEW validation poses.")
        self.update_counts()

    def export(self):
        if self.failure:
            raise ValueError('Capture stopped; inspect the fault and use saved observations offline if needed')
        if self.pending:
            raise ValueError('Wait for the capture to finish')
        result = self.session.validate(self.board)
        self.val_button.configure(state='disabled')
        m = result['validation']['metrics']
        state = 'PASSED consistency checks' if result['validated'] else 'FAILED consistency checks — do not use for planning'
        self.status.set(f"{state}. Held-out RMS: {m['translation_mm']['rms']:.1f}mm / "
                        f"{m['rotation_deg']['rms']:.2f}deg / {m['reprojection_px']['rms']:.1f}px. "
                        f"Saved {self.session.path / 'calibration.json'}. Depth mapping and physical accuracy are not yet verified.")

    def undo(self):
        if self.pending or self.session.data['fit'] is not None:
            raise ValueError('Undo is available only before solving, outside a capture')
        if self.session.data['samples']:
            removed = self.session.data['samples'].pop()
            removed['exclusion_reason'] = 'User explicitly undid last training capture before solving'
            self.session.data['excluded'].append(removed); self.session.save(); self.update_counts()
            self.status.set('Last training capture excluded; raw image retained for audit.')

    def fatal(self, reason):
        if self.failure:
            return
        self.failure = reason; self.pending = None
        self.quality.set('Camera preview stopped — see error below')
        if self.observer:
            self.observer.close()
        if self.camera:
            self.camera.close()
        for b in (self.train_button, self.val_button, self.fit_button, self.export_button, self.undo_button):
            b.configure(state='disabled')
        self.status.set('CAPTURE STOPPED: ' + reason + '. No motor state was changed. Support the arm; close and restart after fixing the cause.')

    def tick(self):
        try:
            self._tick()
        except Exception as e:
            self.fatal(str(e))
        if self.root.winfo_exists():
            self.root.after(60, self.tick)

    def _tick(self):
        if self.failure or not self.camera:
            return
        now = time.monotonic()
        if self.camera.error:
            self.fatal(self.camera.error); return
        if self.camera.metadata is not None and self.parameters is None:
            self.parameters = camera_parameters(self.camera.metadata)
            self.session.data['camera'] = self.camera.metadata; self.session.save()
            if not self.session.data['preview_only']:
                self.observer = Observer(self.session.path)
                for b in (self.train_button, self.fit_button, self.undo_button):
                    b.configure(state='normal')
            self.status.set('Camera ready. Support/reposition the disabled arm manually; capture varied stationary poses.'
                            if self.observer else 'PREVIEW ONLY: no CAN interface has been opened. Check board visibility and lighting.')
        if self.observer and self.observer.error:
            self.fatal(self.observer.error); return
        latest = self.camera.latest
        if latest is None:
            if now - self.started > 20:
                self.fatal('Camera startup timed out (see camera_stderr.log)')
            return
        header, raw = latest
        if now - header['received_monotonic'] > 1:
            self.fatal(self.camera.unusable_frame_message()); return
        if header['sequence'] != self.sequence:
            self.sequence = header['sequence']; display = raw.copy()
            try:
                detection = self.board.detect(raw, *self.parameters)
                uv = np.asarray(detection['corners_px'], dtype=np.float32).reshape(-1, 1, 2)
                ids = np.asarray(detection['corner_ids'], dtype=np.int32).reshape(-1, 1)
                cv2.aruco.drawDetectedCornersCharuco(display, uv, ids)
                self.quality.set(f"Board: {detection['corner_count']}/15 corners | "
                                 f"reprojection {detection['reprojection_rms_px']:.2f}px | RGB 1280×720 | frame {header['sequence']}")
                if self.pending:
                    self.pending['frames'].append(dict(header=header, image=raw, detection=detection))
            except ValueError as e:
                self.quality.set(str(e))
            if self.camera.jpeg_report['rejected']:
                self.quality.set(self.quality.get() + f" | Damaged JPEGs skipped: {self.camera.jpeg_report['rejected']}")
            display = cv2.cvtColor(cv2.resize(display, (960, 540)), cv2.COLOR_BGR2RGB)
            self.photo = ImageTk.PhotoImage(Image.fromarray(display))
            self.canvas.configure(image=self.photo, width=960, height=540, text='')
        if self.pending and now >= self.pending['end']+.22:
            try:
                self.finish_capture()
            except Exception as e:
                self.pending = None; self.status.set('Capture rejected: ' + str(e))

    def close(self):
        if self.observer:
            self.observer.close()
        if self.camera:
            self.camera.close()
        self.root.destroy()

    def run(self):
        try:
            self.root.mainloop()
        finally:
            if self.observer:
                self.observer.close()
            if self.camera:
                self.camera.close()
