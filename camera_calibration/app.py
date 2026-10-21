"""Hands-free capture GUI. Nothing connects until the Start button is pressed."""
import signal
import time
import tkinter as tk
from tkinter import messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

from safe_zone.geometry import Model
from .automatic import AutomaticCapture, AUTO_POLICY
from .core import Board, URDF, HAND, MIN_TRAIN, MIN_VALIDATION, camera_parameters, novel_pose
from .io import Camera, Session, FRESH_FRAME_MAX_AGE, FRAME_OUTAGE_TIMEOUT
from .observer import Observer


class App:
    def __init__(self, args):
        self.args = args; self.board = Board(); self.model = Model(URDF)
        self.root = tk.Tk(); self.root.title('Gemini 2 / OpenArmX camera calibration — NO MOTOR CONTROL')
        self.session = Session(args.output, preview=args.command == 'preview', capture_policy=AUTO_POLICY)
        self.camera = None; self.observer = None; self.parameters = None
        self.sequence = 0; self.started = None; self.failure = None
        self.camera_waiting = False; self.auto = AutomaticCapture()
        self.status = tk.StringVar(value='Read the setup checklist, then press Start. No devices are open.')
        self.auto_status = tk.StringVar(value='Automatic capture not started')
        self.quality = tk.StringVar(value='Camera not started — press Start below')
        self.counts = tk.StringVar()
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
        self.start_button = tk.Button(row, text='Start camera only' if args.command == 'preview' else 'Start camera + automatic capture',
                                      command=lambda: self.guard(self.start))
        self.start_button.pack(side='left', padx=3)
        self.auto_button = tk.Button(row, text='Stop automatic captures [Space]',
                                     command=lambda: self.guard(self.toggle_auto), state='disabled')
        self.auto_button.pack(side='left', padx=3)
        self.fit_button = tk.Button(row, text='Solve / freeze training [S]', command=lambda: self.guard(self.fit), state='disabled')
        self.fit_button.pack(side='left', padx=3)
        row2 = tk.Frame(self.root); row2.pack(padx=12, pady=4)
        self.export_button = tk.Button(row2, text='Validate / export [E]', command=lambda: self.guard(self.export), state='disabled')
        self.export_button.pack(side='left', padx=3)
        self.undo_button = tk.Button(row2, text='Undo last training capture (when stopped)', command=lambda: self.guard(self.undo), state='disabled')
        self.undo_button.pack(side='left', padx=3)
        tk.Button(row2, text='Quit (no motor action)', command=self.close).pack(side='left', padx=3)
        tk.Label(self.root, textvariable=self.counts, font=('Sans', 12, 'bold')).pack(pady=4)
        tk.Label(self.root, textvariable=self.auto_status, wraplength=980, fg='#175b29').pack(padx=12)
        tk.Label(self.root, textvariable=self.status, wraplength=980, justify='left', fg='#143e60').pack(padx=12, pady=6)
        tk.Label(self.root, text='Move by hand; pause briefly with the board visible. A beep confirms each saved DISTINCT pose.\n'
                 'First 5 poses: validation only. All later poses: training, with no fixed limit. '
                 'Stop automatic captures before Solve, then Validate.\n'
                 'Vary position AND wrist tilt about two axes. Calibration does not authorize collision-avoidance motion.',
                 justify='left').pack(padx=12, pady=4)
        tk.Label(self.root, text=f'Session: {self.session.path}', wraplength=1000, fg='#555').pack(padx=12, pady=6)
        for key, function in [('s', self.fit), ('e', self.export), ('<space>', self.toggle_auto)]:
            self.root.bind(key, lambda event, f=function: self.key_action(event, f))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        signal.signal(signal.SIGINT, lambda *_: self.close())
        signal.signal(signal.SIGTERM, lambda *_: self.close())
        self.update_counts(); self.refresh_controls()
        self.root.after(60, self.tick)

    def key_action(self, event, function):
        if event.widget.winfo_class() not in ('Checkbutton', 'Button'):
            self.guard(function)

    def guard(self, function):
        try:
            function()
        except Exception as e:
            self.status.set(str(e))
            messagebox.showwarning('Calibration — no motor state was changed', str(e), parent=self.root)

    def refresh_controls(self):
        data = self.session.data
        usable = bool(self.observer is not None and not self.failure and not data['preview_only'])
        frozen = data['fit'] is not None; finalized = data['validation'] is not None
        nt = sum(s['phase'] == 'training' for s in data['samples'])
        nv = sum(s['phase'] == 'validation' for s in data['samples'])
        collect = usable and not frozen and not finalized
        self.auto_button.configure(state='normal' if collect else 'disabled',
                                   text='Stop automatic captures [Space]' if self.auto.active else 'Resume automatic captures [Space]')
        self.fit_button.configure(state='normal' if collect and not self.auto.active and nt >= MIN_TRAIN and nv == MIN_VALIDATION else 'disabled')
        self.export_button.configure(state='normal' if usable and frozen and not finalized else 'disabled')
        undoable = bool(data['samples'] and data['samples'][-1]['phase'] == 'training')
        self.undo_button.configure(state='normal' if collect and not self.auto.active and undoable else 'disabled')

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
        self.camera = Camera(self.session.path, self.args.serial, self.args.fps, self.args.color_format)
        self.quality.set('Connecting to Gemini 2…')
        self.start_button.configure(state='disabled')
        self.status.set('Connecting to selected Gemini 2; no motion commands will be sent.')

    def toggle_auto(self):
        # Stopping never closes feedback, changes motors, solves, or discards saved poses.
        if self.auto.active:
            self.auto.stop()
            self.status.set('Automatic captures stopped. Saved poses retained; resume or solve when ready.')
        else:
            if self.failure:
                raise ValueError('Session stopped: ' + self.failure)
            if self.observer is None or self.observer.error or self.parameters is None:
                raise ValueError('Start capture mode and wait for healthy camera/disabled-arm feedback first')
            self.session.next_auto_phase()  # Refuse resume after fitting/finalization.
            self.auto.start()
            self.status.set('Automatic captures resumed. Support and reposition the arm manually.')
        self.auto_status.set(self.auto.reason)
        self.refresh_controls()

    def automatic_capture(self, now):
        if not self.auto.active or self.observer is None:
            return
        result = self.auto.candidate(self.observer.samples(), now)
        if result is None:
            self.auto_status.set('AUTO ON — ' + self.auto.reason)
            return
        frame, joints, evidence = result
        A = self.model.transforms(joints)[HAND]
        if not novel_pose(A, self.session.data['samples']):
            self.auto_status.set('AUTO ON — Pose already saved. Move >2 cm or rotate >7 degrees, safely.')
            return
        phase = self.session.next_auto_phase()
        sample = dict(phase=phase, joints_rad=joints, T_base_hand=A.tolist(),
                      frame_header=frame['header'], stationary_evidence=evidence, **frame['detection'])
        saved = self.session.append(sample, frame['image'])
        self.auto.clear()  # Never reuse the accepted window for another capture.
        self.status.set(f"Saved {phase} pose {saved['id']}. Reposition manually; no button press needed.")
        self.auto_status.set('AUTO ON — Waiting for a different steady pose')
        self.update_counts(); self.refresh_controls()
        self.root.bell()

    def update_counts(self):
        samples = self.session.data['samples']
        nt = sum(s['phase'] == 'training' for s in samples)
        nv = sum(s['phase'] == 'validation' for s in samples)
        self.counts.set(f'Total saved: {len(samples)} | Validation: {nv} / 5 reserved | '
                        f'Training: {nt} (15 minimum, no fixed limit)'
                        + (' | FIT FROZEN' if self.session.data['fit'] else ''))

    def fit(self):
        if self.failure:
            raise ValueError('Capture stopped; inspect the fault and use saved observations offline if needed')
        if self.auto.active:
            raise ValueError('Stop automatic captures before solving')
        self.status.set('Solving with TRAINING poses only; checking local model correction if needed…'); self.root.update_idletasks()
        fit = self.session.fit(self.board)
        m = fit['training_metrics']
        self.status.set(f"Training fit frozen: {m['translation_mm']['rms']:.1f}mm / "
                        f"{m['rotation_deg']['rms']:.2f}deg RMS. "
                        + ('Includes local kinematic correction; no motor settings changed. ' if fit.get('kinematic_correction') else '')
                        + 'Press Validate to test the first 5 reserved poses; they were NOT fitted.')
        self.update_counts(); self.refresh_controls()

    def export(self):
        if self.failure:
            raise ValueError('Capture stopped; inspect the fault and use saved observations offline if needed')
        if self.auto.active:
            raise ValueError('Stop automatic captures before validation')
        result = self.session.validate(self.board)
        m = result['validation']['metrics']
        state = 'PASSED consistency checks' if result['validated'] else 'FAILED consistency checks — do not use for planning'
        self.status.set(f"{state}. Held-out RMS: {m['translation_mm']['rms']:.1f}mm / "
                        f"{m['rotation_deg']['rms']:.2f}deg / {m['reprojection_px']['rms']:.1f}px. "
                        f"Saved {self.session.path / 'calibration.json'}. Depth mapping and physical accuracy are not yet verified.")
        self.refresh_controls()

    def undo(self):
        if self.auto.active or self.session.data['fit'] is not None:
            raise ValueError('Stop automatic captures before undo; undo is unavailable after solving')
        if not self.session.data['samples'] or self.session.data['samples'][-1]['phase'] != 'training':
            raise ValueError('Only training captures can be undone; the first 5 validation poses remain reserved')
        removed = self.session.data['samples'].pop()
        removed['exclusion_reason'] = 'User explicitly undid last training capture before solving'
        self.session.data['excluded'].append(removed)
        try:
            self.session.save()
        except Exception:
            self.session.data['excluded'].pop(); removed.pop('exclusion_reason')
            self.session.data['samples'].append(removed)
            raise
        self.auto.clear(); self.update_counts(); self.refresh_controls()
        self.status.set('Last training capture excluded; raw image retained for audit.')

    def fatal(self, reason):
        if self.failure:
            return
        self.failure = reason; self.auto.stop()
        self.auto_status.set('AUTO OFF — session stopped')
        self.quality.set('Camera preview stopped — see error below')
        if self.observer:
            self.observer.close()
        if self.camera:
            self.camera.close(reason=reason)
        self.refresh_controls()
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
                self.auto.start(); self.refresh_controls()
            self.status.set('Camera ready. Automatic capture is ON; support/reposition the disabled arm manually.'
                            if self.observer else 'PREVIEW ONLY: no CAN interface has been opened. Check board visibility and lighting.')
        if self.observer and self.observer.error:
            self.fatal(self.observer.error); return
        latest = self.camera.latest
        if latest is None:
            if now - self.started > 20:
                self.fatal('Camera startup timed out (see camera_stderr.log)')
            return
        header, raw = latest
        frame_age = now - header['received_monotonic']
        if frame_age > FRESH_FRAME_MAX_AGE:
            if frame_age >= FRAME_OUTAGE_TIMEOUT:
                self.fatal(self.camera.unusable_frame_message(now)); return
            self.auto.clear(); self.camera_waiting = True
            self.auto_status.set('AUTO PAUSED — waiting for fresh RGB frames' if self.auto.active else self.auto.reason)
            self.quality.set('RGB preview paused — waiting for a fresh image')
            return
        if self.camera_waiting:
            self.camera_waiting = False; self.auto.clear()
            self.status.set('Fresh RGB frames restored. New steady evidence is required before any capture.')
        if header['sequence'] != self.sequence:
            self.sequence = header['sequence']; display = raw.copy()
            try:
                detection = self.board.detect(raw, *self.parameters)
                uv = np.asarray(detection['corners_px'], dtype=np.float32).reshape(-1, 1, 2)
                ids = np.asarray(detection['corner_ids'], dtype=np.int32).reshape(-1, 1)
                cv2.aruco.drawDetectedCornersCharuco(display, uv, ids)
                self.quality.set(f"Board: {detection['corner_count']}/15 corners | "
                                 f"reprojection {detection['reprojection_rms_px']:.2f}px | RGB 1280×720 | frame {header['sequence']}")
                self.auto.add(dict(header=header, image=raw, detection=detection))
            except ValueError as e:
                self.quality.set(str(e)); self.auto.clear(); self.auto.reason = str(e)
            if self.camera.jpeg_report['rejected']:
                self.quality.set(self.quality.get() + f" | Damaged JPEGs skipped: {self.camera.jpeg_report['rejected']}")
            display = cv2.cvtColor(cv2.resize(display, (960, 540)), cv2.COLOR_BGR2RGB)
            self.photo = ImageTk.PhotoImage(Image.fromarray(display))
            self.canvas.configure(image=self.photo, width=960, height=540, text='')
        self.automatic_capture(now)

    def close(self):
        self.auto.stop()
        if self.observer:
            self.observer.close()
        if self.camera:
            self.camera.close()
        self.root.destroy()

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.auto.stop()
            if self.observer:
                self.observer.close()
            if self.camera:
                self.camera.close()
