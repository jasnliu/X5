"""Left-before-right recording step and simultaneous, per-arm center/relax.

The right-arm alignment/strike workflow is unchanged. The left arm never enters
it: its only targets are approach, recording, endpoint hold and center.
"""
from functools import partial
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog

import numpy as np

from goal_motion.control import GoalControl
from camera_search.app import App as CameraSearchApp
from centering.motors import RIGHT_GRIPPER_CLOSED
from safe_zone.geometry import Zone, LEFT_TCP
from smooth_playback.trajectory import Geometry
from .left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from .mit_center_return import MitCenterReturn
from .recording_cache import load_cached_smooth_recording, load_cached_playback_trajectory
from .recording_preflight import RecordingPreflight
from .recording_only import refresh_feedback
from .smooth_recording import SmoothRecording
from .smooth_recording_worker import RecordingSession
from .trajectory import PlaybackFollower

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LEFT_RECORDING = ROOT / 'left_recordings/record1.json'
APPROACH = 'LEFT MOVING TO RECORDING START'
PLAYING = 'LEFT RECORDING PLAYBACK'
END_HOLD = 'LEFT RECORDING END HOLD'
CENTERING = 'BOTH ARMS CENTERING'
FAULT = 'DUAL RECORDING SAFE HOLD'
LEFT_PHASES = {APPROACH, PLAYING, END_HOLD}


def left_geometry(model):
    g = Geometry.__new__(Geometry)
    g.model, g.side = model, 'left'
    g.zone = Zone.load(ROOT / 'left_zones/zone1.json', model.digest, LEFT_TCP)
    joints = {j.get('name'): j for j in model.joints}
    limits = [joints[f'openarmx_left_joint{i}'].find('limit') for i in range(1, 8)]
    g.lower = np.array([float(j.get('lower')) for j in limits])
    g.upper = np.array([float(j.get('upper')) for j in limits])
    g.center = LEFT_CENTER.copy()
    return g


class DualRecording:
    def __init__(self, app):
        self.app = app
        self.g = left_geometry(app.model)
        self.recording = self.job = self.session = None
        self.left_done = False
        self.returning = False
        self.cleanup_error = None
        self.label = tk.StringVar(value='Left recording: none selected')
        self.widget = tk.Label(app.root, textvariable=self.label, wraplength=650)
        self.widget.pack(fill='x', padx=20, pady=4, before=app.recording_label)
        self.button = tk.Button(app.root, text='SELECT LEFT RECORDING', command=self.choose)
        self.button.pack(fill='x', padx=20, pady=4, before=app.recording_label)

    @property
    def ready(self):
        return self.recording is not None and self.job is None and self.cleanup_error is None

    def selectable(self):
        a = self.app
        calibration = getattr(a, 'hihat_calibration', None)
        return (self.job is None and not getattr(a, 'close_requested', False)
                and not (a.bus and a.bus.active) and not (calibration and calibration.busy))

    def refresh(self):
        self.button.config(state='normal' if self.selectable() else 'disabled')

    def choose(self):
        if self.selectable():
            path = filedialog.askopenfilename(initialdir=ROOT / 'left_recordings',
                filetypes=[('OpenArmX left motion JSON', '*.json')])
            if path:
                self.load(path)

    def load(self, path):
        if not self.selectable():
            return False
        self.recording = None
        loader = load_cached_smooth_recording if self.app.hardware and not self.app.test_mode else load_cached_playback_trajectory
        self.job = RecordingPreflight(path, self.g.model, self.g.zone,
            self.g.lower.copy(), self.g.upper.copy(), self.g.center.copy(), .4,
            playback_speed=.8, loader=partial(loader, side='left'))
        self.label.set('Left recording: loading ' + Path(path).name)
        self.app._refresh_buttons()
        self.app.root.after(20, self.poll_load)
        return True

    def poll_load(self):
        if self.job is None:
            return
        for kind, value in self.job.poll():
            if kind == 'progress':
                self.label.set('Left recording: ' + value)
            else:
                self.job = None
                if kind == 'error' or (self.app.bus and self.app.bus.active):
                    self.recording = None
                    self.label.set('Left recording rejected: ' + str(value if kind == 'error' else 'Arm became active'))
                    self.widget.config(fg='#b00020')
                else:
                    self.recording = value
                    self.label.set(f'Left recording: {value.source.name} • {value.duration_s:.2f} s • endpoint hold')
                    self.widget.config(fg='#087f23')
                self.app._refresh_buttons()
                return
        self.app.root.after(20, self.poll_load)

    def cancel_load(self):
        if self.job is not None:
            self.job.cancel()
            self.job = None

    def positions(self, side='left'):
        return np.array([self.app.bus.states[side, i][0] for i in range(1, 8)])

    def reset(self):
        if not self.ready:
            raise RuntimeError('Select and preflight a left recording before RUN')
        self.left_done = False
        self.returning = False
        self.return_prepared = False

    def begin_left(self):
        a = self.app
        if not self.ready or not a.bus.fresh() or not a.bus.left_center_ready():
            raise RuntimeError('Fresh centered left arm and validated left recording required')
        q = self.positions()
        self.g.check_line(q, self.recording.first_joints, measured_start=True)
        self._refresh_after_geometry(q, 'left')
        a.control = None  # Right holds center; only the left arm moves next.
        a.bus.set_left_hold(self.recording.first_joints)
        a.phase = APPROACH
        a.status.set('Left arm: center → recording start; right arm holds center')

    def _refresh_after_geometry(self, q, side):
        if not self.app.test_mode:
            refresh_feedback(self.app.bus)
        if not self.app.bus.fresh() or np.max(np.abs(self.positions(side)-q)) > math.radians(.20):
            raise RuntimeError(f'{side.capitalize()} arm moved during path validation')

    def stop(self):
        if self.session is not None:
            result = self.session.stop()  # Join before any GUI target or return.
            self.session = None
            self.app.bus.playback_session = None
            if result.get('cleanup_error') or not result.get('gains_restored', False):
                self.cleanup_error = str(result.get('cleanup_error') or result.get('error'))
        if self.cleanup_error:
            raise RuntimeError('Left playback settings not restored: ' + self.cleanup_error)

    def tick(self, now):
        a = self.app
        if self.returning:
            self.tick_return(now)
            # Keep the original right-arm extra_control/gripper supervision
            # running until that arm has independently relaxed.
            return 'right' in a.bus.center_disabled
        if a.phase == FAULT:
            return True
        if a.phase not in LEFT_PHASES:
            return False
        if not a.bus.fresh():
            raise RuntimeError('Left playback lost fresh feedback')
        if (any(a.bus.states['right', i][1] == 0 for i in range(1, 9))
                or np.max(np.abs(self.positions('right')-a.center_goal)) > math.radians(3.1)
                or abs(a.bus.states['right', 8][0]-RIGHT_GRIPPER_CLOSED) > math.radians(1.)):
            raise RuntimeError('Right arm/gripper did not hold center during left playback')
        self.g.check(self.positions(), measured=True)
        if abs(a.bus.states['left', 8][0]-LEFT_GRIPPER_TARGET) > math.radians(1.):
            raise RuntimeError('Left gripper did not maintain centered closed position')
        if a.phase == APPROACH and a.bus.left_hold_ready():
            a.bus.begin_left_playback()
            if isinstance(self.recording, SmoothRecording):
                self.session = RecordingSession(self.recording, a.bus)
                a.bus.playback_session = self.session
                refresh_feedback(a.bus, .10)
            else:
                self.follower = PlaybackFollower()
                self.started = now
            a.phase = PLAYING
        elif a.phase == PLAYING:
            if self.session is not None:
                update = self.session.poll()
                a.status.set(f'Left recording: {update["elapsed"]:.2f}/{self.recording.duration_s:.2f} s • {update["phase"]}')
                finished = update['done']
                if finished:
                    result = update['result']
                    self.stop()
                    if not result['success']:
                        raise RuntimeError('Left recording failed: ' + str(result.get('error')))
            else:
                desired, finished = self.recording.joints_at(now-self.started)
                a.bus.set_left_positions(self.follower.update(self.positions(), desired, now))
            if finished:
                a.bus.set_left_hold(self.recording.last_joints)
                a.phase = END_HOLD
                a.status.set('Left recording complete — holding its endpoint before right playback')
        elif a.phase == END_HOLD and a.bus.left_hold_ready():
            self.left_done = True
            a.begin_stage('MOVING TO RECORDING START', a.playback_trajectory.first_joints)
            a.status.set('Left holds its endpoint; right arm moves to its recording start')
        return True

    def prepare_return(self):
        """Finish joins/geometry BEFORE creating the original MIT return clock."""
        if self.returning:
            return
        self.return_prepared = False
        a = self.app
        self.stop()
        if not a._stop_smooth_playback():
            raise RuntimeError(a.smooth_playback_cleanup_error)
        if not a.test_mode:
            # Joining a worker includes a settled hold/gain restoration and can
            # outlast GUI feedback freshness. Request new replies, never relax
            # the freshness limit or require a second Center click.
            refresh_feedback(a.bus, .15)
        if not a.bus.fresh():
            raise RuntimeError('Fresh feedback required to center both arms')
        a.control = a.setup = None
        a.playback_active = a.alignment_active = False
        self.return_controls = {}
        disabled = getattr(a.bus, 'center_disabled', {})
        for side, g, goal in (('left', self.g, self.g.center),
                              ('right', a.playback_trajectory.geometry if isinstance(a.playback_trajectory, SmoothRecording) else None, a.center_goal)):
            if side in disabled:
                continue
            q = self.positions(side)
            if g is not None:
                g.check_line(q, goal, measured_start=True)
            else:
                if not a.ik.path_between_inside(q, goal):
                    raise RuntimeError('Right return leaves zone1')
            self._refresh_after_geometry(q, side)
            lower, upper = (self.g.lower, self.g.upper) if side == 'left' else (a.lower, a.upper)
            if side == 'left':
                self.return_controls[side] = GoalControl(goal, lower, upper, time.monotonic())
        self.return_prepared = True

    def begin_return(self, phase='CENTER RELAX RECENTERING'):
        if self.returning:
            return
        if not getattr(self, 'return_prepared', False):
            self.prepare_return()
        self.return_prepared = False
        a = self.app
        if not a.bus.fresh():
            raise RuntimeError('Fresh feedback required to dispatch center targets')
        # Execute the SAME inherited begin_stage and goal_motion.App.tick as
        # start_beatTest. Do not run a second right controller in extra_control.
        # The left target is still dispatched in this same GUI tick.
        a.bus.begin_dual_center(
            right_start=lambda: CameraSearchApp.begin_stage(a, phase, a.center_goal))
        self.returning = True
        a.phase = phase
        a.status.set('Centering BOTH arms together; each relaxes after its own verified arrival')

    def right_center_reached(self):
        """Called by the original right controller's complete_stage callback."""
        a = self.app
        if 'right' not in a.bus.center_disabled:
            try:
                a.bus.disable_centered_side('right')
            except RuntimeError as exc:
                if str(exc) != 'Center not settled yet':
                    raise
                return
        a.control = None  # No target resends after this side has relaxed.

    def tick_return(self, now):
        a = self.app
        if not a.bus.fresh():
            raise RuntimeError('Fresh feedback lost during two-arm centering')
        for side, control in self.return_controls.items():
            if side in a.bus.center_disabled:
                continue
            states = [a.bus.states[side, i] for i in range(1, 9)]
            if any(state[1] == 0 for state in states):
                raise RuntimeError(f'{side.capitalize()} motor stopped before verified center')
            if side == 'left':
                self.g.check(self.positions(side), measured=True)
                if abs(states[7][0]-LEFT_GRIPPER_TARGET) > math.radians(1.):
                    raise RuntimeError('Left gripper hold lost during centering')
            _, _, reached = control.update(self.positions(side), now)
            if reached:
                try:
                    a.bus.disable_centered_side(side)
                except RuntimeError as exc:
                    if str(exc) != 'Center not settled yet':
                        raise
        confirmed = []
        for side, disabled_at in a.bus.center_disabled.items():
            samples = [a.bus.states[side, i] for i in range(1, 9)]
            if all(mode == 0 and stamp > disabled_at for _, mode, stamp in samples):
                confirmed.append(side)
            elif now-disabled_at > 2.:
                raise RuntimeError(f'{side.capitalize()} disable not confirmed; use physical stop')
        if len(confirmed) == 2:
            self.returning = False
            a.bus.active = False
            a.relax('Both arms centered and individually relaxed')

    def fault(self, message):
        try:
            self.stop()
        except Exception as exc:
            message = str(message) + '; ' + str(exc)
        self.returning = False
        self.return_prepared = False
        controller = getattr(self.app.bus, 'mit_center_return', None)
        if isinstance(controller, MitCenterReturn):
            # Stop an already-handed-off right J7 center controller advancing
            # after a fault; retain its bounded MIT hold, without a mode change.
            q = self.app.bus.states.get(('right', 7))
            hold = controller.actual if q is None else q[0]
            controller.goal = controller.position = controller.actual = hold
            controller.velocity = 0.
        self.app.control = self.app.setup = None
        self.app.phase = FAULT
        self.app.status.set('Playback halted; drives retain last targets; no right playback advance; NOT relaxed: ' + str(message))
