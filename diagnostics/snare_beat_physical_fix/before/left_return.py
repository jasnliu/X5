"""Left-only reverse-recording return; the right centering controller is separate."""
from dataclasses import dataclass
import math
import time

import numpy as np

from goal_motion.control import GoalControl
from .left_hold import LEFT_GRIPPER_TARGET
from .smooth_recording import SmoothRecording
from .smooth_recording_worker import RecordingSession
from .trajectory import PlaybackFollower


@dataclass
class ReverseMotion:
    """Evaluate the validated motion backward, without refitting or resampling."""
    original: object
    duration: float

    @property
    def first(self): return self.original.at(self.duration)
    @property
    def last(self): return self.original.at(0.)
    @property
    def metadata(self):
        return dict(self.original.metadata, playback_direction='reverse',
                    reverse_from_seconds=self.duration)
    def at(self, elapsed):
        return self.original.at(max(0., self.duration-max(0., float(elapsed))))


@dataclass
class ReverseRecording:
    original: object
    duration_s: float

    def __post_init__(self):
        if (self.original.side != 'left' or not math.isfinite(self.duration_s)
                or not 0. <= self.duration_s <= self.original.duration_s):
            raise ValueError('Invalid left reverse recording interval')

    @property
    def side(self): return 'left'
    @property
    def geometry(self): return self.original.geometry
    @property
    def smooth_motion(self): return ReverseMotion(self.original.smooth_motion, self.duration_s)
    @property
    def first_joints(self): return self.original.joints_at(self.duration_s)[0]
    @property
    def last_joints(self): return self.original.first_joints
    def joints_at(self, elapsed):
        return (self.original.joints_at(max(0., self.duration_s-max(0., float(elapsed))))[0],
                elapsed >= self.duration_s)


class LeftReturnSession(RecordingSession):
    """Only left J1-J7 are writable; right center/disable remains GUI-owned."""
    center_return = True
    side = 'left'

    def __init__(self, recording, bus, **kwargs):
        if recording.side != 'left':
            raise ValueError('Left return cannot own right recording targets')
        super().__init__(recording, bus, **kwargs)


class LeftReturn:
    def __init__(self, dual, reverse_from):
        self.dual = dual
        self.failure = None
        self.phase = 'APPROACH REVERSE START'
        self.reverse = ReverseRecording(dual.recording, reverse_from)
        q = dual.positions('left')
        dual.g.check_line(q, self.reverse.first_joints, measured_start=True)
        dual.g.check_line(self.reverse.last_joints, dual.g.center)
        self.first_target = self.reverse.first_joints
        self.control = GoalControl(self.first_target, dual.g.lower, dual.g.upper, time.monotonic())

    def tick(self, now):
        d, a = self.dual, self.dual.app
        if self.failure or 'left' in a.bus.center_disabled:
            return
        q = d.positions('left')
        states = [a.bus.states['left', i] for i in range(1, 9)]
        if any(mode != 2 for _, mode, _ in states):
            raise RuntimeError('Left motor stopped during reverse return')
        d.g.check(q, measured=True)
        if abs(states[7][0]-LEFT_GRIPPER_TARGET) > math.radians(1.):
            raise RuntimeError('Left gripper hold lost during reverse return')
        if self.phase == 'APPROACH REVERSE START':
            self.control.update(q, now)
            if not a.bus.left_hold_ready() or np.max(np.abs(q-self.first_target)) > math.radians(.20):
                return
            a.bus.begin_left_playback()
            if isinstance(d.recording, SmoothRecording):
                d.session = LeftReturnSession(self.reverse, a.bus)
                a.bus.playback_session = d.session
            else:
                self.follower = PlaybackFollower()
            self.started = now
            self.phase = 'REVERSE RECORDING'
            print('LEFT RETURN: playing selected recording backward to its start', flush=True)
        elif self.phase == 'REVERSE RECORDING':
            if d.session is not None:
                update = d.session.poll()
                done = update['done']
                if done:
                    result = update['result']
                    d.stop()  # Writer has already exited: join before next target.
                    if not result['success']:
                        raise RuntimeError('Left reverse recording failed: '+str(result.get('error')))
            else:
                desired, done = self.reverse.joints_at(now-self.started)
                a.bus.set_left_positions(self.follower.update(q, desired, now))
            if done:
                a.bus.set_left_hold(self.reverse.last_joints)
                self.phase = 'VERIFY RECORDING START'
                self.control = GoalControl(self.reverse.last_joints, d.g.lower, d.g.upper, now)
        elif self.phase == 'VERIFY RECORDING START':
            self.control.update(q, now)
            if not a.bus.left_hold_ready() or np.max(np.abs(q-self.reverse.last_joints)) > math.radians(.20):
                return
            # This leg was checked before either arm started its return.
            a.bus.set_left_hold(d.g.center)
            d.return_controls['left'] = GoalControl(d.g.center, d.g.lower, d.g.upper, now)
            self.phase = 'CENTER'
            print('LEFT RETURN: recording start verified; centering, then relaxing', flush=True)

    def fault(self, message):
        """Do not freeze/rebase the right wrist when the LEFT return fails."""
        self.failure = str(message)
        self.phase = 'FAILED'
        self.dual.return_controls.pop('left', None)
        try:
            self.dual.stop()
            if self.dual.app.bus.fresh():
                self.dual.app.bus.set_left_hold(self.dual.positions('left'))
        except Exception as exc:
            self.failure += '; '+str(exc)
        text = 'LEFT RETURN halted; not relaxed; Emergency Relax remains available: '+self.failure
        self.dual.app.status.set(text)
        print(text, flush=True)
