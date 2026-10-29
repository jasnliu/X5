"""Beat sequencing adapter for the proven plan_drums hardware Controller.

The parent owns arming and both powered arms. A delegate plays ONE checked leg,
then hands back to the parent's settling/beat/return sequence. It cannot enable,
change modes/settings, auto-return, or relax. Controller.step/send/halt retain
the planner's exact clock law, discontinuity guard and playback supervision.
"""
import time
import numpy as np
from scipy.interpolate import PPoly

from drum_planner.baseline.motion import Controller, HOLD_PHASES
from drum_planner.baseline.paths import blend, recovery_at
from drum_planner.baseline.curve import Curve
from drum_planner.baseline.paths import validate_timing, validate_drive_speed
from drum_planner.execution import TRACK_L1
from .trajectory import PlaybackFollower


class PositionPort:
    """Expose only the existing position delegate to the shared controller."""
    def __init__(self, bus, side):
        self.bus, self.control_side = bus, side

    def set_positions(self, target):
        self.bus.command(target)


class Connector:
    """Legacy checked strike connector, not a refit of the generated path."""
    def __init__(self, original, step, side):
        self.plan = dict(moving_side=side)
        self.duration = float(step['duration'])
        if not np.isfinite(self.duration) or self.duration <= 0:
            raise ValueError('Invalid checked connector duration')
        self.start = np.asarray(step['start'], float)
        self.delta = np.asarray(step['goal'], float)-self.start
        self.limits = original.limits
        self.parts = self.retimed = None
        self.scale = 1.
        t = self.duration
        self.curves = [PPoly(np.array([6*d/t**5, -15*d/t**4, 10*d/t**3, 0, 0, a])[:, None],
                             [0., t]) for a, d in zip(self.start, self.delta)]

    def at(self, elapsed):
        return self.start+self.delta*blend(elapsed/self.duration)


class GeneratedController(Controller):
    """Same controller; only ownership and dual-powered-arm policy differ."""
    def __init__(self, bus, curve, side, step, targets, sample, *, now=time.monotonic):
        self.raw_bus = bus
        self.read_feedback = sample
        self.other_target = np.asarray(targets['right' if side == 'left' else 'left']).copy()
        self.other_follower = PlaybackFollower()
        self.latching_hold = False
        self.segment = step
        self.prefix = step is not None and step['kind'] == 'prefix'
        if step is not None and step['kind'] not in ('prefix', 'strike_return', 'planned'):
            raise ValueError('Unknown generated return segment')
        selected = (Curve(step['plan']) if step is not None and step['kind']=='planned' else
                    Connector(curve, step, side) if step is not None and not self.prefix else curve)
        if step is not None and step['kind']=='planned':
            validate_timing(selected); validate_drive_speed(selected,1.)
            if selected.plan['moving_side']!=side or abs(selected.duration-step['duration'])>1e-9:
                raise ValueError('Calculated return arm/duration mismatch')
            for key,at in (('start',0.),('goal',selected.duration)):
                if key in step and not np.allclose(selected.at(at),step[key],atol=1e-10,rtol=0):
                    raise ValueError('Calculated return endpoint/curve mismatch')
        super().__init__(PositionPort(bus, side), selected.plan,
                         dict(moving_side=side, start=selected.at(0).tolist()),
                         curve=selected, now=now)
        self.segment_duration = curve.duration if step is None else float(step['duration'])
        if self.prefix and abs(self.segment_duration-step['spec']['bridge_s']-step['spec']['return_s']) > 1e-9:
            raise ValueError('Prepared return duration mismatch')

    def sample(self, mode='enabled'):
        # Unlike single-arm plan_drums, both arms intentionally remain powered.
        # Keep honest feedback, including both grippers, bounds and zones; never
        # pretend that the stationary opposite arm is disabled.
        self.raw_bus.poll()
        values = self.read_feedback(self.raw_bus, self.now())
        self.q = values[self.side].copy()
        if self.started and not self.latching_hold and self.phase not in HOLD_PHASES:
            self.other_follower.update(values[self.other], self.other_target, self.now())
        return self.q

    def begin_segment(self):
        q = self.sample()
        target = (recovery_at(self.curve, self.segment['spec'], 0.)[0]
                  if self.prefix else self.curve.at(0.))
        if np.sum(abs(q-target)) > TRACK_L1:
            raise RuntimeError('Arm moved before generated playback; hold and retry from the checked path')
        self.anchor = target.copy()
        self.started = True  # Already armed by the sole parent owner; NO enable.
        if self.prefix:
            self.return_spec = self.segment['spec']
            self.tau = float(self.return_spec['tau'])
        self.emit('RECOVERY' if self.prefix else 'OUTWARD')
        self.segment_started = self.phase_at
        self.previous = self.phase_at

    @property
    def complete(self):
        # Parent verifies both-arm settling and decides beat/next leg/release.
        # Do not run the single-arm controller's automatic goal wait/return.
        return self.phase in ('SETTLING_GOAL', 'SETTLING_REST')

    def halt(self, *args, **kwargs):
        # A tracking failure does not veto a measured HOLD with healthy feedback.
        self.latching_hold = True
        try:
            return super().halt(*args, **kwargs)
        finally:
            self.latching_hold = False
