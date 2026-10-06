"""Gravity-driven variants with independent shared predictive reversal math."""
from camera_playback.mit_strike import Command
from ..config import TARGET
from ..curves import Catch, smooth
from .base import Method, Definition, TRACK, TRACK_BOUNDS


class Gravity(Method):
    def __init__(self, *args):
        super().__init__(*args)
        self.catch = None
        self.catch_at = None

    def damping(self, depth):
        return self.p['fall_kd']

    def falling(self, sample, now, depth):
        return Command(sample.position, 0, 0, self.damping(depth), 0)

    def update(self, sample, now):
        depth, speed = self.anchor-sample.position, max(0., -sample.velocity)
        if self.catch is None:
            lead = max(0., now-sample.at)+self.p['latency']+.002
            plan = Catch(depth+speed*lead, speed, self.p['brake'], self.p['return_time'])
            if plan.low < TARGET:
                self.phase = 'fall'
                return self.falling(sample, now, depth)
            self.catch, self.catch_at = plan, now
        t = now-self.catch_at
        self.done = t >= self.catch.duration
        self.phase = 'settling' if self.done else ('catch' if t < self.catch.catch_time else 'withdrawal')
        return self.hold() if self.done else self.tracking(self.catch.at(t))


class VariableDamping(Gravity):
    def damping(self, depth):
        u = (depth/TARGET-self.p['ramp_start'])/(1-self.p['ramp_start'])
        return self.p['fall_kd']+(self.p['end_kd']-self.p['fall_kd'])*smooth(u)


DEFAULTS = dict(TRACK, fall_kd=.02, latency=.004, brake=40., return_time=.20)
BOUNDS = dict(TRACK_BOUNDS, fall_kd=(0.,.15), latency=(.001,.012), brake=(10.,60.), return_time=(.13,.4))
DEFINITION = Definition('gravity', 'Lightly damped gravity', 'gravity',
    'No downward drive; position/velocity-based predictive catch.', DEFAULTS, BOUNDS, Gravity)
VARIABLE = Definition('variable_damping', 'Scheduled gravity damping', 'variable_damping',
    'Damping rises smoothly before the predictive turnaround.',
    dict(DEFAULTS, fall_kd=.005, end_kd=.08, ramp_start=.5),
    dict(BOUNDS, end_kd=(.01,.2), ramp_start=(.2,.85)), VariableDamping)
