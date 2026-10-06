"""Small method interface. A strategy cannot command any joint except J7."""
from dataclasses import dataclass, field
import math
from camera_playback.mit_strike import Command
from ..config import TARGET


@dataclass(frozen=True)
class Definition:
    id: str
    label: str
    family: str
    description: str
    defaults: dict
    bounds: dict
    factory: object = field(repr=False)

    def parameters(self, overrides=None):
        p = dict(self.defaults)
        if overrides:
            unknown = set(overrides)-set(p)
            if unknown:
                raise ValueError(f'Unknown {self.id} parameters: {sorted(unknown)}')
            p.update(overrides)
        for k, v in p.items():
            lo, hi = self.bounds[k]
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError(f'{self.id}.{k} must be in [{lo}, {hi}]')
        return p

    def create(self, anchor, bias, overrides=None):
        return self.factory(anchor, bias, self.parameters(overrides))


class Method:
    phase = 'approach'
    done = False

    def __init__(self, anchor, bias, parameters):
        self.anchor, self.bias, self.p = anchor, bias, parameters
        self.started = None

    def start(self, now, history):
        self.started = now

    def tracking(self, reference, kp=None, kd=None):
        x, v, a = reference
        kp = self.p.get('kp', 40.) if kp is None else kp
        kd = self.p.get('kd', 1.8) if kd is None else kd
        torque = self.bias*self.p.get('bias_scale',1.)-self.p.get('inertia', .02)*a-self.p.get('friction', .02)*v
        return Command(self.anchor-x, -v, kp, kd, torque)

    def hold(self):
        return Command(self.anchor, 0, 40, 1.8, self.bias)

    def reference(self, elapsed):
        return self.curve.at(elapsed)

    def update(self, sample, now):
        t = now-self.started
        self.done = t >= self.curve.duration
        ref = self.reference(t)
        self.phase = 'settling' if self.done else ('withdrawal' if ref[1] < 0 else 'approach')
        return self.hold() if self.done else self.tracking(ref)


TRACK = dict(kp=40., kd=1.8, inertia=.02, friction=.02,bias_scale=1.)
TRACK_BOUNDS = dict(kp=(0., 80.), kd=(0., 4.), inertia=(.003, .06), friction=(0., .2),bias_scale=(.4,1.3))
