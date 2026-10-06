"""Feedforward torque-led stroke; smoothly restore hold tracking on withdrawal."""
from ..curves import smooth
from .base import Definition
from .powered import Powered, DEFAULTS, BOUNDS


class Torque(Powered):
    def tracking(self, reference, kp=None, kd=None):
        # Reference acceleration specifies the rounded drive/braking torque.
        # Low gains in the stroke; full hold authority only near the upper end.
        progress = self.elapsed/self.curve.duration
        w = smooth((progress-.65)/.35)
        return super().tracking(reference, self.p['kp']+(40-self.p['kp'])*w,
                                self.p['kd']+(1.8-self.p['kd'])*w)

    def update(self, sample, now):
        self.elapsed = now-self.started
        return super().update(sample, now)


DEFINITION = Definition('torque', 'Rounded torque reversal', 'torque',
    'Acceleration-derived torque pulses with weak stroke feedback, then blended upper hold.',
    dict(DEFAULTS, kp=1., kd=.12), BOUNDS, Torque)
