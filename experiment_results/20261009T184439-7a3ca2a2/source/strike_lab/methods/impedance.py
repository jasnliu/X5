"""Smooth moving virtual spring with low descent and higher withdrawal stiffness."""
from ..curves import smooth
from .powered import Powered, DEFAULTS, BOUNDS
from .base import Definition


class Impedance(Powered):
    def update(self, sample, now):
        self.elapsed = now-self.started
        return super().update(sample, now)

    def tracking(self, reference, kp=None, kd=None):
        w = smooth((self.elapsed/self.curve.duration-.25)/.5)
        return super().tracking(reference,
                                self.p['kp']+(self.p['return_kp']-self.p['kp'])*w,
                                self.p['kd']+(self.p['return_kd']-self.p['kd'])*w)


DEFINITION = Definition('impedance', 'Variable virtual spring', 'impedance',
    'Moving spring reference with smoothly scheduled stiffness and damping.',
    dict(DEFAULTS, kp=8., kd=.3, return_kp=40., return_kd=1.8),
    dict(BOUNDS, return_kp=(10.,80.), return_kd=(.3,4.)), Impedance)
