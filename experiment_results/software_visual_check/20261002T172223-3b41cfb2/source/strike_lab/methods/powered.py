"""Fully powered smooth trajectories, with nonzero bottom acceleration."""
from ..curves import Dip, CosineDip
from .base import Method, Definition, TRACK, TRACK_BOUNDS


class Powered(Method):
    def __init__(self, *args):
        super().__init__(*args)
        self.curve = Dip(self.p['down_time'], self.p['up_time'], self.p['curvature'])


class Cosine(Method):
    def __init__(self, *args):
        super().__init__(*args)
        self.curve = CosineDip(self.p['duration'])


DEFAULTS = dict(TRACK, down_time=.22, up_time=.20, curvature=.8)
BOUNDS = dict(TRACK_BOUNDS, down_time=(.13,.4), up_time=(.13,.4), curvature=(.2,5.))
DEFINITION = Definition('powered', 'Powered S-curve', 'powered',
    'Asymmetric quintic down/up path; fixed ten-degree bottom.', DEFAULTS, BOUNDS, Powered)
COSINE = Definition('cosine', 'Powered rounded sinusoid', 'powered',
    'Single smooth sin^4 pulse, not two point-to-point moves.',
    dict(TRACK, duration=.48), dict(TRACK_BOUNDS, duration=(.30,.8)), Cosine)
