"""Small waypoint spline family, suitable for bounded between-trial optimization."""
from ..curves import Dip
from .base import Method, Definition
from .powered import DEFAULTS, BOUNDS


class Optimized(Method):
    def __init__(self, *args):
        super().__init__(*args)
        self.curve = Dip(self.p['down_time'], self.p['up_time'], self.p['curvature'],
                         self.p['knot_fraction'], self.p['velocity_shape'])


DEFINITION = Definition('optimized', 'Optimized waypoint spline', 'optimized',
    'Adjustable smooth approach knot and bottom curvature; endpoint always ten degrees.',
    dict(DEFAULTS, knot_fraction=.55, velocity_shape=1.),
    dict(BOUNDS, knot_fraction=(.35,.7), velocity_shape=(.8,1.2)), Optimized)
