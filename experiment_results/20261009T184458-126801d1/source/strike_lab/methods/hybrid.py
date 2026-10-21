"""Powered impulse, coast, predictive upward catch; no impact trigger."""
import math
from camera_playback.mit_strike import Command
from .gravity import Gravity, DEFAULTS, BOUNDS
from .base import Definition


class Hybrid(Gravity):
    def falling(self, sample, now, depth):
        t = now-self.started
        pulse = (math.sin(math.pi*t/self.p['kick_time'])**2
                 if 0 <= t < self.p['kick_time'] else 0.)
        self.phase = 'drive' if pulse else 'coast'
        return Command(sample.position, 0, 0, self.p['fall_kd'], -self.p['kick_torque']*pulse)


DEFINITION = Definition('hybrid', 'Accelerate / coast / withdraw', 'hybrid',
    'Smooth downward torque burst followed by low-impedance coast and predictive catch.',
    dict(DEFAULTS, kick_time=.06, kick_torque=.18),
    dict(BOUNDS, kick_time=(.02,.12), kick_torque=(0.,.6)), Hybrid)
