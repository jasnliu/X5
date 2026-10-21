"""Slow, bounded J7 centering without disabling or changing a loaded drive's mode."""
import math
from .mit_strike import Command


class MitCenterReturn:
    def __init__(self, position, goal, bias, now):
        if not all(math.isfinite(x) for x in (position, goal, bias, now)):
            raise ValueError('Invalid MIT center return')
        self.position = self.actual = float(position)
        self.goal = float(goal)
        self.velocity = 0.
        self.bias = max(-1.5, min(1.5, bias))
        self.at = now

    def update(self, actual, now):
        if not math.isfinite(actual) or not math.isfinite(now) or now < self.at:
            raise RuntimeError('Invalid MIT center feedback')
        elapsed = now-self.at
        dt = min(.04, elapsed)
        measured_velocity = (actual-self.actual)/elapsed if elapsed > .001 else 0.
        if elapsed > .25:
            # Never catch up after a GUI delay. Resume from the measured hold.
            self.position, self.velocity = actual, 0.
        distance = self.goal-self.position
        desired_velocity = math.copysign(min(.35, math.sqrt(2*.6*abs(distance))), distance)
        old_velocity = self.velocity
        self.velocity += max(-.6*dt, min(.6*dt, desired_velocity-self.velocity))
        step = .5*(old_velocity+self.velocity)*dt
        if abs(step) >= abs(distance):
            self.position, self.velocity = self.goal, 0.
        else:
            self.position += step
        # Do not accumulate a large position error against an obstruction.
        self.position = max(actual-.04, min(actual+.04, self.position))
        if abs(self.goal-actual) < .04 and abs(measured_velocity) < .1:
            self.bias = max(-1.5, min(1.5, self.bias+60*(self.goal-actual)*dt))
        kp, kd = 40., 1.8
        pd = kp*(self.position-actual)+kd*(self.velocity-measured_velocity)
        torque = max(-3., min(3., pd+self.bias))-pd
        if abs(torque) > 12:
            raise RuntimeError('MIT center feedback changed too fast')
        self.at, self.actual = now, actual
        return Command(self.position, self.velocity, kp, kd, torque)
