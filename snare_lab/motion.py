"""Faster snare descent with unchanged return and existing dynamic limits."""
import math
from functools import lru_cache
from types import SimpleNamespace

from numpy.polynomial import Polynomial

from strike_lab.curves import Quintic
from strike_lab.engine import validate_curve
from strike_lab.methods import METHODS
from .config import (STRIKE_METHOD_ID, DOWNWARD_SPEED_MULTIPLIER,
                     DOWNWARD_PEAK_GAIN, DOWNWARD_PEAK_FRACTION)


class ScaledCurve:
    def __init__(self, original, scale):
        self.original, self.scale = original, scale
        self.duration = original.duration * scale

    def at(self, elapsed):
        x, v, a = self.original.at(elapsed / self.scale)
        return x, v / self.scale, a / self.scale ** 2


def _extrema(poly):
    points = [0., 1.]
    points.extend(float(r.real) for r in poly.deriv().roots()
                  if abs(r.imag) < 1e-9 and 0. < r.real < 1.)
    values = [float(poly(x)) for x in points]
    return min(values), max(values)


def _descent_fits(curve, target, limits):
    """Check polynomial extrema, not only sampled references, for the new leg."""
    if isinstance(curve, AcceleratingDescent):
        return all(_descent_fits(leg, target, limits) for leg in curve.legs)
    p = Polynomial(curve.c)
    for order, bound in enumerate((target, limits.velocity, limits.acceleration, limits.jerk)):
        lo, hi = _extrema(p.deriv(order) / curve.duration ** order)
        if not all(math.isfinite(v) for v in (lo, hi)):
            return False
        if order == 0:
            if lo < -1e-10 or hi > target + 1e-10:
                return False
        elif order == 1:
            if lo < -1e-10 or hi > bound:
                return False  # No reversal before reaching the bottom.
        elif max(abs(lo), abs(hi)) > bound:
            return False
    return True


class AcceleratingDescent:
    """Build speed into the late stroke, then join the unchanged return smoothly."""
    def __init__(self, approach, braking, peak_fraction, peak_speed):
        self.legs = (approach, braking)
        self.peak_at = approach.duration
        self.peak_fraction, self.peak_speed = peak_fraction, peak_speed
        self.duration = approach.duration + braking.duration

    def at(self, elapsed):
        if elapsed < self.peak_at:
            return self.legs[0].at(elapsed)
        return self.legs[1].at(elapsed - self.peak_at)


@lru_cache(maxsize=128)
def _accelerating_descent(target, bottom_acceleration, old_peak,
                         velocity, acceleration, jerk):
    # Cache the calculation-only plan: physical execution reuses preflight's
    # result instead of searching while a joint is held in MIT mode.
    limits = SimpleNamespace(velocity=velocity, acceleration=acceleration, jerk=jerk)
    desired_peak = max(old_peak, min(old_peak * DOWNWARD_PEAK_GAIN, velocity * .99))
    for gain in (1., .95, .90, .85):
        peak = max(old_peak, desired_peak * gain)
        for offset in (0., .02, .04, .06, .08):
            fraction = DOWNWARD_PEAK_FRACTION - offset
            knot = target * fraction
            # Smooth increasing velocity, zero acceleration at each end.
            approach = Quintic(0., 0., 0., knot, peak, 0., 2 * knot / peak)
            if not _descent_fits(approach, target, limits):
                continue
            for step in range(61):
                factor = .70 + .005 * step
                duration = 2 * (target-knot) / peak * factor
                braking = Quintic(knot, peak, 0., target, 0., bottom_acceleration, duration)
                if not _descent_fits(braking, target, limits):
                    continue
                # Do not introduce a second acceleration/velocity peak.
                if _extrema(Polynomial(braking.c).deriv(2) / duration**2)[1] > 1e-9:
                    continue
                return AcceleratingDescent(approach, braking, fraction, peak)
    return None  # Keep the existing validated descent if no new shape fits.


class FastDownCurve:
    def __init__(self, original, previous_down, target, limits,
                 multiplier=DOWNWARD_SPEED_MULTIPLIER):
        self.original, self.previous_down = original, previous_down
        self.up = original.duration - previous_down
        # Match the original return's position, velocity and acceleration at
        # the join. Simply time-scaling descent would introduce an accel jump.
        bottom = original.at(previous_down)

        def candidate(duration):
            return Quintic(0., 0., 0., target, 0., bottom[2], duration)

        requested = previous_down / multiplier
        self.down = requested
        self.descent = candidate(requested)
        if not _descent_fits(self.descent, target, limits):
            slow = previous_down
            if not _descent_fits(candidate(slow), target, limits):
                raise ValueError('Original snare descent exceeds dynamic limits')
            fast = requested
            for _ in range(45):
                trial = (fast + slow) / 2
                if _descent_fits(candidate(trial), target, limits):
                    slow = trial
                else:
                    fast = trial
            # Keep a small margin rather than landing exactly on a limit.
            self.down = min(previous_down, slow * 1.001)
            self.descent = candidate(self.down)
        self.previous_fast_down = self.down
        self.previous_descent = self.descent
        old_peak = _extrema(Polynomial(self.descent.c).deriv() / self.down)[1]
        curved = _accelerating_descent(target, bottom[2], old_peak,
                                      limits.velocity, limits.acceleration, limits.jerk)
        self.curved_acceleration = curved is not None
        if curved is not None:
            self.descent = curved
            self.down = curved.duration
        self.duration = self.down + self.up

    def at(self, elapsed):
        if elapsed >= self.duration:
            return 0., 0., 0.
        if elapsed >= self.down:
            return self.original.at(self.previous_down + elapsed - self.down)
        return self.descent.at(elapsed)


def build_method(anchor, bias, parameters, degrees, limits):
    method = METHODS[STRIKE_METHOD_ID].create(
        anchor, bias, parameters, target=math.radians(degrees))
    curve = method.curve
    scale, last = 1., None
    # Use the same normalized sampling grid as the shared reference validator.
    # Velocity, acceleration and jerk fall as 1/s, 1/s^2 and 1/s^3.
    for i in range(1001):
        t = curve.duration * i / 1000
        _, v, a = curve.at(t)
        if not all(math.isfinite(z) for z in (v, a)):
            raise ValueError('Nonfinite snare reference')
        scale = max(scale, abs(v) / limits.velocity,
                    math.sqrt(abs(a) / limits.acceleration))
        if last is not None:
            jerk = abs((a-last[1]) / (t-last[0]))
            scale = max(scale, (jerk / limits.jerk) ** (1/3))
        last = t, a
    if scale > 1.:
        method.curve = ScaledCurve(curve, scale * 1.001)
    previous_down = parameters['down_time'] * getattr(method.curve, 'scale', 1.)
    method.curve = FastDownCurve(method.curve, previous_down, method.target, limits)
    # Retain shape, reversal, finite-value and all shared dynamic checks.
    validate_curve(method, limits)
    return method


def stroke_timeout(method, limits):
    return max(limits.trial_timeout, method.curve.duration + 1.)
