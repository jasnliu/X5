"""Ride-anchored snare timing calibration; observes actual worker releases.

No arm transport here. Only the existing J6 worker schedules/executes strokes.
The acoustic pairing, signed absolute correction and two-good-hit lock are
shared with hi-hat calibration. The reference descent is NOT counted twice.
"""
import math
from .hihat_sync import (HiHatSynchronizer, Closure, TimingFault,
                         CALIBRATION_TIMEOUT)


class SnareSynchronizer(HiHatSynchronizer):
    def __init__(self, down_seconds, log=lambda row: None):
        super().__init__(None, log, calibration=True, instrument='snare',
                         nominal_lead=down_seconds)
        self.last_strike = 0

    @property
    def complete(self):
        return self.locked  # Workflow waits independently for full worker return.

    def observe(self, status):
        if not status.calibrating or status.strike_index == 0:
            return
        if status.scheduled_at is None or status.scheduled_at < self.epoch-1e-6:
            return  # Old status before the worker accepts the new calibration.
        if status.strike_index == self.last_strike:
            return
        if status.strike_index != self.last_strike+1:
            raise TimingFault('Snare calibration release evidence was lost')
        expected = self.epoch+(status.strike_index-1)*self.period
        if (status.released_at is None or not math.isfinite(status.released_at)
                or abs(status.scheduled_at-expected) > 1e-6
                or not math.isfinite(status.advance) or abs(status.advance) > self.max_advance):
            raise TimingFault('Snare calibration release/grid evidence invalid')
        self.last_strike = status.strike_index
        self.pair_advance = status.advance
        self.pending.append(Closure(2*(status.strike_index-1), expected,
                                    status.released_at, status.advance))
        self.log(dict(kind='command', index=2*(status.strike_index-1), target=expected,
                      at=status.released_at, advance=status.advance,
                      reference_descent=self.nominal_lead))

    def tick(self, now, available):
        if not self.active:
            return
        self.evaluate(now, available)
        if now-self.started > CALIBRATION_TIMEOUT and not self.complete:
            raise TimingFault('Ride/snare synchronization did not converge within 45 seconds')

