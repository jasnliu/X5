"""Uncalibrated motor-1 quarter notes on the read-only swing grid.

The ESP32 owns the exact keyboard strike: clockwise PWM 180/255 for 50 ms,
then coast/release. The host only sends K; it never times the pulse or return.
"""
import math

from .hihat_sync import MAX_LATENESS, TimingFault


class BassDrumScheduler:
    """One kick per quarter, with no pickup, extras, offset or catch-up burst.

    Created only for swing, not calibration. Its owner cancels it on stop/fault.
    send() returns the actual monotonic send time; epoch is beat 1 of the ride.
    """

    def __init__(self, send, period, log=lambda row: None):
        if not math.isfinite(period) or period <= .050:
            raise ValueError('Bass beat period must exceed the 50 ms strike pulse')
        self.send, self.period, self.log = send, period, log
        self.index = 0

    def tick(self, now, epoch):
        if epoch is None:
            return  # Wait for the ride pickup to establish the common grid.
        target = epoch + self.index * self.period
        if now < target:
            return
        if now - target > MAX_LATENESS:
            raise TimingFault('Bass drum deadline missed; refusing catch-up burst')
        actual = self.send()
        if not math.isfinite(actual) or actual < now - .001 or actual - target > MAX_LATENESS:
            raise TimingFault('Bass drum serial delivery missed deadline')
        self.log(dict(kind='bass_command', instrument='bass', at=actual,
                      index=self.index, beat=self.index % 4 + 1, target=target,
                      advance=0., lateness=actual - target))
        self.index += 1
