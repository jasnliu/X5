"""Unmodified previous hardwaretest algorithm: comparison baseline, not a favorite."""
from dataclasses import replace
from camera_playback.mit_strike import StrikeController, StrikeSettings
from ..config import TARGET
from .base import Method, Definition


class Baseline(Method):
    def __init__(self, anchor, bias, parameters):
        super().__init__(anchor, bias, parameters)
        self.c = StrikeController(anchor, replace(StrikeSettings(), **parameters))
        self.c.bias = bias

    def start(self, now, history):
        super().start(now, history)
        for sample in history:
            self.c.update(sample, sample.at)
        self.c.strike(TARGET, history[-1], now)

    def update(self, sample, now):
        command = self.c.update(sample, now)
        self.phase = self.c.phase
        self.done = self.c.phase == 'hold'
        return command


DEFINITION = Definition('baseline', 'Current MIT baseline', 'baseline',
    'Previous gravity/catch controller; same fixed ten-degree request.',
    dict(fall_kd=.04, brake_accel=40., return_accel=20., command_latency=.004),
    dict(fall_kd=(.001,.15), brake_accel=(10.,60.), return_accel=(10.,40.),
         command_latency=(.001,.012)), Baseline)
