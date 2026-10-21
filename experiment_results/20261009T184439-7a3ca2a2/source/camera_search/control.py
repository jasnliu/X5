"""Motion-controller details specific to the camera search sequence."""

import numpy as np

from goal_motion.control import GoalControl, TOLERANCE


# The motor feedback maps 16 bits across 25.14 radians.  A measured joint can
# therefore straddle the original 3 degree settling threshold by one encoder
# count even while it is physically stationary.  Keep the original tolerance
# and settling time, but include exactly one measurable encoder increment.
ENCODER_LSB_RAD = 25.14 / 65535
SETTLE_SECONDS = 0.6


class CameraGoalControl(GoalControl):
    """GoalControl with one-encoder-count settling hysteresis.

    Command generation, speed, correction, stall detection, timeout, and the
    nominal 3 degree tolerance all remain those of the original right-arm
    program.  Only the reached test accounts for feedback quantization.
    """

    def __init__(self, desired, lower, upper, now):
        super().__init__(desired, lower, upper, now)
        self.quantized_tolerance_since = None

    def update(self, actual, now):
        command, error, _ = super().update(actual, now)
        if np.max(np.abs(error)) <= TOLERANCE + ENCODER_LSB_RAD:
            if self.quantized_tolerance_since is None:
                self.quantized_tolerance_since = now
        else:
            self.quantized_tolerance_since = None
        reached = (self.quantized_tolerance_since is not None and
                   now - self.quantized_tolerance_since >= SETTLE_SECONDS)
        return command, error, reached
