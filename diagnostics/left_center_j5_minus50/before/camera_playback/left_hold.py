"""Playback-local left center and native CSP hold; no additional CAN owner."""
from collections import deque
import math

import numpy as np

from centering.motors import Motors, MOTOR_RUNNING_STATE, parameter

LEFT_CENTER = np.radians([0., 0., 0., 0., -25., 0., 0.])
# Match the raw motor-8 readout, not an estimated millimeter opening.
# Internal control coordinates negate raw feedback; the CAN target is positive.
LEFT_GRIPPER_CLOSED_DEGREES = 14.16
LEFT_GRIPPER_TARGET = -math.radians(LEFT_GRIPPER_CLOSED_DEGREES)
LEFT_TOLERANCE = math.radians(3.)
LEFT_GRIPPER_TOLERANCE = math.radians(1.)


class LeftCenterMonitor:
    """Verify arrival, then supervise a fixed firmware-held target indefinitely."""
    def __init__(self, now, goal=LEFT_CENTER):
        self.goal = np.asarray(goal, dtype=float).copy()
        self.started = now
        self.history = deque()
        self.settled_since = None
        self.ready = False
        self.fault = None

    def update(self, states, now):
        if self.fault:
            return  # Report once so the right arm's existing recovery can run.
        try:
            samples = [states['left', i] for i in range(1, 9)]
            if any(now - stamp >= .3 or mode != MOTOR_RUNNING_STATE
                   for _, mode, stamp in samples):
                raise RuntimeError('Left hold lost fresh running feedback')
            actual = np.array([q for q, _, _ in samples])
            if not np.isfinite(actual).all():
                raise RuntimeError('Invalid left hold feedback')
            error = np.abs(np.r_[self.goal, LEFT_GRIPPER_TARGET] - actual)
            within = np.all(error <= np.r_[np.full(7, LEFT_TOLERANCE), LEFT_GRIPPER_TOLERANCE])
            if self.ready:
                if not within:
                    raise RuntimeError('Left arm/gripper drifted from its held target')
                return
            self.history.append((now, actual))
            while self.history and now - self.history[0][0] > 2.5:
                self.history.popleft()
            if self.history and now - self.history[0][0] >= 2.4:
                span = np.ptp(np.array([q for _, q in self.history]), axis=0)
                stalled = (error > math.radians(8.)) & (span < math.radians(1.))
                if np.any(stalled):
                    raise RuntimeError('STALL: left motor ' + str(int(np.flatnonzero(stalled)[0]) + 1))
            if within:
                if self.settled_since is None:
                    self.settled_since = now
                self.ready = now - self.settled_since >= .6
            else:
                self.settled_since = None
            if not self.ready and now - self.started > 30.:
                raise RuntimeError('Left center/gripper position timeout')
        except RuntimeError as exc:
            self.ready = False
            self.fault = str(exc)
            raise


class LeftHoldDrive(Motors):
    """Permit the one left hold target without widening right-gripper limits."""

    def command_allowed(self, frame):
        return (frame == parameter(8, 0x7016, -LEFT_GRIPPER_TARGET)
                or super().command_allowed(frame))


def left_setup_view(owner):
    """Borrow the parent's sockets/states, but keep side and readbacks separate.

    Never poll/close this view: the GUI retains both CAN locks and all feedback
    ownership. Only the existing confirmed-CSP setup, fixed target and disable
    methods use it; right-arm writers cannot route through it accidentally.
    """
    view = LeftHoldDrive.__new__(LeftHoldDrive)
    view.control_side, view.control_gripper = 'left', True
    view.sockets, view.states = owner.sockets, owner.states
    view.modes = {}
    view.isolated_motors = set()
    view.right_joint7_session = None
    view.right_joint7_query_period = None
    view.last_right_joint7_query = 0.
    view.active = False
    return view
