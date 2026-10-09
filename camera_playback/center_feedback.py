"""Measured boundary overshoot ONLY for the center leg of arm playback.

The center target can coincide with a joint stop (left J4 = 0). Firmware may
settle slightly beyond it. Use the EXISTING .20-degree center acceptance for
that boundary joint, never as a widened command/recording/strike limit. Check
the TCP at its actual measured pose, not at a clipped substitute.
"""
import math
import numpy as np
from safe_zone.geometry import MEMBERSHIP_BUFFER_M

CENTER_BOUNDARY_TOLERANCE = math.radians(.20)


def check_center_feedback(geometry, actual):
    q = np.asarray(actual, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError('Invalid center feedback')
    checked = q.copy()
    for i in range(7):
        if (geometry.center[i] == geometry.lower[i]
                and geometry.lower[i]-CENTER_BOUNDARY_TOLERANCE <= q[i] < geometry.lower[i]):
            checked[i] = geometry.lower[i]
        elif (geometry.center[i] == geometry.upper[i]
                and geometry.upper[i] < q[i] <= geometry.upper[i]+CENTER_BOUNDARY_TOLERANCE):
            checked[i] = geometry.upper[i]
    geometry.check(checked, measured=True)
    if not geometry.zone.contains(geometry.tcp(q), MEMBERSHIP_BUFFER_M):
        raise ValueError('Measured center motion outside existing zone1 TCP envelope')


def check_center_path(geometry, start, target):
    """Validate an inward departure/recovery from a measured center boundary.

    Every commanded endpoint is strictly legal. Interpolation stays on the
    actual measured line; only tiny center-boundary overshoot is accepted.
    """
    geometry.check(target)
    n = max(2, int(np.ceil(np.max(np.abs(np.asarray(target)-start))/math.radians(.25)))+1)
    for q in np.linspace(start, target, n):
        check_center_feedback(geometry, q)
