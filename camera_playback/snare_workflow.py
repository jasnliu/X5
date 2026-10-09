"""Beat lifecycle adapter; delegates every motor stroke to snare.sh's reference."""
import math
import time

import numpy as np

from snare_lab.runner import prepare_strike
from snare_lab.motion import build_method
from .snare_beat import SNARE_DEGREES, SnareSession, validate_tempo

ARMING = 'LEFT SNARE ARMING AT CENTER'


def preflight(dual):
    """Called while disabled when a recording is loaded; no CAN operations."""
    if not (getattr(dual.app, 'hybrid_enabled', False) or getattr(dual.app, 'test_mode', False)):
        return
    tuning = prepare_strike(dual.g, dual.recording, SNARE_DEGREES)
    template = build_method(-float(dual.recording.last_joints[5]), 0., tuning[0], SNARE_DEGREES, tuning[2])
    dual.snare_tuning, dual.snare_template = tuning, template


def check_tempo(app):
    dual = getattr(app, 'dual', None)
    if getattr(dual, 'snare_template', None) is not None:
        validate_tempo(app.strike_bpm, dual.snare_template, dual.snare_tuning[1])


def session(app):
    if getattr(app, 'bus', None) is None:
        return None
    return vars(app.bus).get('left_joint6_session') or vars(app.bus).get('snare_simulation')


def arm(dual):
    a = dual.app
    if time.monotonic()-dual.snare_arming_at > 15.:
        raise RuntimeError('Snare centered mode setup timeout; NOT relaxed')
    s = session(a)
    if s is None:
        try:
            a.bus.snare_center_evidence()
        except RuntimeError:
            return False  # Keep the center targets until strict settling proof.
        _, rules, limits = dual.snare_tuning
        SnareSession(a.bus, dual.snare_template, rules, limits,
                     -float(dual.g.upper[5]), -float(dual.g.lower[5]))
        return False
    state = s.status
    if state.error:
        raise RuntimeError('Left snare setup: '+state.error)
    return state.armed


def start(app):
    s = session(app)
    if s is None:
        if getattr(getattr(app, 'dual', None), 'snare_template', None) is not None:
            raise RuntimeError('Left snare controller missing')
        return
    s.start_swing(app.strike_bpm)
    app.bus.left_monitor = None  # J6 intentionally leaves endpoint during hit.
    app.bus.left_playback_active = True  # Fresh running/gripper supervision remains.
    print('SNARE: powered +11°; 1 or 2 random eighth notes (1–8) per four-quarter-note measure; '
          'spaced for full unchanged return, including bar boundaries', flush=True)


def finish(app):
    s = session(app)
    if s is not None:
        s.finish()


def finished(app):
    s = session(app)
    return s is None or s.finished


def tick(dual, now):
    a, s = dual.app, session(dual.app)
    if s is None:
        return
    state = s.status
    if state.error:
        raise RuntimeError('Left snare: '+state.error+'; powered hold, NOT relaxed')
    if state.armed and now-state.at > .15:
        raise RuntimeError('Left snare worker status stale')
    if state.moving or state.scheduled:
        q = dual.positions()
        anchor = dual.recording.last_joints
        held = [0, 1, 2, 3, 4, 6]
        if np.max(np.abs(q[held]-anchor[held])) > math.radians(dual.snare_tuning[2].held_joint_drift_deg):
            raise RuntimeError('Left held joints moved during snare beat')
        dual.g.check(q, measured=True)
    if state.completed > getattr(dual, 'snare_seen_completed', 0):
        dual.snare_seen_completed = state.completed
        evidence = 'simulated reference' if getattr(a, 'test_mode', False) else 'measured peak'
        print(f'SNARE measure {state.measure}, eighth note {state.beat}/8: +11° powered; '
              f'{evidence} {math.degrees(state.peak_drop):.2f}°; returned', flush=True)


def start_simulation(app):
    from .snare_beat import SnareReferenceSimulation
    from .simulation import SimulatedMotors
    dual = getattr(app, 'dual', None)
    if (isinstance(app.bus, SimulatedMotors) and getattr(dual, 'snare_template', None) is not None):
        app.bus.snare_simulation = SnareReferenceSimulation(app.bus, dual.snare_template, dual.snare_tuning[1])
        start(app)


def simulation_epoch(app, epoch):
    s = vars(app.bus).get('snare_simulation') if getattr(app, 'bus', None) is not None else None
    if s is not None:
        s.set_epoch(epoch)
