"""Hardware-only adapter between the existing playback UI and hybrid worker."""
import math
import time

import numpy as np

from centering.motors import MOTOR_RUNNING_STATE
from safe_zone.geometry import MEMBERSHIP_BUFFER_M
from strike_lab.config import StrikeGoal
from .hybrid_strike import HybridSession, load_tuning
from .strike import StrikePlan, _path_inside


RESTORING = 'HYBRID LEAVING MIT SESSION'
FINISHING = 'HYBRID FINISHING RETURN'
FAULT = 'HYBRID HOLD FAULT'


def enabled(app):
    return getattr(app, 'hybrid_enabled', False)


def engaged(app):
    return (getattr(app, 'hybrid_session', None) is not None
            or getattr(app, 'hybrid_restoring', False) or getattr(app, 'phase', None) == FAULT)


def prepare(app):
    """Reuse the accepted camera pose, never experiment's record3 preparation."""
    tuning = getattr(app, 'hybrid_tuning', None) or load_tuning()
    parameters, rules, limits = tuning
    plan = app.strike_plan
    anchor = plan.anchor_joints.copy()
    # Intersect the already-validated path with the experiment's 12-degree
    # corridor. Do not silently widen either the geometric or experiment limit.
    lower = max(float(plan.targets[-1][6]), anchor[6]-math.radians(limits.hard_depth_deg))
    upper = min(float(app.upper[6]), anchor[6]+math.radians(limits.upper_excursion_deg))
    upper_pose = anchor.copy()
    upper_pose[6] = upper
    if not _path_inside(app.hill_ik.model, app.hill_ik.zone, anchor, upper_pose):
        upper = float(anchor[6])
    targets = []
    for target in plan.targets:
        try:
            StrikeGoal(math.degrees(anchor[6]-target[6])).validate(rules, limits, anchor[6])
        except ValueError:
            break
        targets.append(target)
    if not targets:
        raise ValueError('No 5-degree hybrid search target fits the validated limits')
    app.strike_plan = StrikePlan(anchor, tuple(targets))
    app.hybrid_anchor = anchor
    app.hybrid_tuning = tuning
    app.hybrid_session = HybridSession(app.bus, float(anchor[6]), lower, upper, tuning)
    app.hybrid_seen_completed = 0
    app.hybrid_seen_bottoms = 0
    app.hybrid_main_count = 0
    app.hybrid_stopping = False
    app.hybrid_restoring = False
    print(f'HYBRID J7: experiment hardware tuning; kick {parameters["kick_torque"]:g} Nm / '
          f'{parameters["kick_time"]*1000:g} ms; {limits.hz:g} Hz; '
          f'search 5° through {5+len(targets)-1}°; fixed camera anchor; '
          '100 BPM swing permits partial returns after 4° rebound', flush=True)


def begin_search(app):
    app.control = None
    app.hold_until = None
    if getattr(app, 'hybrid_session', None) is None:
        prepare(app)
    session = app.hybrid_session
    app.hybrid_expected_count = session.status.count+1
    app.strike_hit_pending = None
    app.strike_attempt_motion_seen = False
    app.strike_attempt_started_at = None  # Set from worker's actual release time.
    app.strike_attempt_returned_at = None
    app.strike_sound_deadline = None
    session.request('search', math.radians(app._current_strike_degrees()))
    app.phase = 'STRIKE MOVING OUT'
    app.status.set(f'Hybrid hit search: J7 −{app._current_strike_degrees()}° '
                   '— waiting for fresh settled hold, then impulse/coast/catch/return')


def sync_search(app, status=None):
    session = getattr(app, 'hybrid_session', None)
    if session is None or not app.strike_active:
        return False
    s = session.status if status is None else status
    if s.error or s.count < app.hybrid_expected_count:
        return False
    app.strike_attempt_started_at = s.released_at
    app.strike_attempt_motion_seen |= s.peak_drop >= math.radians(3.)
    # Audio may run before the display tick; timestamp gating must still use
    # the actual completed return, not a delayed Tk observation.
    app.strike_attempt_returned_at = s.returned_at
    return True


def start_swing(app, events):
    app.hybrid_main_count = app.hybrid_seen_bottoms = 0
    app.hybrid_seen_closed = 0
    app.hybrid_session.request('swing', math.radians(app.continuous_strike_degrees), events)
    app.status.set(f'HYBRID first detected depth {app.continuous_strike_degrees}°: '
                   '100 BPM swing + hi-hat; 4° minimum rebound before partial-return releases')
    print(app.status.get(), flush=True)


def request_stop(app, detail='Stop requested'):
    if getattr(app, 'hybrid_stopping', False) or getattr(app, 'hybrid_restoring', False):
        return
    app.strike_active = False
    app.strike_sound_deadline = None
    app.control = None
    app._stop_hihat_sequence()
    app.hybrid_stop_request = app.hybrid_session.request('finish')
    app.hybrid_stopping = True
    app.continuous_stop_requested = True
    app.phase = FINISHING
    app.status.set(detail+' — finishing hybrid return, then Center + Relax')
    app.center_relax_button.config(state='disabled')


def close(app):
    session = getattr(app, 'hybrid_session', None)
    if session is not None:
        session.stop()  # Ownership is retained if joining fails.
        app.hybrid_session = None


def fault(app, message):
    """Never let inherited CSP recovery write into an active/unknown MIT mode."""
    first = app.phase != FAULT
    app.control = app.setup = None
    app.strike_active = False
    app.strike_sound_deadline = None
    app.continuous_strike_active = False
    app._stop_hihat_sequence()
    try:
        close(app)
    except Exception as exc:
        message = str(message)+'; worker stop failed: '+str(exc)
    if first:
        app.hybrid_fault_hold = None
        app.hybrid_fault_last_send = 0.
        if app.bus and app.bus.active and app.bus.fresh():
            app.hybrid_fault_hold = float(app.arm()[6])
    # The worker faults to a current-pose hold. Do not initiate new motion from
    # an unverified controller state. Emergency Relax remains available.
    app.phase = FAULT
    app.hybrid_stopping = False
    app.status.set('HYBRID FAULT: '+str(message)+' — automatic motion stopped; '
                   'use Emergency Relax or physical stop, then restart')
    if first:
        print(app.status.get(), flush=True)
    maintain_fault(app, time.monotonic())


def maintain_fault(app, now):
    # A GUI-detected failure may stop an otherwise healthy worker halfway
    # through its torque pulse. Replace that retained command with a fixed
    # live-pose hold AFTER joining. Never enable, rebase repeatedly, or recenter.
    q = getattr(app, 'hybrid_fault_hold', None)
    if (q is not None and app.bus and app.bus.active
            and getattr(app, 'hybrid_session', None) is None
            and now-app.hybrid_fault_last_send >= .1):
        try:
            app.bus.hold_right_joint7_unknown_mode(q)
        except Exception as exc:
            app.status.set('HYBRID FAULT: hold delivery failed — use physical stop: '+str(exc))
        app.hybrid_fault_last_send = now


def tick(app, now):
    if app.phase == FAULT:
        maintain_fault(app, now)
        return
    if getattr(app, 'hybrid_restoring', False):
        state = app.bus.right_joint7_operating_state()
        if (app.bus.right_joint7_mode_readback() == 5 and state is not None
                and state[0] == MOTOR_RUNNING_STATE and state[1] >= app.hybrid_csp_at):
            app.hybrid_restoring = False
            app.hybrid_stopping = False
            app.continuous_strike_active = False
            app._cancel_continuous_striking()
            app.begin_stage('CENTER RELAX RECENTERING', app.center_goal)
            app.status.set('Hybrid stopped; position mode confirmed — centering, then relaxing')
        elif now > app.hybrid_restore_deadline:
            fault(app, 'Position-mode confirmation missing after hybrid return')
        elif now-app.hybrid_last_enable >= .1:
            app.hybrid_csp_at = app.bus.reassert_right_joint7_csp_hold(float(app.hybrid_anchor[6]))
            app.hybrid_last_enable = now
        return
    session = getattr(app, 'hybrid_session', None)
    if session is None:
        return
    s = session.status
    if s.error:
        fault(app, s.error)
        return
    if s.sample is not None and now-s.at > .1:
        fault(app, 'Hybrid worker status heartbeat stale')
        return
    limit = math.radians(app.hybrid_tuning[2].held_joint_drift_deg)
    if np.max(np.abs(app.arm()[:6]-app.hybrid_anchor[:6])) > limit:
        fault(app, 'J1-J6 moved beyond the experiment held-joint limit')
        return
    if app.continuous_strike_active and s.closed_count > getattr(app, 'hybrid_seen_closed', 0):
        app.hybrid_seen_closed = s.closed_count
        print(f'J7 STRIKE LOW strike {s.closed_count} [{s.closed_label}, hybrid]: '
              f'{math.degrees(s.closed_peak):.2f} deg down from anchor; '
              f'lowest J7 encoder: {math.degrees(app.hybrid_anchor[6]-s.closed_peak):.2f} deg; '
              f'commanded: {app.continuous_strike_degrees} deg', flush=True)
    if getattr(app, 'hybrid_stopping', False):
        if (s.request_id >= app.hybrid_stop_request and not s.swing and s.ready
                and s.sample is not None and now-s.sample.at <= .020):
            close(app)
            app.hybrid_restoring = True
            app.hybrid_csp_at = app.bus.restore_right_joint7_csp(float(app.hybrid_anchor[6]))
            app.hybrid_restore_deadline = time.monotonic()+2.
            app.hybrid_last_enable = time.monotonic()
            app.phase = RESTORING
            app.status.set('Hybrid returned and settled; confirming position mode before centering')
        return
    if app.strike_active:
        sync_search(app, s)
        if s.count < app.hybrid_expected_count:
            return
        if s.phase in ('catch', 'withdrawal', 'settling'):
            app.phase = 'STRIKE RETURNING TO CYMBAL POSE'
        if s.completed > app.hybrid_seen_completed and s.returned_at is not None:
            app.hybrid_seen_completed = s.completed
            app.phase = 'STRIKE RETURNING TO CYMBAL POSE'
            print(f'HYBRID search −{app._current_strike_degrees()}°: '
                  f'measured peak {math.degrees(s.peak_drop):.3f}°; returned and settled', flush=True)
            app.complete_stage(s.returned_at)
    elif app.continuous_strike_active:
        app.continuous_strike_count = s.count
        app.phase = ('100 BPM STRIKE WAITING' if s.phase == 'hold' else
                     '100 BPM STRIKE MOVING OUT' if s.phase in ('drive', 'coast') else
                     '100 BPM STRIKE RETURNING')
        if s.main_count > app.hybrid_main_count:
            if s.main_count != app.hybrid_main_count+1 or now-s.main_at > .080:
                request_stop(app, 'Hi-hat deadline missed; refusing a catch-up burst')
                return
            app.hybrid_main_count = s.main_count
            app.continuous_hihat_pending = True
            app.continuous_current_event_at = s.main_at
            app.continuous_current_swing_label = s.label
            app._send_pending_hihat(now)
        if s.bottoms > app.hybrid_seen_bottoms:
            app.hybrid_seen_bottoms = s.bottoms
            print(f'HYBRID swing {s.count} ({s.label}): target {app.continuous_strike_degrees}°; '
                  f'arrival error {s.last_lateness*1000:+.1f} ms; '
                  f'partial returns {s.partial_returns}', flush=True)
