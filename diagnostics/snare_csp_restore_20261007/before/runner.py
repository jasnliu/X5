"""Left-arm (J6) snare-strike sequence.

center -> gripper close -> play left_recordings/record1.json -> single
strike (strike_lab method 'powered' -- see snare_lab/config.py for why, not
'hybrid') -> reverse the recording -> center -> relax.

Both --simulate and --hardware share the geometry/recording/validation
helpers below; only the actual motor I/O differs. The physical strike
drives strike_lab's generic Method/Definition interface directly, in this
same process -- no dedicated subprocess, matching exactly how
strike_lab.backends.HardwareBackend + strike_lab.engine.Engine already run
non-hybrid methods for the right arm's experiment.sh (hybrid's dedicated
multiprocess HybridSession exists for its own GUI-isolation reasons, not a
requirement of the method framework itself). Only the MIT-mode CAN I/O
(camera_playback.mit_strike.Joint7Channel/Joint7Worker) is reused, already
generalized by side/motor/sign -- see that module's docstrings. Everything
BEFORE and AFTER the strike (centering, gripper, recording playback,
reverse, center, relax) uses the same plain CSP position control the left
arm already uses in production (centering.motors.Motors,
camera_playback.left_hold, camera_playback.left_return), unchanged.

No RViz/robot_state_publisher visualization is wired up yet (snare.sh always
runs headless); this module only prints phase transitions to stdout.
"""
from dataclasses import replace
import json
import math
from pathlib import Path
import queue
import threading
import time

import numpy as np

from centering.motors import Motors, SPEED, CURRENT, GRIPPER_CURRENT, MOTOR_RUNNING_STATE
from safe_zone.geometry import Model, Zone, LEFT_TCP
from smooth_playback.trajectory import Geometry
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET, LeftHoldDrive
from camera_playback.left_return import ReverseRecording
from camera_playback.mit_strike import Joint7Channel, Joint7Worker, StrikeSettings
from camera_playback.trajectory import PlaybackFollower
from camera_playback.recording_cache import load_cached_playback_trajectory
from strike_lab.backends import SimBackend
from strike_lab.config import Plant, Rules
from strike_lab.engine import StableWindow, limit_command
from strike_lab.methods import METHODS
from .config import (ROOT, STRIKE_MOTOR, STRIKE_SIGN, STRIKE_METHOD_ID,
                     LEFT_POWERED_OVERRIDES,
                     LEFT_RETURN_TOLERANCE_DEG, LEFT_SETTLE_RANGE_DEG,
                     validate_joint_depth, joint_rotation_limits)
from .motion import build_method, stroke_timeout

STRIKE_INDEX = STRIKE_MOTOR - 1  # 0-indexed position within a 7-joint vector.
GRIPPER_TOLERANCE = math.radians(1.)
POSE_TOLERANCE = math.radians(.20)
SETTLE_SECONDS = .6
TUNING_PATH = ROOT / 'config/experiment_hardware_tuned.json'


class _LeftJoint6Channel(Joint7Channel):
    """Binds motor/sign so this channel filters/decodes/encodes J6 CAN
    frames instead of Joint7Channel's default (right arm's J7). A plain
    module-level class, not a lambda/closure, since it is still used as a
    channel_factory passed to Joint7Worker (kept picklable on principle,
    even though the strike itself no longer spawns a subprocess)."""
    def __init__(self, interface):
        super().__init__(interface, STRIKE_MOTOR, STRIKE_SIGN)


def log(message):
    print(message, flush=True)


def notify_fault(body):
    """Push an ntfy alert when the left arm needs manual/physical
    intervention (fault recovery itself failed, or relax after a stall
    failed) -- the operator may not be watching this terminal. Best-effort:
    a delivery failure here must never mask or replace the original fault,
    only ever logged."""
    try:
        import importlib.util
        path = Path.home() / '.local/bin/claude-ntfy.py'
        spec = importlib.util.spec_from_file_location('claude_ntfy', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = module.post_ntfy('snare.sh: manual intervention required', body, 'warning,robot')
        module.play_sound()
        log(f'LEFT: ntfy alert sent: {result}')
    except Exception as exc:
        log(f'LEFT: ntfy alert FAILED to send: {exc}')


# ---------------------------------------------------------------- geometry --

def load_geometry():
    model = Model(ROOT / 'model/openarmx.urdf')
    zone = Zone.load(ROOT / 'left_zones/zone1.json', model.digest, LEFT_TCP)
    import xml.etree.ElementTree as ET
    root = ET.parse(ROOT / 'model/openarmx.urdf').getroot()
    limits = [root.find(f"joint[@name='openarmx_left_joint{i}']/limit") for i in range(1, 8)]
    lower = np.array([float(e.get('lower')) for e in limits])
    upper = np.array([float(e.get('upper')) for e in limits])
    return model, zone, lower, upper


def left_geometry(model, zone, lower, upper):
    """Mirrors camera_playback.dual_recording.left_geometry() exactly, but
    takes an already-loaded model/zone/limits instead of reloading them."""
    g = Geometry.__new__(Geometry)
    g.model, g.side = model, 'left'
    g.zone = zone
    g.lower, g.upper = lower, upper
    g.center = LEFT_CENTER.copy()
    return g


def load_recording(path, model, zone, lower, upper, progress=None):
    return load_cached_playback_trajectory(
        path, model, zone, lower, upper, LEFT_CENTER, SPEED,
        progress=progress, playback_speed=None, side='left')


def dipped_target(recording, degrees):
    """The planned strike target, in plain displayed-joint terms: the
    recording's endpoint with STRIKE_INDEX moved by `degrees` in the
    direction that actually strikes the snare (anchor + amount here;
    confirmed against record1.json's own endpoint -- the right arm's J7
    convention is anchor - amount, the opposite)."""
    target = recording.last_joints.copy()
    target[STRIKE_INDEX] += math.radians(degrees)
    return target


def validate_strike_path(geometry, recording, degrees):
    """Refuse up front if the planned dip would leave the validated left
    TCP envelope. Required because left_zones/zone1.json was built from the
    recorded path alone -- it has never seen this excursion."""
    geometry.check_line(recording.last_joints, dipped_target(recording, degrees), measured_start=True)


def strike_corridor(recording, limits, lower_limit, upper_limit):
    """Internal (sign-adjusted) anchor/lower/upper for the strike method,
    using the full positive J6 range and the existing bounded return overshoot.
    Both ends remain inside the left-J6 URDF limits."""
    anchor_displayed = float(recording.last_joints[STRIKE_INDEX])
    anchor = STRIKE_SIGN * anchor_displayed
    lower_internal, upper_internal = sorted((STRIKE_SIGN * lower_limit, STRIKE_SIGN * upper_limit))
    lower = lower_internal  # Positive displayed J6 strike ends at its URDF upper limit.
    upper = min(upper_internal, anchor + math.radians(limits.upper_excursion_deg))
    return anchor_displayed, anchor, lower, upper


def load_left_tuning(recording, lower_limit, upper_limit):
    """The right arm's tuned parameters for STRIKE_METHOD_ID, with
    LEFT_POWERED_OVERRIDES applied (zeroing the fixed gravity-compensation
    load_torque -- see snare_lab.config's docstring), the upper-excursion
    margin widened, and the post-strike settle tolerances widened for J6.
    The maximum strike depth comes from J6's remaining positive rotation.
    Method-agnostic (unlike camera_playback.hybrid_strike.load_tuning(),
    which only ever loaded the 'hybrid' block) since this no longer assumes
    any one method. Used by both --simulate and --hardware so what's
    simulated matches what's attempted."""
    config = json.loads(TUNING_PATH.read_text())
    definition = METHODS[STRIKE_METHOD_ID]
    base = config.get('parameters', {}).get(STRIKE_METHOD_ID, {})
    parameters = definition.parameters(dict(base, **LEFT_POWERED_OVERRIDES))
    rules = replace(Rules(**config.get('rules', {})),
                    return_tolerance_deg=LEFT_RETURN_TOLERANCE_DEG,
                    settle_range_deg=LEFT_SETTLE_RANGE_DEG)
    limits = joint_rotation_limits(float(recording.last_joints[STRIKE_INDEX]),
                                   lower_limit, upper_limit, config.get('limits', {}))
    return parameters, rules, limits


def prepare_strike(geometry, recording, degrees):
    """All depth, path and reference checks complete before opening motor I/O."""
    lower, upper = geometry.lower[STRIKE_INDEX], geometry.upper[STRIKE_INDEX]
    anchor = float(recording.last_joints[STRIKE_INDEX])
    validate_joint_depth(degrees, anchor, lower, upper)
    validate_strike_path(geometry, recording, degrees)
    parameters, rules, limits = load_left_tuning(recording, lower, upper)
    method = build_method(STRIKE_SIGN * anchor, 0., parameters, degrees, limits)
    log(f'LEFT: J6 maximum positive strike depth {limits.hard_depth_deg:.6f} deg '
        f'(joint upper limit {math.degrees(upper):.6f} deg)')
    log(f'LEFT: powered reference down {method.curve.down:.3f} s '
        f'(previous {method.curve.previous_fast_down:.3f} s), '
        f'return unchanged at {method.curve.up:.3f} s; dynamic limits unchanged')
    if method.curve.curved_acceleration:
        descent = method.curve.descent
        log(f'LEFT: curved acceleration reaches {math.degrees(descent.peak_speed):.1f} deg/s '
            f'at {100*descent.peak_fraction:.0f}% of downward travel; '
            'smooth braking into the existing return')
    else:
        log('LEFT: dynamic limits retain the previous descent; no curved speed increase available')
    return parameters, rules, limits


def check_strike_position(sample, lower, upper):
    if sample is None or not math.isfinite(sample.position) or not lower <= sample.position <= upper:
        raise RuntimeError('Left J6 measured position outside joint/return corridor or missing')


# -------------------------------------------------------------- simulation --

class _SignFlippedPlayback:
    """Presents `original` with STRIKE_INDEX sign-flipped, so SimBackend's
    generic "anchor - amount = struck" arithmetic produces the correct
    displayed-domain direction once read back out (see runner module
    docstring and camera_playback.mit_strike.Command.frame()). Hardware
    playback never needs this -- it drives plain CSP in displayed terms
    throughout; only the strike session's anchor/lower/upper are converted."""
    def __init__(self, original):
        self._o = original

    def _flip(self, joints):
        joints = list(joints)
        joints[STRIKE_INDEX] = STRIKE_SIGN * joints[STRIKE_INDEX]
        return joints

    @property
    def duration_s(self):
        return self._o.duration_s

    @property
    def first_joints(self):
        return self._flip(self._o.first_joints)

    @property
    def last_joints(self):
        return self._flip(self._o.last_joints)

    def joints_at(self, elapsed):
        q, done = self._o.joints_at(elapsed)
        return self._flip(q), done


def _displayed(positions):
    positions = list(positions)
    positions[STRIKE_INDEX] = STRIKE_SIGN * positions[STRIKE_INDEX]
    return positions


def run_simulate(args):
    realtime = not args.fast
    model, zone, lower, upper = load_geometry()
    geometry = left_geometry(model, zone, lower, upper)
    recording = load_recording(args.recording, model, zone, lower, upper,
                               progress=log)
    parameters, rules, limits = prepare_strike(geometry, recording, args.degrees)
    _, _, lo, hi = strike_corridor(recording, limits, lower[STRIKE_INDEX], upper[STRIKE_INDEX])
    # gravity=0: J6 isn't meaningfully gravity-loaded in this pose (see
    # snare_lab/config.py), so the synthetic plant shouldn't be either --
    # LEFT_POWERED_OVERRIDES already zeroed the matching load_torque belief.
    plant = Plant(inertia=parameters['inertia'], gravity=0., friction=parameters['friction'])
    backend = SimBackend(plant, realtime=realtime,
                         playback=_SignFlippedPlayback(recording),
                         rest_pose=LEFT_CENTER.tolist(), strike_index=STRIKE_INDEX)
    last_phase = None
    def publish(data):
        nonlocal last_phase
        if data.get('phase') != last_phase:
            last_phase = data['phase']
            log(f"[sim] {last_phase}")
    stop = threading.Event()
    anchor, bias = backend.prepare(stop, publish)
    method = build_method(anchor, bias, parameters, args.degrees, limits)

    window = StableWindow(anchor, rules)
    deadline = time.monotonic() + 3.
    while not window.ready:
        sample, now = backend.receive(), backend.now()
        check_strike_position(sample, lo, hi)
        window.add(sample)
        backend.send(method.hold())
        backend.advance(.002)
        if time.monotonic() > deadline:
            raise RuntimeError('Simulated anchor did not settle before strike')

    sample, now = backend.receive(), backend.now()
    method.start(now, list(window.samples))
    window = StableWindow(anchor, rules)
    last_send, last_torque = None, bias
    peak_drop = 0.
    returned = False
    last_phase = None
    deadline = time.monotonic() + stroke_timeout(method, limits)
    while True:
        sample, now = backend.receive(), backend.now()
        check_strike_position(sample, lo, hi)
        peak_drop = max(peak_drop, anchor - sample.position)
        command = method.update(sample, now)
        if method.phase != last_phase:
            last_phase = method.phase
            log(f"[sim] STRIKE {last_phase}")
        if method.done:
            if not returned:
                window = StableWindow(anchor, rules)
                returned = True
            window.add(sample)
            command = method.hold()
        bounded, last_torque, _ = limit_command(
            command, sample, now, anchor, limits, last_send, last_torque)
        if not lo <= bounded.position <= hi:
            raise RuntimeError('J6 commanded position outside validated corridor')
        backend.send(bounded)
        last_send = now
        backend.advance(.002)
        if returned and window.ready:
            break
        if time.monotonic() > deadline:
            if window.samples:
                errs = [math.degrees(s.position - anchor) for s in window.samples]
                log(f'[sim] STRIKE DIAGNOSTIC at timeout: returned={returned} '
                    f'window_len={len(window.samples)} '
                    f'error_deg_min={min(errs):.3f} max={max(errs):.3f} '
                    f'last={errs[-1]:.3f} tolerance_deg={rules.return_tolerance_deg:g}')
            raise RuntimeError('Simulated strike did not settle before timeout')
    log(f"[sim] strike complete: measured peak {math.degrees(peak_drop):.2f} deg "
        f"(commanded {args.degrees:g} deg)")

    # Reverse the recording, then center, in plain displayed terms -- pure
    # simulated interpolation, same fixed-step style as SimBackend.prepare().
    log('[sim] REVERSE RECORDING')
    reverse = ReverseRecording(recording, recording.duration_s)
    positions = _displayed(backend.positions)
    elapsed, step = 0., .02
    while True:
        q, done = reverse.joints_at(elapsed)
        positions = list(q)
        if done:
            break
        elapsed = min(reverse.duration_s, elapsed + step)
        if realtime:
            time.sleep(step)
    log('[sim] CENTERING')
    start = list(positions)
    steps = 40
    for i in range(1, steps + 1):
        positions = [a + (b - a) * i / steps for a, b in zip(start, LEFT_CENTER)]
        if realtime:
            time.sleep(.02)
    log('[sim] CENTERED')
    log('[sim] RELAXED (simulation; nothing was ever moved on real hardware)')
    return 0


# ----------------------------------------------------------------- hardware --

def _wait_fresh(bus, timeout=2.):
    deadline = time.monotonic() + timeout
    while not bus.fresh():
        if time.monotonic() > deadline:
            raise RuntimeError('Fresh left-arm encoders unavailable before setup')
        bus.poll()
        time.sleep(.01)


def _wait_pose_once(bus, goal, phase, timeout=15., gripper_target=None):
    deadline = time.monotonic() + timeout
    stable = None
    while True:
        bus.poll()
        if any(bus.states['left', i][1] != MOTOR_RUNNING_STATE for i in range(1, 9)):
            raise RuntimeError(f'Left motor stopped during {phase}')
        if (gripper_target is not None
                and abs(bus.states['left', 8][0] - gripper_target) > GRIPPER_TOLERANCE):
            raise RuntimeError(f'Left gripper did not stay closed during {phase}')
        actual = np.array([bus.states['left', i][0] for i in range(1, 8)])
        if np.max(np.abs(actual - goal)) < POSE_TOLERANCE:
            if stable is None:
                stable = time.monotonic()
            if time.monotonic() - stable >= SETTLE_SECONDS:
                return
        else:
            stable = None
        if time.monotonic() > deadline:
            raise RuntimeError(f'{phase}: position timeout')
        time.sleep(.01)


def _wait_pose(bus, goal, phase, timeout=15., gripper_target=None, attempts=3):
    """_wait_pose_once(), self-healing: a motor that transiently drops out
    (observed specifically right after restore_left_joint6_csp's disable/
    re-enable cycle, but handled generally here) is re-enabled at its
    current position and the wait resumes, rather than failing the whole
    sequence over a single brief dropout."""
    for attempt in range(attempts):
        try:
            return _wait_pose_once(bus, goal, phase, timeout=timeout, gripper_target=gripper_target)
        except RuntimeError as exc:
            if 'motor stopped' not in str(exc).lower() or attempt == attempts-1:
                raise
            log(f'LEFT: {exc}; re-enabling and retrying ({attempts-1-attempt} attempts left)')
            bus.poll()
            for motor in range(1, 8):
                if bus.states['left', motor][1] != MOTOR_RUNNING_STATE:
                    _ensure_running(bus, motor, CURRENT[motor - 1], f'J{motor}')
            if bus.states['left', 8][1] != MOTOR_RUNNING_STATE:
                _ensure_running(bus, 8, GRIPPER_CURRENT, 'gripper')
            bus.set_positions(goal)


def _play(bus, recording_like, phase, gripper_target):
    """Drive CSP joints 1-7 along recording_like.joints_at(t), gripper held
    closed throughout. Shared by the forward recording and its reverse."""
    bus.set_positions(recording_like.first_joints)
    _wait_pose(bus, recording_like.first_joints, f'{phase}: moving to start',
              gripper_target=gripper_target)
    follower = PlaybackFollower()
    started = time.monotonic()
    while True:
        bus.poll()
        if any(bus.states['left', i][1] != MOTOR_RUNNING_STATE for i in range(1, 9)):
            raise RuntimeError(f'Left motor stopped during {phase}')
        if abs(bus.states['left', 8][0] - gripper_target) > GRIPPER_TOLERANCE:
            raise RuntimeError(f'Left gripper did not stay closed during {phase}')
        actual = np.array([bus.states['left', i][0] for i in range(1, 8)])
        desired, done = recording_like.joints_at(time.monotonic() - started)
        bus.set_positions(follower.update(actual, desired, time.monotonic()))
        if done:
            break
        time.sleep(.02)
    bus.set_positions(recording_like.last_joints)
    _wait_pose(bus, recording_like.last_joints, f'{phase}: holding endpoint',
              gripper_target=gripper_target)


def _is_stall(exc):
    """A 'STALL:' fault means the joint is powered but not tracking its
    target -- i.e. already stationary. Forcing more motion into a stuck
    joint is what's actually unsafe there, so the right response is to
    relax directly, not to keep commanding it toward center. Every other
    fault leaves the arm somewhere it was not meant to stop (mid-recording,
    mid-strike, mid-reverse): relaxing immediately there is what's
    dangerous (uncontrolled drop/swing), so those recover to the custom
    center position first."""
    return 'stall' in str(exc).lower()


def _ensure_running(bus, motor, current_limit, label):
    """Re-enable one left motor via the same confirmed-enable routine
    center()/_play() use, at its own current measured position. Used both
    to bring a motor back from a disabled state and, during recovery, to
    heal a motor that drops out again mid-move."""
    bus.poll()
    position = bus.states['left', motor][0]
    log(f'LEFT FAULT RECOVERY: re-enabling {label} at its current position '
        f'({math.degrees(position):+.2f} deg)')
    deadline = time.monotonic() + 5.
    for _ in bus._prepare_center_motor_confirmed(motor, position, current_limit):
        bus.poll()
        if time.monotonic() > deadline:
            raise RuntimeError(f'{label} re-enable confirmation timed out')
        time.sleep(.005)


def _restore_strike_joint_csp(bus, target_position):
    """restore_left_joint6_csp(), confirmed -- it disables/re-enables J6 but
    does not itself wait for the re-enable to be confirmed running (mirrors
    the right arm's restore_right_joint7_csp), and empirically this can
    leave J6 disabled a moment later. Used after every real strike attempt
    (successful or not), not just during fault recovery -- the same gap
    bites both paths identically."""
    try:
        bus.restore_left_joint6_csp(target_position)
        log('LEFT: J6 CSP restore sent')
    except Exception as exc:
        log(f'LEFT: J6 CSP restore raised: {exc}')
    deadline = time.monotonic() + 1.
    while bus.states['left', STRIKE_MOTOR][1] != MOTOR_RUNNING_STATE:
        bus.poll()
        if time.monotonic() > deadline:
            break
        time.sleep(.01)
    if bus.states['left', STRIKE_MOTOR][1] != MOTOR_RUNNING_STATE:
        _ensure_running(bus, STRIKE_MOTOR, CURRENT[STRIKE_INDEX], 'J6')


def _recover_to_center_and_relax(bus):
    """Best-effort: release any stuck MIT ownership, restore ordinary CSP
    on the strike joint, drive the whole arm back to LEFT_CENTER under
    control -- self-healing any motor that drops out again along the way --
    then relax. Returns True only if centered-then-relaxed succeeded; on any
    failure the arm is deliberately left powered and NOT relaxed (never
    silently relax from an unconfirmed position)."""
    try:
        if not getattr(bus, 'active', False):
            log('LEFT FAULT RECOVERY: arm was never activated; nothing to center')
            return True
        bus.poll()
        if not bus.fresh():
            raise RuntimeError('Fresh feedback unavailable; cannot safely recover position')
        session = getattr(bus, 'left_joint6_session', None)
        if session is not None:
            try:
                session.stop()
                log('LEFT FAULT RECOVERY: stopped the stuck MIT session')
            except Exception as exc:
                log(f'LEFT FAULT RECOVERY: stopping MIT session failed: {exc}')
        bus.poll()
        if bus.states['left', STRIKE_MOTOR][1] == MOTOR_RUNNING_STATE:
            # Already running (e.g. fault struck before MIT was entered at
            # all): restore_left_joint6_csp's disable/re-enable cycle is
            # unnecessary churn here -- do nothing, let the self-healing
            # centering loop below handle anything that does drop out.
            log('LEFT FAULT RECOVERY: J6 already running; skipping CSP restore')
        else:
            _restore_strike_joint_csp(bus, bus.states['left', STRIKE_MOTOR][0])

        log('LEFT FAULT RECOVERY: driving to custom center position')
        bus.set_positions(LEFT_CENTER)
        _wait_pose(bus, LEFT_CENTER, 'fault recovery centering', timeout=20., attempts=5)
        bus.relax()
        log('LEFT FAULT RECOVERY: reached custom center and relaxed safely')
        return True
    except Exception as exc:
        log(f'LEFT FAULT RECOVERY FAILED: {exc}')
        log('LEFT FAULT RECOVERY: arm is NOT relaxed; it remains powered wherever '
            'it stopped; physical/manual intervention required')
        notify_fault(f'Left arm fault recovery FAILED ({exc}). It is NOT relaxed and '
                    'remains powered wherever it stopped. Turn it off manually.')
        return False


def _strike_hardware(bus, anchor, lower, upper, parameters, rules, limits, degrees):
    """Drive STRIKE_METHOD_ID directly over J6's own raw CAN channel, in
    this same process -- see module docstring for why no subprocess is
    needed. Mirrors strike_lab.backends.HardwareBackend's receive/advance
    split and strike_lab.engine.Engine.trial()'s settle-then-strike-then-
    settle loop, generalized by side/motor/sign. Returns the measured peak
    depth in degrees."""
    interface = bus.sockets['left'].getsockname()[0]
    channel = _LeftJoint6Channel(interface)
    last_bus_poll = time.monotonic()

    def receive():
        nonlocal last_bus_poll
        channel.receive()
        now = time.monotonic()
        if now - last_bus_poll >= .02:
            # Rate-limited like HardwareBackend.receive()'s own bus poll:
            # keeps the parent's other 15 motors fresh for fault recovery
            # without adding overhead to this tight control loop.
            bus.poll()
            last_bus_poll = now
        return channel.sample

    def advance(seconds):
        deadline = time.monotonic() + max(0., seconds)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            channel.wait(remaining)
            channel.receive()

    try:
        helper = Joint7Worker(interface, anchor, lower, upper, StrikeSettings(),
                              queue.Queue(), threading.Event(), lambda s: None,
                              motor=STRIKE_MOTOR, sign=STRIKE_SIGN)
        helper._prepare(channel)
        bias = helper.controller.bias

        method = build_method(anchor, bias, parameters, degrees, limits)

        window = StableWindow(anchor, rules)
        deadline = time.monotonic() + 3.
        while not window.ready:
            sample = receive()
            if sample is not None:
                check_strike_position(sample, lower, upper)
                window.add(sample)
                channel.send(method.hold().frame(STRIKE_MOTOR, STRIKE_SIGN))
            advance(1. / limits.hz)
            if time.monotonic() > deadline:
                raise RuntimeError('J6 anchor did not settle before strike')

        now = time.monotonic()
        method.start(now, list(window.samples))
        window = StableWindow(anchor, rules)
        last_send, last_torque = None, bias
        peak_drop = 0.
        returned = False
        last_phase = None
        start = time.monotonic()
        while True:
            sample = receive()
            now = time.monotonic()
            if sample is None:
                raise RuntimeError('J6 feedback lost during strike')
            check_strike_position(sample, lower, upper)
            if sample.state != 2:
                raise RuntimeError('J6 not running during strike')
            peak_drop = max(peak_drop, anchor - sample.position)
            command = method.update(sample, now)
            if method.phase != last_phase:
                last_phase = method.phase
                log(f'LEFT: STRIKE {last_phase}')
            if method.done:
                if not returned:
                    window = StableWindow(anchor, rules)
                    returned = True
                window.add(sample)
                command = method.hold()
            bounded, last_torque, _ = limit_command(
                command, sample, now, anchor, limits, last_send, last_torque)
            if not lower <= bounded.position <= upper:
                raise RuntimeError('J6 commanded position outside validated corridor')
            channel.send(bounded.frame(STRIKE_MOTOR, STRIKE_SIGN))
            last_send = now
            if returned and window.ready:
                break
            if now - start > stroke_timeout(method, limits):
                if window.samples:
                    errs = [math.degrees(s.position - anchor) for s in window.samples]
                    log(f'LEFT: STRIKE DIAGNOSTIC at timeout: returned={returned} '
                        f'window_len={len(window.samples)} '
                        f'error_deg_min={min(errs):.3f} max={max(errs):.3f} '
                        f'last={errs[-1]:.3f} tolerance_deg={rules.return_tolerance_deg:g}')
                raise RuntimeError('Left strike did not settle before timeout')
            advance(max(0., 1. / limits.hz - (time.monotonic() - now)))
        return math.degrees(peak_drop)
    finally:
        channel.close()


def run_hardware(args):
    """Implemented and regression-tested at the unit level (see
    tests/test_left_hybrid_strike.py). On any fault other than a motor
    stall, recovers to the custom center position before relaxing (see
    _recover_to_center_and_relax); a stall relaxes directly, since the
    joint is already stationary and forcing it further is the unsafe path."""
    model, zone, lower, upper = load_geometry()
    geometry = left_geometry(model, zone, lower, upper)
    recording = load_recording(args.recording, model, zone, lower, upper, progress=log)
    parameters, rules, limits = prepare_strike(geometry, recording, args.degrees)

    bus = Motors.__new__(LeftHoldDrive)
    Motors.__init__(bus, 'left', False)
    bus.control_gripper = True  # Enables the gripper setup-frame allowlist in
                               # allowed(); the closed-target frame itself is
                               # permitted unconditionally by LeftHoldDrive.
    try:
        try:
            _wait_fresh(bus)
            if any(v[1] != 0 for v in bus.states.values()):
                raise RuntimeError('Both arms and grippers must be relaxed before centering')
            log('LEFT: centering (J1-J7)')
            for _ in bus.center(LEFT_CENTER.tolist(), None, confirm_enabled=True):
                bus.poll()
                time.sleep(.002)
            log('LEFT: closing gripper')
            for _ in bus._prepare_center_motor_confirmed(8, LEFT_GRIPPER_TARGET, GRIPPER_CURRENT):
                bus.poll()
                time.sleep(.002)
            bus.set_positions(LEFT_CENTER)
            _wait_pose(bus, LEFT_CENTER, 'centering', gripper_target=LEFT_GRIPPER_TARGET)

            log('LEFT: playing left_recordings/record1.json')
            _play(bus, recording, 'recording playback', LEFT_GRIPPER_TARGET)
            bus.poll()
            measured_j6 = bus.states['left', STRIKE_MOTOR][0]
            log(f'LEFT: DIAGNOSTIC measured J6 after playback: {math.degrees(measured_j6):.3f} deg '
                f'(recording endpoint expects {math.degrees(recording.last_joints[STRIKE_INDEX]):.3f} deg)')

            anchor_displayed, anchor, lo, hi = strike_corridor(
                recording, limits, lower[STRIKE_INDEX], upper[STRIKE_INDEX])
            log(f'LEFT: {STRIKE_METHOD_ID} strike, {args.degrees:g} deg '
                f'(J6 anchor {math.degrees(anchor_displayed):.2f} deg)')
            peak_deg = _strike_hardware(bus, anchor, lo, hi, parameters, rules, limits, args.degrees)
            log(f'LEFT: strike complete, measured peak {peak_deg:.2f} deg')
            _restore_strike_joint_csp(bus, anchor_displayed)

            log('LEFT: reversing recording')
            reverse = ReverseRecording(recording, recording.duration_s)
            _play(bus, reverse, 'reverse recording', LEFT_GRIPPER_TARGET)

            log('LEFT: centering')
            bus.set_positions(LEFT_CENTER)
            _wait_pose(bus, LEFT_CENTER, 'final centering', gripper_target=LEFT_GRIPPER_TARGET)
            bus.relax()
            log('LEFT: relaxed')
        except Exception as exc:
            if _is_stall(exc):
                log(f'LEFT FAULT (motor stall): {exc}')
                log('LEFT FAULT: joint is stationary; relaxing directly, not forcing further motion')
                try:
                    bus.relax()
                    log('LEFT FAULT: relaxed after stall')
                except Exception as relax_exc:
                    log(f'LEFT FAULT: relax after stall also failed: {relax_exc}; '
                        'use physical power cutoff')
                    notify_fault(f'Left arm STALL fault, and relax also failed ({relax_exc}). '
                                'Use the physical power cutoff.')
                raise
            log(f'LEFT FAULT (non-stall): {exc}')
            log('LEFT FAULT: relaxing off-center is dangerous; attempting recovery to '
                'custom center first')
            _recover_to_center_and_relax(bus)
            raise
    finally:
        bus.close()
    return 0
