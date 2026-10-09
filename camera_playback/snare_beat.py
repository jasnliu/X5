"""One or two spaced, random eighth-note snares per bar; no hardware at import.

The preflighted reference is snare.sh's actual build_method(11 degrees).
A separate 500 Hz process owns J6 from centered setup through centered stop.
Recording workers pass J6 targets through shared memory, never CSP on the wire.
"""
import copy
import errno
from itertools import combinations
from dataclasses import dataclass, replace
import math
import multiprocessing as mp
import os
import pickle
import queue
import random
import signal
import socket
import time

from .mit_center_return import MitCenterReturn
from .mit_strike import Command, Joint7Session, Joint7Worker, Status, StrikeSettings
from .tempo import parse_bpm
from snare_lab.motion import build_method, stroke_timeout
from strike_lab.engine import StableWindow, limit_command

SNARE_DEGREES = 11.
# Full unchanged return + settling must fit between chosen notes, even at a bar boundary.
# Also allow the opening pickup to establish beat 1 before descent starts.
MAX_SNARE_BPM = 120.
MAX_RELEASE_LATENESS = .020


def minimum_hit_spacing(method, rules):
    """Complete descent/return, measured settling, and the existing 30 ms margin."""
    return method.curve.duration + rules.settle_seconds + .030


def validate_tempo(bpm, method, rules):
    bpm = parse_bpm(bpm)
    period = 60. / bpm
    minimum = max(minimum_hit_spacing(method, rules),
                  3 * (method.curve.down + .010))
    if bpm > MAX_SNARE_BPM or period < minimum:
        raise ValueError('Snare beat supports 20–120 BPM with its full unchanged return')
    return period


class MeasureGrid:
    """Eight straight eighth notes per four-quarter-note bar, independent of ride extras.

    Choose 1 or 2 with equal probability, then uniformly choose a feasible
    combination. Never shorten the stroke, shift notes late, or omit a bar.
    """
    def __init__(self, epoch, period, rng=None, *, minimum_spacing=0.):
        if not all(math.isfinite(x) for x in (epoch, period)) or period <= 0:
            raise ValueError('Invalid snare beat clock')
        if not math.isfinite(minimum_spacing) or minimum_spacing < 0:
            raise ValueError('Invalid snare minimum spacing')
        self.epoch, self.period = epoch, period
        self.rng = rng or random.Random()
        self.minimum_spacing = minimum_spacing
        self.last_at = None
        self.measure = 0
        self._choose_measure()

    def _time(self, note):
        return self.epoch + (4*self.measure + (note-1)/2)*self.period

    def _choose_measure(self):
        count = self.rng.randrange(2) + 1
        choices = [notes for notes in combinations(range(1, 9), count)
                   if (self.last_at is None or
                       self._time(notes[0])-self.last_at >= self.minimum_spacing-1e-9)
                   and all((b-a)*self.period/2 >= self.minimum_spacing-1e-9
                           for a, b in zip(notes, notes[1:]))]
        if not choices:
            raise ValueError('No snare pattern fits the full return at this tempo')
        self.notes = self.rng.choice(choices)
        self.note_index = 0

    @property
    def beat(self):
        """One-based eighth-note position (1..8), retained in the status field."""
        return self.notes[self.note_index]

    @property
    def at(self):
        return self._time(self.beat)

    def advance(self):
        self.last_at = self.at
        self.note_index += 1
        if self.note_index == len(self.notes):
            self.measure += 1
            self._choose_measure()


@dataclass(frozen=True)
class SnareStatus(Status):
    at: float = 0.
    armed: bool = False
    scheduled: bool = False
    moving: bool = False
    measure: int = 0
    beat: int = 0
    released_at: float | None = None
    scheduled_at: float | None = None
    queue_retries: int = 0


class SnareController:
    """Pure controller; the powered stroke is copied, never reimplemented."""
    def __init__(self, template, rules, limits, lower, upper, initial, now, rng=None):
        self.template, self.rules, self.limits = template, rules, limits
        self.anchor, self.lower, self.upper = template.anchor, lower, upper
        self.path = MitCenterReturn(initial, initial, 0., now)
        self.window = StableWindow(self.anchor, rules)
        self.method = self.grid = None
        self.rng = rng
        self.last_send = None
        self.last_torque = 0.
        self.status = SnareStatus(armed=True)

    def start(self, epoch, period):
        validate_tempo(60./period, self.template, self.rules)
        if self.grid is not None or self.method is not None:
            raise RuntimeError('Snare sequence already active')
        self.grid = MeasureGrid(epoch, period, self.rng,
                                minimum_spacing=minimum_hit_spacing(self.template, self.rules))

    def finish(self):
        # Cancel FUTURE hits immediately; finish the current full return.
        self.grid = None
        self.status = replace(self.status, scheduled=False)

    def update(self, sample, now, displayed_goal):
        moving = self.method is not None
        l = self.limits
        if (sample is None or not all(math.isfinite(v) for v in
                (sample.position, sample.velocity, sample.torque, sample.at))
                or sample.state != 2 or not -.001 <= now-sample.at <= (l.feedback_timeout if moving else .25)):
            raise RuntimeError('Snare J6 feedback stale, invalid, or not running')
        if not self.lower <= sample.position <= self.upper:
            raise RuntimeError('Snare J6 outside joint limits')
        if abs(sample.velocity) > l.velocity*1.25:
            raise RuntimeError('Snare J6 excessive measured velocity')
        if moving and self.last_send is not None and now-self.last_send > l.control_timeout:
            raise RuntimeError('Snare control deadline missed')
        if not math.isfinite(displayed_goal) or not self.lower <= -displayed_goal <= self.upper:
            raise RuntimeError('Invalid left J6 path target')
        self.window.add(sample)
        grid = self.grid
        if grid is not None:
            if abs(-displayed_goal-self.anchor) > 1e-6:
                raise RuntimeError('Left path moved while snare beat was active')
            release_at = grid.at-self.template.curve.down
            if now >= release_at:
                if now-release_at > MAX_RELEASE_LATENESS:
                    raise RuntimeError('Snare missed beat deadline; no catch-up hit')
                if self.method is not None or not self.window.ready:
                    raise RuntimeError('Snare not returned and settled before next beat')
                if now-sample.at <= min(.006, self.rules.maximum_feedback_gap/2):
                    self.method = copy.copy(self.template)
                    self.method.bias = self.path.bias
                    self.method.start(now, list(self.window.samples))
                    self.window = StableWindow(self.anchor, self.rules)
                    self.last_torque = self.path.bias
                    self.last_send = None
                    self.status = replace(self.status, measure=grid.measure+1, beat=grid.beat,
                        released_at=now, scheduled_at=grid.at, peak_drop=0.)
                    grid.advance()
        method = self.method
        if method is None:
            self.path.goal = -displayed_goal
            command = self.path.update(sample.position, now)
            phase = 'powered path/hold'
        else:
            depth = math.degrees(self.anchor-sample.position)
            if not -l.upper_excursion_deg <= depth <= min(14., l.hard_depth_deg):
                raise RuntimeError('Snare excursion outside validated strike corridor')
            self.status = replace(self.status, peak_drop=max(self.status.peak_drop, self.anchor-sample.position))
            command = method.update(sample, now)
            phase = method.phase
            if method.done:
                self.window.add(sample)
                command = method.hold()
            command, self.last_torque, _ = limit_command(command, sample, now,
                self.anchor, l, self.last_send, self.last_torque)
            if method.done and self.window.ready:
                self.method = None
                self.path = MitCenterReturn(sample.position, self.anchor, method.bias, now)
                self.status = replace(self.status, completed=self.status.completed+1)
            elif now-method.started > stroke_timeout(method, l):
                raise RuntimeError('Snare return failed to settle')
        if not self.lower <= command.position <= self.upper:
            raise RuntimeError('Snare command outside joint limits')
        self.last_send = now
        self.status = replace(self.status, at=now, sample=sample, phase=phase,
            moving=self.method is not None, scheduled=self.grid is not None,
            ready=self.method is None and self.window.ready)
        return command


def _run(interface, template, rules, limits, lower, upper, goal, clock, stop, heartbeat, sender):
    # Only parent owns Ctrl-C / emergency; child must not die mid-handoff.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    from snare_lab.runner import _LeftJoint6Channel
    channel = None
    controller = None
    status = SnareStatus()
    last_publish = 0.
    def publish(s, force=False):
        nonlocal last_publish
        now = time.monotonic()
        if force or now-last_publish >= .005:
            try:
                sender.send(pickle.dumps(s))
            except BlockingIOError:
                pass
            last_publish = now
    try:
        channel = _LeftJoint6Channel(interface)
        # Parent freezes ALL left targets at independently verified center
        # until armed status. This is the ONLY disable/mode setup in the run.
        helper = Joint7Worker(interface, -goal.value, lower, upper, StrikeSettings(),
            queue.Queue(), stop, lambda s: publish(replace(status, at=time.monotonic(), sample=s.sample)),
            motor=6, sign=-1.)
        helper._prepare(channel)
        now = time.monotonic()
        controller = SnareController(template, rules, limits, lower, upper, channel.sample.position, now)
        controller.path.bias = helper.controller.bias
        started = False
        deadline = now
        queue_blocked_at = None
        commands_sent = queue_retries = 0
        max_gap = 0.
        while not stop.is_set():
            channel.receive()
            now = time.monotonic()
            if now < deadline:
                channel.wait(deadline-now)
                continue
            if now-heartbeat.value > 2.:
                raise RuntimeError('Snare parent heartbeat lost; no further strikes')
            epoch, period, enabled = clock[:]
            if not enabled:
                controller.finish()
                started = False
            elif epoch > 0 and not started:
                controller.start(epoch, period)
                started = True
            previous_send, previous_torque = controller.last_send, controller.last_torque
            command = controller.update(channel.sample, now, goal.value)
            if controller.method is not None and time.monotonic()-channel.sample.at > limits.feedback_timeout:
                raise RuntimeError('Snare feedback expired before send')
            if stop.is_set():
                break
            try:
                channel.send(command.frame(6, -1.))
            except OSError as exc:
                if exc.errno not in (errno.ENOBUFS, errno.EAGAIN):
                    raise
                # Recording targets + state queries can briefly fill can1's
                # ten-frame TX queue. A refused write was NOT sent. Do not
                # queue an old strike command behind a sleep: receive fresh
                # feedback and recompute the current trajectory next turn.
                # Preserve the successful-send clock and torque slew history.
                controller.last_send, controller.last_torque = previous_send, previous_torque
                queue_retries += 1
                if queue_blocked_at is None:
                    queue_blocked_at = now
                if time.monotonic()-queue_blocked_at >= limits.control_timeout:
                    raise RuntimeError('Snare CAN transmit queue blocked beyond control deadline') from exc
                time.sleep(.00025)
                deadline = time.monotonic()
                continue
            sent_at = time.monotonic()
            gap = 0. if previous_send is None else sent_at-previous_send
            max_gap = max(max_gap, gap)
            controller.last_send = sent_at
            queue_blocked_at = None
            commands_sent += 1
            controller.status = replace(controller.status, commands_sent=commands_sent,
                last_gap=gap, max_gap=max_gap, feedback_age=sent_at-channel.sample.at,
                queue_retries=queue_retries)
            publish(controller.status)
            period_s = 1/limits.hz
            deadline = max(deadline+period_s, time.monotonic()+period_s if deadline+period_s <= now else 0.)
    except InterruptedError:
        pass
    except Exception as exc:
        status = status if controller is None else controller.status
        publish(replace(status, error=str(exc), ready=False, phase='fault', at=time.monotonic()), True)
        if channel is not None and channel.sample is not None and not stop.is_set():
            try:
                q = channel.sample.position
                if lower <= q <= upper:
                    channel.send(Command(q, 0., 40., 1.8).frame(6, -1.))
            except Exception:
                pass  # Never disable or re-enable an off-center faulted arm.
    finally:
        if channel is not None:
            channel.close()
        sender.close()


class SnareSession(Joint7Session):
    def __init__(self, bus, template, rules, limits, lower, upper):
        if os.environ.get('STRIKE_LAB_OFFLINE_ONLY') == '1':
            raise RuntimeError('Physical snare transport forbidden in offline tests')
        bus.snare_center_evidence()  # Strict .2 deg / .12 span / .6 s, BEFORE any mode change.
        if getattr(bus, 'left_joint6_session', None) is not None:
            raise RuntimeError('Left J6 already owned')
        self.bus, self.session_attr = bus, 'left_joint6_session'
        self.closed = False
        self.template, self.rules = template, rules
        ctx = mp.get_context('spawn')
        self.goal = ctx.Value('d', bus.states['left', 6][0])
        self.clock = ctx.Array('d', [0., 0., 0.])
        self.heartbeat = ctx.Value('d', time.monotonic())
        self.stop_event = ctx.Event()
        self.requests = ctx.Queue(maxsize=1)  # Joined lifecycle inherited from Joint7Session.
        self.receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.receiver.setblocking(False)
        sender.setblocking(False)
        self._status = SnareStatus()
        self.process = ctx.Process(target=_run, name='left-j6-snare-beat', args=(
            bus.sockets['left'].getsockname()[0], template, rules, limits, lower, upper,
            self.goal, self.clock, self.stop_event, self.heartbeat, sender))
        bus.left_joint6_session = self
        bus.left_drive.left_joint6_session = self
        bus.left_monitor = None
        previous = {k: os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS')}
        try:
            os.environ.update({k: '1' for k in previous})
            self.process.start()
        except Exception:
            bus.left_joint6_session = bus.left_drive.left_joint6_session = None
            self.receiver.close()
            self.requests.close()
            raise
        finally:
            sender.close()
            for k, v in previous.items():
                if v is None: os.environ.pop(k, None)
                else: os.environ[k] = v

    @property
    def status(self):
        self.heartbeat.value = time.monotonic()
        return super().status

    def start_swing(self, bpm):
        s = self.status
        if s.error or not s.ready or time.monotonic()-s.at > .1:
            raise RuntimeError('Left snare must be settled at its recording endpoint')
        period = validate_tempo(bpm, self.template, self.rules)
        self.clock[:] = [0., period, 1.]  # Right worker publishes actual pickup-derived epoch.

    def finish(self):
        with self.clock.get_lock():
            self.clock[2] = 0.

    @property
    def finished(self):
        s = self.status
        return not s.scheduled and not s.moving and not s.error and s.armed

    def stop(self):
        super().stop()
        if getattr(self.bus.left_drive, 'left_joint6_session', None) is self:
            self.bus.left_drive.left_joint6_session = None


class SnareReferenceSimulation:
    """Ideal reference-only visualization for --test; NEVER a hardware facade."""
    def __init__(self, bus, template, rules):
        self.bus, self.template, self.rules = bus, template, rules
        self.grid = self.method = None
        self.waiting = False
        self.status = SnareStatus(armed=True, ready=True, at=time.monotonic())

    def start_swing(self, bpm):
        self.period = validate_tempo(bpm, self.template, self.rules)
        self.waiting = True

    def set_epoch(self, epoch):
        if self.waiting:
            self.grid = MeasureGrid(epoch, self.period,
                                    minimum_spacing=minimum_hit_spacing(self.template, self.rules))
            self.waiting = False

    def update_pose(self, now):
        if self.grid is not None and now >= self.grid.at-self.template.curve.down:
            if self.method is not None:
                raise RuntimeError('Simulated snare strokes overlap')
            self.method = copy.copy(self.template)
            self.method.start(self.grid.at-self.template.curve.down, [])
            self.status = replace(self.status, measure=self.grid.measure+1, beat=self.grid.beat,
                released_at=self.method.started, scheduled_at=self.grid.at)
            self.grid.advance()
        if self.method is not None:
            elapsed = now-self.method.started
            self.bus._left_joints[5] = -(self.template.anchor-self.method.curve.at(elapsed)[0])
            if elapsed >= self.method.curve.duration+self.rules.settle_seconds:
                self.method = None
                self.status = replace(self.status, completed=self.status.completed+1,
                                      peak_drop=self.template.target)
        self.status = replace(self.status, at=now, scheduled=self.grid is not None or self.waiting,
                              moving=self.method is not None, ready=self.method is None)

    def finish(self):
        self.grid = None
        self.waiting = False
        self.status = replace(self.status, scheduled=False)

    @property
    def finished(self):
        return not self.status.moving and not self.status.scheduled

    def stop(self):
        self.finish()
        self.method = None
        self.bus.snare_simulation = None
