"""Hardware beat's hybrid controller. No hardware or file I/O at import time.

The method, tuned parameters, catch curve, settling window and torque limiter
are the experiment's actual implementations. Swing adds a release scheduler
and a smoothly braked partial rebound for short pairs; another stroke requires
4 degrees of encoder clearance above the selected low target.
The fixed camera anchor and selected absolute bottom never move between hits.
"""
from dataclasses import dataclass, replace
from collections import deque
import json
import math
import multiprocessing
import os
from pathlib import Path
import pickle
import queue
import socket
import time

from .mit_strike import (Command, Joint7Channel, Joint7Session, Joint7Worker,
                         Status, StrikeSettings)
from strike_lab.config import Limits, Rules, StrikeGoal
from strike_lab.engine import StableWindow, limit_command
from strike_lab.curves import Quintic
from strike_lab.methods.hybrid import DEFINITION


TUNING_PATH = Path(__file__).resolve().parents[1] / 'config/experiment_hardware_tuned.json'
MIN_REBOUND = math.radians(4.)
MAX_EVENT_LATENESS = .080


def load_tuning(path=TUNING_PATH):
    config = json.loads(Path(path).read_text())
    return (DEFINITION.parameters(config.get('parameters', {}).get('hybrid')),
            Rules(**config.get('rules', {})), Limits(**config.get('limits', {})))


@dataclass(frozen=True)
class HybridStatus(Status):
    request_id: int = 0
    at: float = 0.
    released_at: float | None = None
    returned_at: float | None = None
    bottom_at: float | None = None
    scheduled_at: float | None = None
    count: int = 0
    bottoms: int = 0
    label: str = ''
    main_beat: bool = False
    swing: bool = False
    partial_returns: int = 0
    # Durable sequence numbers let the GUI detect a missed hi-hat event.
    main_count: int = 0
    main_at: float | None = None
    last_peak: float = 0.
    last_lateness: float = 0.
    closed_count: int = 0
    closed_peak: float = 0.
    closed_label: str = ''
    timing_inertia: float = 0.


class HybridController:
    """Pure feedback-driven search/swing state machine, ticked by the worker."""

    def __init__(self, anchor, lower, upper, parameters, rules, limits, bias=0.):
        self.anchor, self.lower, self.upper = float(anchor), float(lower), float(upper)
        if not self.lower <= self.anchor <= self.upper:
            raise ValueError('Hybrid anchor is outside the validated J7 corridor')
        self.parameters = DEFINITION.parameters(parameters)
        self.rules, self.limits, self.bias = rules, limits, bias
        self.window = StableWindow(anchor, rules)
        self.method = None
        self.status = HybridStatus()
        self.last_sample_at = None
        self.amount = None
        self.events = ()
        self.event_index = 0
        self.next_event_at = None
        self.marked_bottom = False
        self.main_reported = False
        self.search_pending = None
        self.swing_pending = None
        self.wait_started = None
        # Scheduling-only identification. The experiment's method parameters
        # and commanded impulse/catch/return remain byte-for-byte unchanged.
        self.timing_inertia = self.parameters['inertia']
        self.coast_samples = deque()
        self.short_return = False
        self.partial_curve = self.finish_curve = None
        self.finish_after_partial = False

    def validate_depth(self, amount):
        if not math.isfinite(amount) or amount <= 0:
            raise ValueError('Hybrid depth must be finite and positive')
        # lower/upper already intersect the URDF and safe-zone path. Do not
        # reparse the URDF (filesystem I/O) during a partial-return release.
        StrikeGoal(math.degrees(amount)).validate(self.rules, self.limits)
        if self.anchor-amount < self.lower:
            raise ValueError('Hybrid depth exceeds the validated J7 path')

    def request_search(self, amount):
        self.validate_depth(amount)
        if self.method or self.search_pending is not None or self.status.swing:
            raise ValueError('Hybrid search is already active')
        self.search_pending = amount

    def request_swing(self, amount, events):
        self.validate_depth(amount)
        if not self.status.ready or self.method or self.search_pending is not None:
            raise ValueError('Hybrid swing must start at the settled search anchor')
        if not events or any(not math.isfinite(e[2]) or e[2] < .2-1e-9 for e in events):
            raise ValueError('Invalid hybrid swing grid')
        self.events = tuple(events)
        self.event_index = len(events)-1  # Opening pickup, as in the powered beat.
        self.next_event_at = None
        self.swing_pending = amount
        self.status = replace(self.status, swing=True, count=0, bottoms=0,
                              main_count=0, main_at=None, partial_returns=0,
                              closed_count=0, closed_peak=0., closed_label='')

    def stop_swing(self):
        """Cancel future releases, never interrupt the current catch/return."""
        self.search_pending = self.swing_pending = None
        self.finish_after_partial = self.partial_curve is not None and self.method is not None
        # Complete the short rebound's braking first. Joining an arbitrary
        # high-acceleration intermediate state straight to the anchor can make
        # a quintic overshoot it; the partial endpoint has exactly zero v/a.
        if not self.finish_after_partial:
            self.short_return = False
        self.status = replace(self.status, swing=False)

    def hold(self, sample, dt):
        # Identical to Engine.hold: bias adapts only at rest, frozen in a stroke.
        if abs(sample.velocity) < .2 and abs(self.anchor-sample.position) < .05:
            self.bias = max(-2., min(2., self.bias+60*(self.anchor-sample.position)*min(.02, dt)))
        return Command(self.anchor, 0, 40, 1.8, self.bias)

    def arrival_seconds(self, sample, now):
        """Predict hybrid release-to-bottom, including current upward momentum.

        Uses the configured load/inertia and damped impulse, not CSP speed.
        This model schedules releases only; real feedback always controls the
        unchanged experiment catch. It is not a physical timing guarantee.
        """
        p = self.parameters
        x, v = self.anchor-sample.position, -sample.velocity
        dt = .002
        load = p['load_torque'] or self.bias*p['bias_scale']
        for step in range(400):
            t = step*dt
            pulse = math.sin(math.pi*t/p['kick_time'])**2 if t < p['kick_time'] else 0.
            acceleration = (load+p['kick_torque']*pulse-(p['friction']+p['fall_kd'])*v)/self.timing_inertia
            nv = v+acceleration*dt
            x += (v+nv)*.5*dt
            v = nv
            speed = max(0., v)
            catch_time = max(.004, 1.5*speed/p['brake'])
            depth = x+speed*(max(0., now-sample.at)+p['latency']+.002)
            base = depth+.5*speed*catch_time
            join = min(1.5*speed/catch_time, p['join_shape']*max(0., base)/p['return_time']**2)
            if base+join*catch_time**2/12 >= self.amount:
                return t+dt+catch_time
        raise RuntimeError('Hybrid arrival prediction exceeded 0.8 seconds')

    def observe_timing(self, sample):
        """Identify effective coast inertia from a 20 ms velocity slope.

        Only use free coast well before contact/catch. This compensates for
        a mounted stick/load differing from the nominal simulated plant;
        it changes release lead time, never torque, gains, or motion limits.
        """
        if (self.method is None or self.status.phase != 'coast'
                or self.anchor-sample.position > self.amount-math.radians(2.)):
            self.coast_samples.clear()
            return
        self.coast_samples.append(sample)
        while self.coast_samples and sample.at-self.coast_samples[0].at > .035:
            self.coast_samples.popleft()
        first = self.coast_samples[0]
        dt = sample.at-first.at
        if dt < .020:
            return
        accel = (first.velocity-sample.velocity)/dt
        velocity = -.5*(sample.velocity+first.velocity)
        p = self.parameters
        force = (p['load_torque'] or self.bias*p['bias_scale'])-(p['friction']+p['fall_kd'])*velocity
        if 5. < accel < 200. and force > .05:
            estimate = force/accel
            if .5*p['inertia'] <= estimate <= 4*p['inertia']:
                self.timing_inertia = .9*self.timing_inertia+.1*estimate

    def _release(self, amount, sample, now, swing=False):
        self.validate_depth(amount)
        partial = self.method is not None
        self._finalize_measurement()
        self.amount = amount
        self.partial_curve = self.finish_curve = None
        self.finish_after_partial = False
        self.short_return = bool(swing and self.events[self.event_index][2] <= .200001)
        self.method = DEFINITION.create(self.anchor, self.bias, self.parameters, target=amount)
        self.method.start(now, list(self.window.samples))
        self.window = StableWindow(self.anchor, self.rules)
        self.marked_bottom = self.main_reported = False
        label, main, scheduled = '', False, None
        if swing:
            label, main, _ = self.events[self.event_index]
            if self.status.count == 0:
                label = 'pickup (extra before beat 1)'
            scheduled = self.next_event_at
        self.status = replace(self.status, phase='drive', ready=False,
                              released_at=now, returned_at=None, peak_drop=0.,
                              count=self.status.count+1, label=label, main_beat=main,
                              scheduled_at=scheduled,
                              partial_returns=self.status.partial_returns+int(partial))
        self.wait_started = None

    def _finalize_measurement(self):
        s = self.status
        if s.count > s.closed_count:
            self.status = replace(s, closed_count=s.count, closed_peak=s.peak_drop,
                                  closed_label=s.label)

    def update(self, sample, now):
        l, s = self.limits, self.status
        moving = self.method is not None
        max_age = l.feedback_timeout if moving else .25
        if (sample is None or not all(math.isfinite(v) for v in
                (sample.position, sample.velocity, sample.torque, sample.at))
                or sample.state != 2 or not -.001 <= now-sample.at <= max_age):
            raise RuntimeError('Hybrid feedback stale, invalid, or J7 not running')
        if not self.lower <= sample.position <= self.upper:
            raise RuntimeError('Hybrid measured J7 left the validated corridor')
        if abs(sample.velocity) > l.velocity*1.25:
            raise RuntimeError('Hybrid measured speed exceeds experiment limit')
        if not moving and now-sample.at > l.feedback_timeout:
            self.window = StableWindow(self.anchor, self.rules)
            self.status = replace(s, at=now, sample=sample, ready=False)
            return Command(self.anchor, 0, 40, 1.8, self.bias)
        dt = 1/l.hz if self.last_sample_at is None else max(0., sample.at-self.last_sample_at)
        self.last_sample_at = sample.at
        self.observe_timing(sample)
        # Match the experiment's pre-release priming rule, not merely the
        # looser in-motion abort age. The worker switches to 500 Hz to prime.
        release_fresh = now-sample.at <= min(.006, self.rules.maximum_feedback_gap/2)
        if not moving:
            self.window.add(sample)
            self.status = replace(s, ready=self.window.ready)
            if self.window.ready:
                self.wait_started = None
                if self.search_pending is not None and release_fresh:
                    amount, self.search_pending = self.search_pending, None
                    self._release(amount, sample, now)
                elif self.swing_pending is not None and release_fresh:
                    amount, self.swing_pending = self.swing_pending, None
                    self._release(amount, sample, now, True)
            else:
                if self.wait_started is None:
                    self.wait_started = now
                if now-self.wait_started > 3.:
                    raise RuntimeError('Hybrid anchor did not settle')

        # A return may be truncated only AFTER its bottom and real 4-degree
        # rebound; never cut off a descent/catch or rebase the fixed anchor.
        if (self.status.swing and self.next_event_at is not None
                and (self.method is None or self.marked_bottom)):
            rebound = sample.position-(self.anchor-self.amount)
            remaining = self.next_event_at-now
            if remaining < -MAX_EVENT_LATENESS:
                raise RuntimeError('Hybrid missed swing deadline; no catch-up burst')
            if release_fresh and rebound >= MIN_REBOUND and remaining <= self.arrival_seconds(sample, now):
                self._release(self.amount, sample, now, True)

        method = self.method
        if method is None:
            command = self.hold(sample, dt)
            phase = 'hold'
        else:
            if now-method.started > l.trial_timeout:
                raise RuntimeError('Hybrid strike/return timeout')
            command = method.update(sample, now)
            phase = method.phase
            if self.finish_curve is not None:
                elapsed = now-self.finish_started
                method.done = elapsed >= self.finish_curve.duration
                phase = 'settling' if method.done else 'withdrawal'
                command = method.hold() if method.done else method.tracking(self.finish_curve.at(elapsed))
            elif (self.short_return and method.catch is not None
                    and now >= method.catch_at+method.catch.catch_time):
                # A full-anchor return builds too much upward momentum for a
                # 200 ms pair. Plan the partial return itself, rather than
                # cutting a fast full return only after it already accelerated.
                # Reuse the experiment's C2 quintic and tracking gains. Search,
                # impulse, predictive catch and all longer returns are unchanged.
                if self.partial_curve is None:
                    rebound = min(self.amount, MIN_REBOUND+math.radians(.4))
                    self.partial_curve = Quintic(method.catch.low, 0., -method.catch.join,
                        method.catch.low-rebound, 0., 0., .110)
                elapsed = now-method.catch_at-method.catch.catch_time
                command = method.tracking(self.partial_curve.at(elapsed))
                method.done = False
                phase = 'withdrawal'
                if self.finish_after_partial and elapsed >= self.partial_curve.duration:
                    endpoint = self.partial_curve.at(self.partial_curve.duration)[0]
                    self.finish_curve = Quintic(endpoint, 0., 0., 0., 0., 0., self.parameters['return_time'])
                    self.finish_started = now
                    self.short_return = False
            self.status = replace(self.status, peak_drop=max(self.status.peak_drop, self.anchor-sample.position))
            if (method.catch is not None and not self.marked_bottom
                    and now >= method.catch_at+method.catch.catch_time):
                self.marked_bottom = True
                scheduled = self.status.scheduled_at
                lateness = 0. if scheduled is None else now-scheduled
                self.status = replace(self.status, bottoms=self.status.bottoms+1,
                                      bottom_at=now, last_lateness=lateness)
                if self.status.swing:
                    if abs(lateness) > MAX_EVENT_LATENESS:
                        raise RuntimeError('Hybrid target arrival missed swing grid')
                    event_at = now if scheduled is None else scheduled
                    self.next_event_at = event_at+self.events[self.event_index][2]
                    self.event_index = (self.event_index+1) % len(self.events)
            if (self.status.swing and self.status.main_beat and not self.main_reported
                    and self.status.scheduled_at is not None and now >= self.status.scheduled_at):
                self.main_reported = True
                self.status = replace(self.status, main_count=self.status.main_count+1,
                                      main_at=self.status.scheduled_at)
            if method.done:
                # Clear pre-bottom/return history; require a new settled window.
                if phase != self.status.phase or self.status.phase != 'settling':
                    self.window = StableWindow(self.anchor, self.rules)
                self.window.add(sample)
                command = self.hold(sample, dt)
                if self.window.ready:
                    self.method = None
                    phase = 'hold'
                    self._finalize_measurement()
                    self.status = replace(self.status, ready=True, completed=self.status.completed+1,
                                          returned_at=now, last_peak=self.status.peak_drop)
        self.status = replace(self.status, phase=phase, at=now, sample=sample,
                              timing_inertia=self.timing_inertia)
        return command


def _run_hybrid(interface, anchor, lower, upper, parameters, rules, limits,
                requests, stop, heartbeat, sender, channel_factory):
    """Only process allowed to command J7 while the hybrid session is alive."""
    channel = None
    controller = HybridController(anchor, lower, upper, parameters, rules, limits)
    last_publish = last_send = None
    last_torque = 0.

    def publish(status, force=False):
        nonlocal last_publish
        now = time.monotonic()
        if not force and last_publish is not None and now-last_publish < .005:
            return
        try:
            sender.send(pickle.dumps(status))
        except BlockingIOError:
            pass
        last_publish = now

    try:
        channel = channel_factory(interface)
        helper = Joint7Worker(interface, anchor, lower, upper, StrikeSettings(),
                              requests, stop, lambda s: publish(HybridStatus(
                                  phase=s.phase, sample=s.sample, at=time.monotonic()), True))
        helper._prepare(channel)
        controller.bias = helper.controller.bias
        last_torque = controller.bias
        last_send = helper.last_command_at
        deadline = time.monotonic()
        while not stop.is_set():
            channel.receive()
            now = time.monotonic()
            if now < deadline:
                channel.wait(deadline-now)
                continue
            if now-heartbeat.value > .5:
                raise RuntimeError('Hybrid GUI heartbeat lost')
            if now-last_send > (limits.control_timeout if controller.method else .25):
                raise RuntimeError('Hybrid control deadline missed')
            try:
                request = requests.get_nowait()
            except queue.Empty:
                request = None
            if request:
                request_id, action, *args = request
                if action == 'search':
                    controller.request_search(*args)
                elif action == 'swing':
                    controller.request_swing(*args)
                elif action == 'finish':
                    controller.stop_swing()
                else:
                    raise ValueError('Unknown hybrid request')
                controller.status = replace(controller.status, request_id=request_id)
            sample = channel.sample
            command = controller.update(sample, now)
            before = time.monotonic()
            if (before-sample.at > limits.feedback_timeout or
                    before-last_send > limits.control_timeout) and controller.method:
                raise RuntimeError('Hybrid tick expired before send')
            command, torque, _ = limit_command(command, sample, before, anchor, limits, last_send, last_torque)
            if not lower <= command.position <= upper:
                raise RuntimeError('Hybrid command exceeds validated corridor')
            if stop.is_set():
                break
            channel.send(command.frame())
            last_send, last_torque = time.monotonic(), torque
            publish(controller.status)
            priming = controller.search_pending is not None or controller.swing_pending is not None
            period = 1/(limits.hz if controller.method or controller.status.swing or priming else 100.)
            deadline = max(deadline+period, last_send+period if deadline+period <= last_send else 0.)
    except InterruptedError:
        pass
    except Exception as exc:
        publish(replace(controller.status, phase='fault', ready=False, error=str(exc), at=time.monotonic()), True)
        # Stop following obsolete trajectories. Retain a current-pose
        # hold for the GUI's explicit fault handling; never auto-retry a strike.
        if channel is not None and channel.sample is not None and not stop.is_set():
            try:
                q = min(upper, max(lower, channel.sample.position))
                channel.send(Command(q, 0, 40, 1.8, max(-2., min(2., controller.bias))).frame())
            except Exception:
                pass
    finally:
        if channel is not None:
            channel.close()
        sender.close()


class HybridSession(Joint7Session):
    """Reuse the established joined ownership/IPC lifecycle, not its strategy."""

    def __init__(self, bus, anchor, lower, upper, tuning=None, channel_factory=Joint7Channel):
        if os.environ.get('STRIKE_LAB_OFFLINE_ONLY') == '1' and channel_factory is Joint7Channel:
            raise RuntimeError('Physical hybrid transport forbidden in offline tests')
        if bus.control_side != 'right' or not bus.active or bus.right_joint7_session is not None:
            raise RuntimeError('Hybrid requires an active, unowned right J7')
        parameters, rules, limits = tuning or load_tuning()
        self.controller = HybridController(anchor, lower, upper, parameters, rules, limits)
        self.bus = bus
        self.lower, self.upper = lower, upper
        self.closed = False
        self.request_id = 0
        ctx = multiprocessing.get_context('spawn')
        self.requests = ctx.Queue(maxsize=4)
        self.stop_event = ctx.Event()
        self.heartbeat = ctx.Value('d', time.monotonic())
        self.receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.receiver.setblocking(False)
        sender.setblocking(False)
        self._status = HybridStatus()
        self.process = ctx.Process(target=_run_hybrid, name='j7-hybrid-beat', args=(
            bus.sockets['right'].getsockname()[0], anchor, lower, upper, parameters,
            rules, limits, self.requests, self.stop_event, self.heartbeat, sender, channel_factory))
        bus.right_joint7_session = self
        try:
            self.process.start()
        except Exception:
            bus.right_joint7_session = None
            self.receiver.close()
            self.requests.close()
            raise
        finally:
            sender.close()

    @property
    def status(self):
        self.heartbeat.value = time.monotonic()
        return super().status

    def request(self, action, *args):
        if self.closed:
            raise RuntimeError('Hybrid session is closed')
        if action in ('search', 'swing'):
            self.controller.validate_depth(args[0])
        try:
            self.request_id += 1
            self.requests.put_nowait((self.request_id, action, *args))
        except queue.Full:
            raise RuntimeError('Hybrid request queue full; no retry') from None
        return self.request_id
