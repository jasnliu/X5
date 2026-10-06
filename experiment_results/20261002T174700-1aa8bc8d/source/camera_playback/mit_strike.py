"""Hardware-test-only J7 control: gravity drop, predictive catch, smooth return.

The pure controller is also used by the offline dynamics tests. The session owns
one filtered CAN socket and a deadline-driven worker; Tk never clocks the motion.
Defaults are initial tuning, NOT a claim of calibration on the physical arm.
"""
from collections import deque
from dataclasses import dataclass, replace
import math
import multiprocessing
import pickle
import queue
import select
import socket
import struct
import time

from centering.motors import motion_control_packet, packet, parameter
from safe_zone.encoder import EFF, FRAME, request_frame


@dataclass(frozen=True)
class StrikeSettings:
    hz: float = 500.0
    fall_kd: float = 0.04       # Gravity still drives descent: KP=0, torque_ff=0.
    kp: float = 40.0
    kd: float = 1.8
    inertia: float = 0.02      # Effective loaded J7 inertia, kg m^2; calibrate.
    brake_accel: float = 40.0  # Achievable upward acceleration, rad/s^2; calibrate.
    return_accel: float = 20.0
    return_speed: float = 2.0  # Trajectory peak, NOT a constant MIT velocity bias.
    command_latency: float = 0.004  # Delivery/actuation estimate, seconds; calibrate.
    feedback_timeout: float = 0.020
    control_timeout: float = 0.020
    hold_timeout: float = 0.250
    hold_hz: float = 100.0
    position_tolerance: float = math.radians(0.15)
    velocity_tolerance: float = 0.03
    settled_seconds: float = 0.040  # Encoder-position window, not raw velocity dwell.
    settled_range: float = math.radians(0.05)
    settle_hysteresis: float = 2.0
    status_timeout: float = 0.100  # UI heartbeat only; never permits stale strikes.
    motion_timeout: float = 3.0

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("MIT settings must be finite and positive")
        if not 50 <= self.hz <= 1000 or self.kp > 500 or max(self.kd, self.fall_kd) > 5:
            raise ValueError("MIT settings exceed supported command ranges")
        if self.return_speed > 33 or self.inertia*max(self.brake_accel, self.return_accel) > 12:
            raise ValueError("MIT trajectory feed-forward exceeds drive range")


@dataclass(frozen=True)
class Sample:
    position: float
    velocity: float
    torque: float
    state: int
    at: float              # Kernel receive timestamp converted to monotonic time.


@dataclass(frozen=True)
class Command:
    position: float
    velocity: float
    kp: float
    kd: float
    torque: float = 0.0

    def frame(self):
        # Displayed right-J7 coordinates have the opposite motor sign.
        values = (self.position, self.velocity, self.kp, self.kd, self.torque)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite J7 command")
        if abs(self.position) > 3.5 or abs(self.velocity) > 33 or abs(self.torque) > 14:
            raise ValueError("J7 command exceeds drive range")
        return motion_control_packet(7, -self.position, -self.velocity,
                                     self.kp, self.kd, -self.torque)


@dataclass(frozen=True)
class Status:
    phase: str = "arming"
    ready: bool = False
    completed: int = 0
    sample: Sample | None = None
    peak_drop: float = 0.0
    overshoot: float = 0.0
    error: str | None = None
    rejected: str | None = None
    last_gap: float = 0.0
    max_gap: float = 0.0
    feedback_age: float = 0.0
    hold_recoveries: int = 0
    commands_sent: int = 0


class StrikeController:
    """No I/O, UI, or sleeps. All times and measured states are explicit."""

    def __init__(self, anchor, settings=StrikeSettings()):
        self.anchor = float(anchor)
        if not math.isfinite(self.anchor) or abs(self.anchor) > 3.5:
            raise ValueError("Invalid J7 reference anchor")
        self.settings = settings
        self.phase = "hold"
        self.ready = False
        self.completed = 0
        self.bias = 0.0
        self.last_sample_at = None
        self.settle_history = deque()
        self.started_at = None
        self.phase_at = 0.0
        self.target = self.anchor
        self.peak_drop = self.overshoot = 0.0
        self.pending_completion = False
        self.progress_position = self.anchor
        self.progress_at = 0.0

    def check_sample(self, sample, now, max_age=None):
        max_age = self.settings.feedback_timeout if max_age is None else max_age
        if (not all(math.isfinite(v) for v in
                    (sample.position, sample.velocity, sample.torque, sample.at))
                or not -0.001 <= now - sample.at <= max_age):
            raise RuntimeError(f"J7 feedback stale or invalid in {self.phase}: "
                               f"age={(now-sample.at)*1000:.1f} ms, limit={max_age*1000:g} ms")
        if sample.state != 2:
            raise RuntimeError("Motor 7: MIT drive is not running")

    def observe_settling(self, sample):
        """Use recent encoder travel/trend, not the drive's noisy raw velocity.

        Enter on a full quiet window; retain readiness with wider travel/trend
        thresholds. The anchor-position tolerance is never widened. This is
        used only in hold and before accepting a press, never for catch timing.
        """
        history = self.settle_history
        s = self.settings
        if history and sample.at <= history[-1].at:
            return  # Reused/out-of-order samples cannot advance readiness.
        if history and sample.at-history[-1].at > s.settled_seconds:
            self.reset_readiness()
        history.append(sample)
        cutoff = sample.at-s.settled_seconds
        while len(history) > 2 and history[1].at <= cutoff:
            history.popleft()
        if (abs(sample.position-self.anchor) > s.position_tolerance
                or len(history) < 3
                or sample.at-history[0].at < s.settled_seconds-1e-9):
            self.ready = False
            return
        times = [item.at-history[0].at for item in history]
        positions = [item.position for item in history]
        mean_t, mean_q = sum(times)/len(times), sum(positions)/len(positions)
        trend = sum((t-mean_t)*(q-mean_q) for t,q in zip(times,positions)) / sum(
            (t-mean_t)**2 for t in times)
        factor = s.settle_hysteresis if self.ready else 1.0
        self.ready = (max(abs(q-self.anchor) for q in positions) <= s.position_tolerance
                      and max(positions)-min(positions) <= s.settled_range*factor
                      and abs(trend) <= s.velocity_tolerance*factor)

    def strike(self, amount, sample, now):
        self.check_sample(sample, now)
        was_ready = self.ready
        self.observe_settling(sample)
        if not was_ready or not self.ready:
            raise ValueError("J7 is not stationary at the reference anchor")
        if not math.isfinite(amount) or amount <= 0:
            raise ValueError("Drop must be finite and positive")
        self.target = self.anchor - amount
        self.phase = "fall"
        self.reset_readiness()
        self.started_at = self.phase_at = now
        self.peak_drop = self.overshoot = 0.0
        self.pending_completion = True
        self.progress_position, self.progress_at = sample.position, now

    def hold_command(self):
        s = self.settings
        return Command(self.anchor, 0.0, s.kp, s.kd, self.bias)

    @property
    def moving(self):
        return self.phase != "hold" or self.pending_completion

    def reset_readiness(self):
        self.ready = False
        self.settle_history.clear()

    def plan_reversal(self, position, velocity):
        """Plan one catch/return join; prediction uses the exact same low point.

        The catch ends at zero velocity but positive upward acceleration. The
        return preserves that acceleration, then ends at rest at the anchor.
        No contact detection or bottom settling is involved.
        """
        s = self.settings
        speed = max(0.0, -velocity)
        catch_time = max(2.0/s.hz, 1.5*speed/s.brake_accel)
        # This cap keeps the modified catch acceleration within brake_accel,
        # including tiny speeds where the minimum catch duration applies.
        max_join_accel = min(s.return_accel, 1.5*speed/catch_time)
        base_low = position - 0.5*speed*catch_time
        distance = self.anchor - base_low
        # Bound return duration before choosing the join acceleration. The
        # extra catch travel is A*catch_time^2/12, not the old stopping distance.
        max_distance = max(abs(distance),
                           abs(distance + max_join_accel*catch_time**2/12.0))
        return_time = max(0.04, 1.875*max_distance/s.return_speed,
                          math.sqrt(5.774*max_distance/s.return_accel))
        # A*T^2/d <= 4 gives a monotonic quintic return within the existing
        # speed/acceleration bounds, even when speed caps lengthen the return.
        join_accel = min(max_join_accel, 4.0*max(0.0, distance)/return_time**2)
        low = base_low - join_accel*catch_time**2/12.0
        return catch_time, join_accel, low, return_time

    def update(self, sample, now):
        s = self.settings
        self.check_sample(sample, now, s.feedback_timeout if self.moving else s.hold_timeout)
        if not self.moving and now-sample.at > s.feedback_timeout:
            if now-sample.at > s.settled_seconds:
                self.reset_readiness()
            return self.hold_command()  # No release/dwell advancement on old data.
        new_sample = self.last_sample_at is None or sample.at > self.last_sample_at
        dt = (0.0 if self.last_sample_at is None else
              max(0.0, min(sample.at - self.last_sample_at, s.feedback_timeout)))
        if self.pending_completion:
            self.peak_drop = max(self.peak_drop, self.anchor - sample.position)
            if self.phase != "fall":
                self.overshoot = max(self.overshoot, sample.position - self.anchor)
            if now - self.started_at > s.motion_timeout:
                raise RuntimeError("J7 MIT strike did not settle before timeout")
            if abs(sample.position - self.progress_position) >= math.radians(0.5):
                self.progress_position, self.progress_at = sample.position, now
            elif (self.phase != "fall" and now-self.progress_at > 0.5
                  and abs(sample.position-self.anchor) > math.radians(8.0)):
                raise RuntimeError("STALL: motor 7")

        if self.phase == "fall":
            down_speed = max(0.0, -sample.velocity)
            lead = max(0.0, now - sample.at) + s.command_latency + 1.0 / s.hz
            catch_position = sample.position - down_speed*lead
            plan = self.plan_reversal(catch_position, -down_speed)
            predicted_low = plan[2]
            if predicted_low <= self.target:
                self.phase = "catch"
                self.phase_at = now
                self.catch_position = catch_position
                self.catch_velocity = -down_speed
                (self.catch_duration, self.join_accel, self.return_position,
                 self.return_duration) = plan
            else:
                self.last_sample_at = sample.at
                return Command(sample.position, 0.0, 0.0, s.fall_kd)

        if self.phase == "catch":
            t = now - self.phase_at
            duration = self.catch_duration
            u = min(1.0, t / duration)
            v0 = self.catch_velocity
            a0 = self.join_accel
            q = (self.catch_position + v0*duration*(u - u**3 + 0.5*u**4)
                 + a0*duration**2*(0.25*u**4 - u**3/3.0))
            v = v0*(1.0 - 3*u*u + 2*u**3) + a0*duration*(u**3 - u*u)
            a = v0/duration*(-6*u + 6*u*u) + a0*(3*u*u - 2*u)
            if t < duration:
                self.last_sample_at = sample.at
                return Command(q, v, s.kp, s.kd, self.bias + s.inertia*a)
            self.phase = "return"
            self.phase_at += duration

        if self.phase == "return":
            u = min(1.0, (now - self.phase_at) / self.return_duration)
            d = self.anchor - self.return_position
            # Quintic with inherited upward acceleration at the bottom, but
            # zero velocity AND acceleration at the top. Do not restart at rest.
            a0 = self.join_accel
            q = (self.return_position + d*(10*u**3 - 15*u**4 + 6*u**5)
                 + 0.5*a0*self.return_duration**2*u*u*(1-u)**3)
            v = (d/self.return_duration*(30*u*u - 60*u**3 + 30*u**4)
                 + 0.5*a0*self.return_duration*(2*u - 9*u*u + 12*u**3 - 5*u**4))
            a = (d/self.return_duration**2*(60*u - 180*u*u + 120*u**3)
                 + a0*(1 - 9*u + 18*u*u - 10*u**3))
            if u < 1.0:
                self.last_sample_at = sample.at
                return Command(q, v, s.kp, s.kd, self.bias + s.inertia*a)
            self.phase = "hold"
            self.reset_readiness()

        # Small integral load compensation is frozen during motion. This avoids
        # changing the release/catch conditions between repeated strikes.
        if new_sample:
            if abs(sample.velocity) < 0.1 and abs(self.anchor - sample.position) < 0.05:
                self.bias = max(-2.0, min(2.0, self.bias +
                                         15.0*(self.anchor - sample.position)*dt))
            self.observe_settling(sample)
            if self.ready and self.pending_completion:
                self.completed += 1
                self.pending_completion = False
        self.last_sample_at = sample.at
        return self.hold_command()


class Joint7Channel:
    """Private right-J7 receiver. Other joints remain on the existing bus loop."""
    timestamp_option = getattr(socket, "SO_TIMESTAMPNS", 35)  # Linux LP64

    def __init__(self, interface):
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            mask = EFF | 0x40000000 | (31 << 24) | (255 << 8)
            filters = b"".join(struct.pack("=II", EFF | kind << 24 | 7 << 8, mask)
                               for kind in (2, 17, 21))
            self.socket.setsockopt(socket.SOL_CAN_RAW, 1, filters)
            self.socket.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack("=I", 0x1fffffff))
            self.socket.setsockopt(socket.SOL_SOCKET, self.timestamp_option, 1)
            self.socket.bind((interface,))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise
        self.sample = None
        self.mode = None
        self.mode_at = 0.0

    def send(self, frame):
        # Never queue retries/sleeps behind a time-critical catch command.
        if self.socket.send(frame) != FRAME.size:
            raise RuntimeError("Short J7 CAN write")

    def receive(self):
        for _ in range(128):
            try:
                frame, ancillary, flags, _ = self.socket.recvmsg(FRAME.size, 128)
            except BlockingIOError:
                return
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                raise RuntimeError("Truncated J7 feedback")
            at = None
            for level, kind, data in ancillary:
                if level == socket.SOL_SOCKET and kind == self.timestamp_option:
                    seconds, nanos = struct.unpack("@ll", data)
                    at = time.monotonic() - (time.time() - (seconds + nanos*1e-9))
            if at is None:
                raise RuntimeError("J7 feedback has no kernel timestamp")
            self.decode(frame, at)
        raise RuntimeError("J7 feedback backlog")

    def decode(self, frame, at):
        cid, dlc, data = FRAME.unpack(frame)
        if cid & 0x60000000:
            raise RuntimeError("J7 CAN transport error")
        kind, motor = (cid >> 24) & 31, (cid >> 8) & 255
        if not cid & EFF or motor != 7:
            return
        if kind == 21 or (kind == 2 and (cid >> 16) & 63):
            raise RuntimeError("Motor 7: drive fault")
        if dlc != 8:
            raise RuntimeError("Short J7 feedback")
        if kind == 17 and data[:2] == b'\x05\x70':
            self.mode, self.mode_at = data[4], at
        elif kind == 2:
            if int.from_bytes(data[6:8], "big") * 0.1 >= 65:
                raise RuntimeError("Motor 7: too hot")
            q = -(int.from_bytes(data[:2], "big") / 65535*25.14 - 12.57)
            v = -(int.from_bytes(data[2:4], "big") / 65535*66.0 - 33.0)
            torque = -(int.from_bytes(data[4:6], "big") / 65535*28.0 - 14.0)
            if self.sample is None or at > self.sample.at:
                self.sample = Sample(q, v, torque, (cid >> 22) & 3, at)

    def wait(self, timeout):
        select.select([self.socket], [], [], max(0.0, timeout))

    def close(self):
        self.socket.close()


class Joint7Worker:
    """Control engine; production runs this in its own spawned Python process."""

    def __init__(self, interface, anchor, lower, upper, settings, requests,
                 stop_event, publish, channel_factory=Joint7Channel):
        self.interface = interface
        self.controller = StrikeController(anchor, settings)
        self.lower, self.upper = lower, upper
        self.requests, self.stop_event = requests, stop_event
        self.publish = publish
        self.channel_factory = channel_factory
        self.status = Status()
        self.last_command_at = None
        self.max_gap = 0.0
        self.hold_recoveries = 0
        self.commands_sent = 0

    def _check_timing(self, sample, now):
        c = self.controller
        gap = 0.0 if self.last_command_at is None else now-self.last_command_at
        self.max_gap = max(self.max_gap, gap)
        limit = c.settings.control_timeout if c.moving else c.settings.hold_timeout
        if gap > limit:
            raise RuntimeError(f"J7 control deadline missed in {c.phase}: "
                               f"gap={gap*1000:.1f} ms, limit={limit*1000:g} ms, "
                               f"feedback age={(now-sample.at)*1000:.1f} ms")
        c.check_sample(sample, now, c.settings.feedback_timeout if c.moving
                       else c.settings.hold_timeout)
        if not c.moving:
            if now-sample.at > c.settings.feedback_timeout:
                if now-sample.at > c.settings.settled_seconds:
                    c.reset_readiness()
                return False
            if gap > c.settings.control_timeout:
                return False  # A late idle send alone does not mean the arm moved.
        return True

    def _send(self, channel, frame):
        if self.stop_event.is_set():
            raise InterruptedError
        channel.send(frame)
        if (FRAME.unpack(frame)[0] >> 24) & 31 == 1:
            self.last_command_at = time.monotonic()  # Successful write, not tick start.
            self.commands_sent += 1

    def _prepare(self, channel):
        c = self.controller
        deadline = time.monotonic() + 0.5
        # Start from a received state, never from a stale GUI snapshot.
        while channel.sample is None or time.monotonic()-channel.sample.at > c.settings.feedback_timeout:
            if time.monotonic() > deadline:
                raise RuntimeError("Fresh J7 feedback unavailable before MIT setup")
            self._send(channel, request_frame(7))
            channel.wait(0.002)
            channel.receive()
        if abs(channel.sample.velocity) <= 0.1:
            c.bias = max(-2.0, min(2.0, channel.sample.torque))
        # Confirm each handoff step instead of assuming that two short sleeps
        # mean the drive accepted it. Mode changes happen only at session entry.
        disabled_at = time.monotonic()
        self._send(channel, packet(4, 7))
        self._wait_setup(channel, lambda: channel.sample is not None
                         and channel.sample.at >= disabled_at
                         and channel.sample.state == 0, "disable confirmation")
        mode_at = time.monotonic()
        self._wait_setup(channel, lambda: channel.mode == 0
                         and channel.mode_at >= mode_at, "MIT mode readback",
                         retry_frame=parameter(7, 0x7005, 0, True))
        enabled_at = time.monotonic()
        self._send(channel, packet(3, 7))
        deadline = enabled_at + 0.5
        last_enable = enabled_at
        while True:
            self._send(channel, c.hold_command().frame())
            channel.wait(1.0/c.settings.hz)
            channel.receive()
            now = time.monotonic()
            sample = channel.sample
            self.publish(Status(phase="arming", sample=sample))
            if sample and sample.at >= enabled_at and sample.state == 2:
                c.check_sample(sample, now)
                return
            if now > deadline:
                state = None if sample is None else sample.state
                raise RuntimeError(f"J7 MIT enable confirmation missing (state={state})")
            # Some firmware transitions do not accept the first enable. Retry
            # only while arming, not during a strike, and never after stop().
            if now-last_enable >= 0.020:
                self._send(channel, packet(3, 7))
                last_enable = now

    def _wait_setup(self, channel, confirmed, step, retry_frame=None):
        deadline = time.monotonic() + 0.150
        last_query = 0.0
        while True:
            now = time.monotonic()
            if now-last_query >= 0.010:
                if retry_frame is not None:
                    self._send(channel, retry_frame)
                self._send(channel, request_frame(7))
                self._send(channel, packet(17, 7, bytes.fromhex('0570000000000000')))
                last_query = now
            channel.wait(0.002)
            channel.receive()
            if confirmed():
                return
            if time.monotonic() > deadline:
                raise RuntimeError(f"J7 {step} missing")
            if self.stop_event.is_set():
                raise InterruptedError

    def run(self):
        channel = None
        c = self.controller
        try:
            channel = self.channel_factory(self.interface)
            self._prepare(channel)
            deadline = time.monotonic()
            while not self.stop_event.is_set():
                channel.receive()
                now = time.monotonic()
                if now < deadline:
                    channel.wait(deadline-now)
                    continue
                sample = channel.sample
                timing_ready = self._check_timing(sample, now)
                if not timing_ready:
                    self.hold_recoveries += 1
                    if c.ready and now-sample.at <= c.settings.settled_seconds:
                        # Retain the last received point before replacing it
                        # with the refresh reply. Skipping it would fabricate
                        # a larger gap in otherwise continuous encoder history.
                        c.observe_settling(sample)
                    # Refresh the retained idle hold and collect its reply
                    # before considering a press. A short scheduler slip can
                    # age the previous reply without any actual arm movement.
                    self._send(channel, c.hold_command().frame())
                    channel.wait(1.0/c.settings.hz)
                    channel.receive()
                    sample = channel.sample
                    now = time.monotonic()
                    timing_ready = self._check_timing(sample, now)
                rejected = self.status.rejected
                try:
                    amount = self.requests.get_nowait()
                except queue.Empty:
                    pass
                else:
                    try:
                        if not timing_ready:
                            raise ValueError("Wait for fresh, settled J7 feedback after hold timing recovery")
                        c.strike(amount, sample, now)
                        rejected = None
                    except ValueError as exc:
                        rejected = str(exc)  # A rejected button press is never retried.
                command = c.update(sample, now)
                if not self.lower <= command.position <= self.upper:
                    raise RuntimeError("J7 planned motion exceeds joint limits")
                before_send = time.monotonic()
                # Computation or scheduling could have consumed the time budget.
                # Never send a release/catch trajectory based on an old tick time.
                self._check_timing(sample, before_send)
                gap = before_send-self.last_command_at
                self._send(channel, command.frame())
                self.status = Status(c.phase, c.ready, c.completed, sample,
                                     c.peak_drop, c.overshoot, rejected=rejected,
                                     last_gap=gap, max_gap=self.max_gap,
                                     feedback_age=before_send-sample.at,
                                     hold_recoveries=self.hold_recoveries,
                                     commands_sent=self.commands_sent)
                self.publish(self.status)
                period = 1.0/(c.settings.hz if c.moving else c.settings.hold_hz)
                deadline += period
                if deadline <= self.last_command_at:
                    deadline = self.last_command_at + period  # No obsolete-command burst.
        except InterruptedError:
            pass
        except Exception as exc:
            self.status = replace(self.status, phase="fault", ready=False, error=str(exc),
                                  max_gap=self.max_gap)
            self.publish(self.status)
            if channel is not None and not self.stop_event.is_set():
                try:
                    channel.send(c.hold_command().frame())
                except Exception:
                    pass  # Existing GUI fault handler reports hold-delivery failures.
        finally:
            if channel is not None:
                channel.close()


def _run_joint7_process(interface, anchor, lower, upper, settings, requests,
                        stop_event, status_socket, channel_factory):
    # No Tk/ROS calls in this control engine; no inherited arm sockets (spawn).
    last_publish = 0.0
    last_phase = None
    def publish(status):
        nonlocal last_publish, last_phase
        now = time.monotonic()
        if now-last_publish < .005 and status.phase == last_phase and not status.error:
            return
        try:
            status_socket.send(pickle.dumps(status))
        except BlockingIOError:
            pass  # A stalled GUI must never stall motor control.
        last_publish, last_phase = now, status.phase
    try:
        Joint7Worker(interface, anchor, lower, upper, settings, requests,
                     stop_event, publish, channel_factory).run()
    finally:
        status_socket.close()


class Joint7Session:
    """Parent-side facade; the spawned process is the sole J7 command writer."""

    def __init__(self, bus, anchor, lower, upper, settings=StrikeSettings(),
                 channel_factory=Joint7Channel):
        if bus.control_side != "right" or not bus.active:
            raise RuntimeError("An active right arm is required for the MIT session")
        if getattr(bus, "right_joint7_session", None) is not None:
            raise RuntimeError("J7 already has a controller")
        self.bus = bus
        # Only immutable anchor/settings are used by the GUI. Actual controller
        # state stays in the child process and is reported through Status.
        self.controller = StrikeController(anchor, settings)
        self.lower, self.upper = float(lower), float(upper)
        if not self.lower <= self.controller.anchor <= self.upper:
            raise ValueError("J7 reference anchor is outside its limits")
        ctx = multiprocessing.get_context("spawn")
        self.requests = ctx.Queue(maxsize=1)
        self.stop_event = ctx.Event()
        self.receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.receiver.setblocking(False)
        sender.setblocking(False)
        self._status = Status()
        self.closed = False
        self.process = ctx.Process(
            target=_run_joint7_process, name="j7-mit-strike",
            args=(bus.sockets["right"].getsockname()[0], float(anchor), self.lower,
                  self.upper, settings, self.requests, self.stop_event, sender, channel_factory),
        )
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
    def owns_feedback(self):
        return self.status.sample is not None

    @property
    def status(self):
        if not self.closed:
            for _ in range(512):
                try:
                    self._status = pickle.loads(self.receiver.recv(65536))
                except BlockingIOError:
                    break
            if self.process.exitcode is not None and not self.stop_event.is_set() and not self._status.error:
                self._status = replace(self._status, ready=False, phase="fault",
                                       error=f"J7 worker exited (code {self.process.exitcode})")
        return self._status

    def strike(self, amount):
        status = self.status
        if not status.ready or status.error:
            raise ValueError("J7 is not ready at its stationary reference anchor")
        if (not math.isfinite(amount) or amount <= 0
                or not self.lower <= self.controller.anchor-amount <= self.upper):
            raise ValueError("Drop exceeds J7 limits")
        try:
            self.requests.put_nowait(amount)
        except queue.Full:
            raise ValueError("A manual strike is already pending") from None
        self._status = replace(status, ready=False, phase="pending", rejected=None)

    def stop(self):
        if self.closed:
            return
        self.stop_event.set()
        self.process.join(timeout=2.0)
        if self.process.is_alive():
            raise RuntimeError("J7 worker did not stop; command ownership retained")
        self.closed = True
        self.receiver.close()
        self.requests.cancel_join_thread()
        self.requests.close()
        if self.bus.right_joint7_session is self:
            self.bus.right_joint7_session = None
