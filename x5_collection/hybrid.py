"""Finite batches using the existing experiment hybrid and beat partial returns.

Only the collection schedule and inclusive goal validation differ. The measured
and commanded corridor stays at 12 degrees, NOT the beat's 13-degree corridor.
At a 12-degree goal there is no extra overshoot allowance: crossing it aborts.
Entry never sends a disable packet at the record3 endpoint. If this firmware
cannot confirm a powered CSP-to-MIT transfer, collection fails closed.
"""
from dataclasses import asdict, replace
import csv
import errno
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import pickle
import queue
import signal
import socket
import time

from camera_playback.hybrid_strike import HybridController, HybridSession, HybridStatus
from camera_playback.mit_strike import Command, Joint7Channel
from centering.motors import packet, parameter
from safe_zone.encoder import request_frame
from strike_lab.engine import limit_command


class FiniteController(HybridController):
    def validate_depth(self, amount):
        # Experiment's target+acceptance gate is for trial success scoring.
        # Collection does not require overshoot to count a successful trial.
        # Keep the actual command and encoder corridor strictly unchanged at 12.
        if not math.isfinite(amount) or not math.radians(10)-1e-12 <= amount <= math.radians(12)+1e-12:
            raise ValueError('Collection only permits 10-12 degree J7 displacements')
        if self.anchor-amount < self.lower-1e-12:
            raise ValueError('Collection target outside validated J7 corridor')

    def request_block(self, depth, count):
        if count not in (1, 2):
            raise ValueError('Only isolated strikes or finite double strikes')
        self.remaining = count
        events = (('second', False, .6), ('first', False, .2 if count == 2 else .6))
        self.request_swing(depth, events)

    def _release(self, *args, **kwargs):
        if self.remaining <= 0:
            raise RuntimeError('Finite collection release budget exhausted')
        super()._release(*args, **kwargs)
        self.remaining -= 1
        if self.remaining == 0:
            # Stop inside the 500 Hz controller, not a delayed parent GUI tick.
            # The final strike still completes its catch and full return.
            self.stop_swing()


def prepare_powered(channel, anchor, stop, publish):
    def setup_send(frame):
        # Stationary setup only. Parent state queries can briefly fill the
        # kernel's ten-frame queue; no retry/sleep is added to strike control.
        deadline = time.monotonic()+.02
        while True:
            try:
                channel.send(frame)
                return
            except OSError as exc:
                if exc.errno not in (errno.ENOBUFS, errno.EAGAIN) or time.monotonic() >= deadline:
                    raise
                if stop.is_set():
                    raise InterruptedError()
                time.sleep(.0005)

    deadline = time.monotonic()+.5
    while channel.sample is None or time.monotonic()-channel.sample.at > .02:
        if stop.is_set():
            raise InterruptedError()
        if time.monotonic() > deadline:
            raise RuntimeError('Fresh J7 feedback unavailable before powered MIT transfer')
        setup_send(request_frame(7)); channel.wait(.002); channel.receive()
    sample = channel.sample
    if sample.state != 2 or abs(sample.velocity) > .15 or abs(sample.position-anchor) > math.radians(.2):
        raise RuntimeError('Powered MIT transfer requires stationary active J7 at anchor')
    bias = max(-2., min(2., sample.torque))
    hold = Command(anchor, 0., 40., 1.8, bias)
    start = time.monotonic()
    setup_send(parameter(7, 0x7005, 0, True))
    # No disable/re-enable, no relaxation, and no retrying a failed transition.
    last_query = 0.
    while time.monotonic()-start < .5:
        if stop.is_set():
            raise InterruptedError()
        setup_send(hold.frame())
        if time.monotonic()-last_query >= .01:
            setup_send(packet(17, 7, bytes.fromhex('0570000000000000')))
            setup_send(request_frame(7))
            last_query = time.monotonic()
        channel.wait(.002); channel.receive()
        s = channel.sample
        publish(HybridStatus(phase='arming', sample=s, at=time.monotonic()))
        if s is None or s.state != 2 or abs(s.position-anchor) > math.radians(.6):
            raise RuntimeError('J7 moved/stopped during powered mode transfer')
        if channel.mode == 0 and channel.mode_at >= start and s.at >= start:
            return bias
    raise RuntimeError('Firmware did not confirm powered MIT transfer; no disable fallback allowed')


def worker(interface, anchor, lower, upper, tuning, requests, stop, heartbeat, sender, directory):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    parameters, rules, limits = tuning
    c = FiniteController(anchor, lower, upper, parameters, rules, limits)
    channel = None
    last_publish = 0.
    block = None
    block_started = float('inf')
    old_release = old_bottom = old_closed = None
    event_file = (Path(directory)/'strikes.jsonl').open('w', buffering=1)
    sample_file = (Path(directory)/'j7.csv').open('w', buffering=65536)
    samples = csv.writer(sample_file)
    samples.writerow(['monotonic_s', 'sample_at', 'block', 'phase', 'position_rad',
                      'velocity_rad_s', 'torque_nm', 'depth_deg', 'count'])

    def publish(s, force=False):
        nonlocal last_publish
        now = time.monotonic()
        if not force and now-last_publish < .005:
            return
        try:
            sender.send(pickle.dumps(s))
        except BlockingIOError:
            pass
        last_publish = now

    def event(kind, **data):
        event_file.write(json.dumps(dict(kind=kind, block=block, t=time.monotonic(), **data))+'\n')

    try:
        channel = Joint7Channel(interface)
        c.bias = prepare_powered(channel, anchor, stop, publish)
        event('powered_mode_transfer', mode=channel.mode, no_disable=True)
        last_send, last_torque = time.monotonic(), c.bias
        deadline = last_send
        while not stop.is_set():
            channel.receive()
            now = time.monotonic()
            if now < deadline:
                channel.wait(deadline-now); continue
            if now-heartbeat.value > .5:
                raise RuntimeError('Collector heartbeat lost')
            if now-last_send > (limits.control_timeout if c.method else .25):
                raise RuntimeError('Hybrid control deadline missed')
            try:
                request_id, action, *args = requests.get_nowait()
            except queue.Empty:
                action = None
            if action:
                if action == 'block':
                    depth, count, block = args
                    block_started = now
                    old_release = old_bottom = old_closed = None
                    c.request_block(depth, count)
                    event('block_requested', depth_deg=math.degrees(depth), strikes=count)
                elif action == 'finish':
                    c.stop_swing()
                else:
                    raise ValueError('Unknown collection request')
                c.status = replace(c.status, request_id=request_id)
            sample = channel.sample
            command = c.update(sample, now)
            before = time.monotonic()
            if c.method and (before-sample.at > limits.feedback_timeout or before-last_send > limits.control_timeout):
                raise RuntimeError('Hybrid tick expired before command')
            command, torque, _ = limit_command(command, sample, before, anchor, limits, last_send, last_torque)
            if not lower <= command.position <= upper:
                raise RuntimeError('Hybrid command left collection corridor')
            if stop.is_set():
                break
            channel.send(command.frame())
            last_send, last_torque = time.monotonic(), torque
            s = c.status
            samples.writerow([now, sample.at, block, s.phase, sample.position,
                              sample.velocity, sample.torque, math.degrees(anchor-sample.position), s.count])
            if s.released_at is not None and s.released_at >= block_started and s.released_at != old_release:
                old_release = s.released_at
                event('release', count=s.count, released_at=s.released_at,
                      depth_deg=math.degrees(c.amount), partial_returns=s.partial_returns)
            if s.bottom_at is not None and s.bottom_at >= block_started and s.bottom_at != old_bottom:
                old_bottom = s.bottom_at
                event('bottom', count=s.count, bottom_at=s.bottom_at, peak_deg=math.degrees(s.peak_drop))
            if s.closed_count and s.closed_count != old_closed:
                old_closed = s.closed_count
                event('closed', count=s.closed_count, peak_deg=math.degrees(s.closed_peak))
            publish(s)
            priming = c.search_pending is not None or c.swing_pending is not None
            period = 1/(limits.hz if c.method or s.swing or priming else 100.)
            deadline = max(deadline+period, last_send+period if deadline+period <= last_send else 0.)
    except InterruptedError:
        pass
    except BaseException as exc:
        event('fault', error=str(exc))
        publish(replace(c.status, error=str(exc), phase='fault', ready=False, at=time.monotonic()), True)
        if channel and channel.sample and channel.mode == 0:
            try:
                q = max(lower, min(upper, channel.sample.position))
                channel.send(Command(q, 0., 40., 1.8, max(-2., min(2., c.bias))).frame())
            except Exception:
                pass
    finally:
        if channel:
            channel.close()
        event_file.close(); sample_file.close(); sender.close()


class CollectionSession(HybridSession):
    def __init__(self, bus, anchor, lower, upper, tuning, directory):
        if os.environ.get('STRIKE_LAB_OFFLINE_ONLY') == '1':
            raise RuntimeError('Hardware forbidden in offline collection checks')
        if not bus.active or bus.control_side != 'right' or bus.right_joint7_session is not None:
            raise RuntimeError('Collection requires an active unowned right J7')
        self.controller = FiniteController(anchor, lower, upper, *tuning)
        self.bus, self.lower, self.upper = bus, lower, upper
        self.session_attr = 'right_joint7_session'
        self.closed, self.request_id = False, 0
        context = mp.get_context('spawn')
        self.requests = context.Queue(maxsize=4)
        self.stop_event = context.Event()
        self.heartbeat = context.Value('d', time.monotonic())
        self.receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.receiver.setblocking(False); sender.setblocking(False)
        self._status = HybridStatus()
        self.process = context.Process(target=worker, name='x5-finite-hybrid', args=(
            bus.sockets['right'].getsockname()[0], anchor, lower, upper, tuning,
            self.requests, self.stop_event, self.heartbeat, sender, str(directory)))
        bus.right_joint7_session = self
        try:
            self.process.start()
        except Exception:
            bus.right_joint7_session = None
            self.receiver.close(); self.requests.close()
            raise
        finally:
            sender.close()

    def block(self, depth, count, number):
        self.controller.validate_depth(math.radians(depth))
        return self.request('block', math.radians(depth), count, number)
