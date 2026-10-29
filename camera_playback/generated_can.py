"""Generated-mode CAN scheduling only; no device creation or motor settings."""
import errno
import time
from safe_zone.encoder import request_frame


class TransmitError(RuntimeError):
    """A failed send is not evidence that received encoders are stale."""


class QueuePressure:
    # Same 20 ms budget as existing playback RetrySocket / MIT control timeout.
    # No sleep/retry of an old frame here: the worker receives and recomputes.
    def __init__(self):
        self.blocked_at = None
        self.retries = 0

    def send(self, send, frame, now):
        if self.blocked_at is not None and now-self.blocked_at >= .020:
            raise TransmitError('CAN transmit queue blocked beyond 20 ms Hold/return deadline')
        try:
            send(frame)
        except OSError as exc:
            if exc.errno not in (errno.ENOBUFS, errno.EAGAIN):
                raise TransmitError(f'CAN transmit failed: {exc}') from exc
            self.retries += 1
            if self.blocked_at is None:self.blocked_at = now
            return False
        self.blocked_at = None
        return True


def receive_without_query_burst(bus, receive):
    """Retain the existing decoder/fault checks; generated scheduler owns queries.

    Motors.poll has independent normal and priority-J7 query timers. Disable
    only those for this call, not feedback, active state or motor ownership.
    """
    priority = bus.right_joint7_query_period
    bus.right_joint7_query_period = None
    bus.last_query = time.monotonic()
    try:return receive()
    finally:bus.right_joint7_query_period = priority


class FeedbackQueries:
    """Bounded state requests per bus/tick, oldest received feedback first.

    The 200 Hz child uses two requests. The GUI retains a complete eight-drive
    sweep: Tk after(20) adds callback time and is NOT a 50 Hz clock. Dividing
    that sweep across callbacks can exceed 120 ms during setup or handoff.
    Recent replies still suppress redundant queries. Freshness is unchanged.
    """
    def __init__(self, period, selected=None, *, max_requests=2):
        self.period, self.selected = period, selected
        if not 1<=max_requests<=8:raise ValueError('Generated query batch must fit the eight drives')
        self.max_requests = max_requests
        self.last_tick = -float('inf')
        self.sent = {}

    def poll(self, bus):
        now = time.monotonic()
        if now-self.last_tick < self.period-1e-6:return
        self.last_tick = now
        for side, sock in bus.sockets.items():
            age = .010 if side == self.selected else .040
            due = [i for i in range(1,9)
                   if now-bus.states.get((side,i),(0,0,-float('inf')))[2] >= age
                   and now-self.sent.get((side,i),-float('inf')) >= age]
            # Use RECEIVED age first. Our own last-query order is obsolete
            # after another worker has supplied feedback during a trajectory.
            due.sort(key=lambda i:(bus.states.get((side,i),(0,0,-float('inf')))[2],
                                   self.sent.get((side,i),-float('inf')),i))
            for i in due[:self.max_requests]:
                try:
                    if sock.send(request_frame(i)) != 16:
                        raise TransmitError(f'{side} J{i} encoder query: short CAN transmit')
                except OSError as exc:
                    raise TransmitError(f'{side} J{i} encoder query CAN transmit failed: {exc}') from exc
                self.sent[side,i] = now
