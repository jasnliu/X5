"""Playback-local CAN backpressure handling and optional no-strike relax gate."""
from collections import deque
import errno
import json
import math
import time

import numpy as np

from centering.motors import Motors
from safe_zone.encoder import FRAME


class RetrySocket:
    """Retry transient queue pressure, never discard a motor command.

    In particular Motors.poll()'s state requests previously bypassed its command
    retry loop. With a ten-frame kernel TX queue, the GUI's query burst could
    collide with the independent 200 Hz playback writer and raise ENOBUFS.
    """
    def __init__(self, sock):
        self.sock = sock
        self.retries = 0
        self.maximum_wait_s = 0.

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def send(self, frame):
        start = time.monotonic()
        while True:
            try:
                sent = self.sock.send(frame)
                if sent != len(frame):
                    raise RuntimeError('Short CAN write')
                self.maximum_wait_s = max(self.maximum_wait_s, time.monotonic()-start)
                return sent
            except OSError as exc:
                if exc.errno not in (errno.ENOBUFS, errno.EAGAIN) or time.monotonic()-start >= .020:
                    raise
                self.retries += 1
                time.sleep(.0005)


class PlaybackMotors(Motors):
    @classmethod
    def adopt(cls, bus, center, strict_center_relax=False, directory=None):
        """Transfer the GUI's already-open bus; do not open/enable new hardware."""
        result = cls.__new__(cls)
        result.__dict__.update(bus.__dict__)
        result.sockets = {side: RetrySocket(sock) for side, sock in bus.sockets.items()}
        bus.sockets = {}
        bus.locks = []
        result.playback_session = None
        result.strict_center_relax = strict_center_relax
        result.center_goal = np.asarray(center).copy()
        result.center_history = deque()
        result.audit_directory = directory
        result.last_center_evidence = None
        return result

    def poll(self):
        session = self.playback_session
        if session is not None and session.owns_feedback:
            # The child's independent sockets supply fresh feedback to BOTH
            # receivers. Drain ours normally, but don't duplicate its queries.
            self.last_query = time.monotonic()
        super().poll()
        if self.strict_center_relax and self.fresh():
            now = time.monotonic()
            q = np.array([self.states['right', i][0] for i in range(1, 8)])
            self.center_history.append((now, q))
            while self.center_history and now-self.center_history[0][0] > .65:
                self.center_history.popleft()

    def center_evidence(self):
        if not self.fresh() or len(self.center_history) < 2:
            raise RuntimeError('REFUSED relax: fresh settled center is not verified')
        now = time.monotonic()
        history = self.center_history
        values = np.array([q for _, q in history])
        error = float(np.max(np.abs(values-self.center_goal)))
        span = float(np.max(np.ptp(values, axis=0)))
        current = np.array([self.states['right', i][0] for i in range(1, 8)])
        if (now-history[-1][0] > .1 or history[-1][0]-history[0][0] < .6
                or error > math.radians(.20) or span > math.radians(.12)
                or np.max(np.abs(current-self.center_goal)) > math.radians(.20)):
            raise RuntimeError('REFUSED relax: arm has not settled at center for 0.6 seconds')
        return dict(t=now, goal_rad=self.center_goal.tolist(), actual_rad=current.tolist(),
                    max_error_deg=math.degrees(error), span_deg=math.degrees(span),
                    settled_s=history[-1][0]-history[0][0])

    def _send(self, side, frame):
        cid, _, data = FRAME.unpack(frame)
        if self.strict_center_relax and ((cid >> 24) & 31 == 1 or
                ((cid >> 24) & 31 == 18 and data[:2] == b'\x05\x70' and data[4] != 5)):
            raise RuntimeError('Recording-only mode forbids MIT/strike mode commands')
        if self.strict_center_relax and (cid >> 24) & 31 == 4:
            state = self.states.get((side, cid & 255))
            # Initial CSP setup may send disable to an ALREADY relaxed motor.
            if state is None or state[1] != 0:
                if self.playback_session is not None:
                    raise RuntimeError('REFUSED relax: playback still owns the arm')
                self.last_center_evidence = self.center_evidence()
                if self.audit_directory:
                    with open(self.audit_directory/'disable_audit.jsonl', 'a') as out:
                        out.write(json.dumps(dict(motor=cid & 255, **self.last_center_evidence))+'\n')
        return super()._send(side, frame)

    def relax(self):
        if self.strict_center_relax and self.active:
            if self.playback_session is not None:
                raise RuntimeError('REFUSED relax: playback still owns the arm')
            self.last_center_evidence = self.center_evidence()
        return super().relax()

    def close(self):
        if self.audit_directory:
            (self.audit_directory/'transport.json').write_text(json.dumps({
                side: dict(retries=s.retries, maximum_wait_s=s.maximum_wait_s)
                for side, s in self.sockets.items()}, indent=2)+'\n')
        super().close()
