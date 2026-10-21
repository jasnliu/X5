"""Bounded rolling evidence for hands-free, stationary-pose capture.

No devices, motor commands, fitting, or disk writes live in this module.
This is NOT moving-arm timestamp synchronization.
"""
from collections import deque

from .observer import stationary_capture

AUTO_POLICY = 'validation_first_auto_v1'
WINDOW_SECONDS = .7
MIN_STABLE_SECONDS = .5
ENCODER_PADDING_SECONDS = .1
MAX_FRAME_AGE_SECONDS = .3


class AutomaticCapture:
    def __init__(self):
        self.active = False
        # Bound RGB memory independently of the number of saved poses.
        self.frames = deque(maxlen=24)
        self.reason = 'Automatic capture not started'

    def start(self):
        self.clear()
        self.active = True
        self.reason = 'Waiting for a visible board and a brief steady pose'

    def stop(self):
        self.active = False
        self.clear()
        self.reason = 'Stopped — saved poses retained'

    def clear(self):
        self.frames.clear()

    def add(self, frame):
        if not self.active:
            return
        stamp = frame['header']['received_monotonic']
        if self.frames:
            last = self.frames[-1]['header']
            if (stamp <= last['received_monotonic'] or
                    frame['header']['sequence'] <= last['sequence']):
                self.clear()
                self.reason = 'Waiting for new, ordered camera frames'
                return
        self.frames.append(frame)
        while self.frames and stamp-self.frames[0]['header']['received_monotonic'] > WINDOW_SECONDS:
            self.frames.popleft()

    def candidate(self, joints, now):
        if not self.active or not self.frames:
            return None
        end = self.frames[-1]['header']['received_monotonic']
        if not 0 <= now-end <= MAX_FRAME_AGE_SECONDS:
            self.clear()
            self.reason = 'Waiting for fresh camera frames'
            return None
        try:
            result = stationary_capture(
                list(self.frames), joints, end-WINDOW_SECONDS, end,
                min_duration=MIN_STABLE_SECONDS, encoder_padding=ENCODER_PADDING_SECONDS)
        except ValueError as exc:
            self.reason = str(exc)
            return None
        result[2]['capture_mode'] = AUTO_POLICY
        result[2]['required_min_duration_s'] = MIN_STABLE_SECONDS
        self.reason = 'Steady synchronized-by-stationarity observation available'
        return result
