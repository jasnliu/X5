"""Automatic background hi-hat calibration; never owns or commands arm motors."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

from .hihat_calibration import HiHatCalibration

STARTUP_TIMEOUT = 90.0


class HiHatCalibrationRuntime:
    def __init__(self, app, *, now=None, directory=None):
        self.app = app
        self.started_at = time.monotonic() if now is None else now
        self.controller_ready_at = None
        self.next_query = self.started_at
        self.failure_reported = False
        self.wait_detail = 'waiting for ESP32 and hi-hat detector'
        self.stream = None
        self.closed = False
        self.directory = directory or (Path(__file__).resolve().parents[1] /
            'playback_results' / 'hihat_calibration' /
            (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:8]))
        try:
            Path(self.directory).mkdir(parents=True, exist_ok=True)
            self.stream = (Path(self.directory) / 'events.jsonl').open('w', buffering=1)
        except OSError as exc:
            print('HIHAT CALIBRATION: diagnostic file unavailable: ' + str(exc), flush=True)
        self.engine = HiHatCalibration(app.hihat, self.log)

    @property
    def ready(self):
        return self.engine.ready

    @property
    def busy(self):
        return self.engine.busy

    @property
    def detail(self):
        e = self.engine
        if e.failure:
            return 'FAULT: ' + e.failure
        if e.ready:
            return f'CALIBRATED {e.selected_angle}° / OPEN 0°'
        if e.phase == 'WAITING':
            return 'CALIBRATING: ' + self.wait_detail
        return f'CALIBRATING {e.angle if e.angle is not None else 90}°: {e.phase.lower()}'

    def log(self, row):
        print('HIHAT CALIBRATION: ' + json.dumps(row, allow_nan=False), flush=True)
        if self.stream is not None:
            try:
                self.stream.write(json.dumps(row, allow_nan=False) + '\n')
            except OSError:
                self.stream.close()
                self.stream = None

    def feed(self, messages):
        self.engine.feed(messages, time.monotonic())

    def tick(self, now=None):
        if self.closed:
            return
        now = time.monotonic() if now is None else now
        c, e = self.app.hihat, self.engine
        try:
            if e.phase == 'FAULT':
                self._report_failure()
                return
            if c.state == 'error':
                raise RuntimeError(c.detail)
            if c.ready():
                if self.controller_ready_at is None:
                    self.controller_ready_at = now
                if now >= self.next_query:
                    c.query_status()
                    self.next_query = now + (.5 if e.ready else .1)
                if c.telemetry is None and now - self.controller_ready_at > 3:
                    raise RuntimeError('ESP32 calibration v2 status unavailable; install the updated X5 hi-hat firmware')
            monitor = getattr(self.app, 'hihat_sound_monitor', None)
            receiver = getattr(monitor, 'receiver', None)
            audio_ready = receiver is not None and receiver.ready(now)
            if e.phase == 'WAITING':
                if now - self.started_at >= STARTUP_TIMEOUT:
                    raise RuntimeError('Hi-hat calibration startup timed out waiting for firmware or detector')
                if receiver is None or receiver.state in {'error', 'stopped'}:
                    raise RuntimeError('Hi-hat detector unavailable for startup calibration')
                if not (c.ready() and c.telemetry is not None and audio_ready):
                    return
                # User-requested automatic startup: no alert, background
                # notifier, countdown, or extra delay once devices are ready.
                e.start(now)
            if e.ready:
                if c.telemetry_at is None or now - c.telemetry_at > 1.5:
                    raise RuntimeError('Calibrated ESP32 status became stale')
                if c.telemetry['angle'] != e.selected_angle:
                    raise RuntimeError('ESP32 calibrated target changed unexpectedly')
            else:
                e.tick(now, audio_ready=audio_ready)
        except Exception as exc:
            e.abort(str(exc), now)
            # Service a pending fault return even when the detector is gone.
            if e.busy:
                e.tick(now, audio_ready=False)
        self._report_failure()

    def _report_failure(self):
        if self.engine.failure and not self.failure_reported:
            self.failure_reported = True
            a = self.app
            a.hihat_fault_detail = self.engine.failure
            message = 'HI-HAT CALIBRATION FAULT: ' + self.engine.failure
            a.result_status.set(message)
            a.result_label.config(fg='#b00020')
            # Existing arm failure path performs center-before-relax. An idle
            # arm remains disabled; calibrator itself never accesses its bus.
            if a.bus and a.bus.active:
                a.fail(message)
            else:
                a.status.set(message)

    def cancel(self, reason='Calibration cancelled; restart the program'):
        if not self.ready and self.engine.failure is None:
            self.engine.abort(reason, time.monotonic())
            self._report_failure()

    def close(self):
        self.closed = True
        if self.stream is not None:
            self.stream.close()
            self.stream = None
