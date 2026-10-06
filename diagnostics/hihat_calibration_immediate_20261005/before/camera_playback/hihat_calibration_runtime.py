"""Automatic background hi-hat calibration; never owns or commands arm motors."""
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import queue
import threading
import time
import uuid

from .hihat_calibration import HiHatCalibration

WARNING_SECONDS = 10.0
STARTUP_TIMEOUT = 90.0


def notify_calibration():
    path = Path.home() / '.local/bin/codex-ntfy.py'
    spec = importlib.util.spec_from_file_location('hihat_calibration_ntfy', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not module.post_ntfy(
            'Automatic hi-hat calibration starts in 10 seconds: 90 to at most 115 degrees, '
            '5-degree steps, 2 seconds closed and 2 seconds open. Keep clear. '
            'The robot arm moves only if RUN is pressed.',
            'X5 hi-hat calibration: movement warning', 'warning,robot'):
        raise RuntimeError('ntfy delivery failed; hi-hat motion is blocked')
    module.play_sound('{"type":"agent-turn-complete"}')


class HiHatCalibrationRuntime:
    def __init__(self, app, *, notifier=notify_calibration, now=None, directory=None):
        self.app = app
        self.notifier = notifier
        self.started_at = time.monotonic() if now is None else now
        self.controller_ready_at = None
        self.next_query = self.started_at
        self.warning_until = None
        self.notification = queue.Queue(maxsize=1)
        self.notification_started = False
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

    def _notify(self):
        # No GUI, serial, or arm access from this thread. It cannot initiate
        # motion after cancellation; only tick() can consume its result.
        try:
            self.notifier()
            result = (True, '')
        except Exception as exc:
            result = (False, str(exc))
        self.notification.put(result)

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
                    raise RuntimeError('Hi-hat calibration startup timed out waiting for firmware, detector or notification')
                if receiver is None or receiver.state in {'error', 'stopped'}:
                    raise RuntimeError('Hi-hat detector unavailable for startup calibration')
                if not (c.ready() and c.telemetry is not None and audio_ready):
                    return
                if not self.notification_started:
                    self.notification_started = True
                    self.wait_detail = 'sending ntfy movement warning'
                    threading.Thread(target=self._notify, daemon=True, name='hihat-calibration-ntfy').start()
                if self.warning_until is None:
                    try:
                        ok, detail = self.notification.get_nowait()
                    except queue.Empty:
                        return
                    if not ok:
                        raise RuntimeError(detail)
                    self.warning_until = now + WARNING_SECONDS
                    self.log(dict(kind='notification_delivered', at=now, warning_seconds=WARNING_SECONDS))
                self.wait_detail = f'motion warning: {max(0, self.warning_until-now):.1f} s'
                if now >= self.warning_until:
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
