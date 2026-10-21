"""Actual Tk preview endurance check; strictly no CAN or robot observer allowed."""
import argparse
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path('/home/jason/Proyectos3/X5')
sys.path.insert(0, str(ROOT))
can_attempts = []
real_socket = socket.socket

def guarded_socket(family=socket.AF_INET, *args, **kwargs):
    if family == socket.PF_CAN:
        can_attempts.append(dict(family=family, monotonic=time.monotonic()))
        raise AssertionError('Live preview test must NEVER open CAN')
    return real_socket(family, *args, **kwargs)
socket.socket = guarded_socket

from camera_calibration import app as module
from camera_calibration import io as io_module
io_module.build_bridge = lambda: Path(__file__).resolve().parent / "orbbec_capture"

def forbidden_observer(*args, **kwargs):
    raise AssertionError('Live preview test must NEVER construct an arm observer')
module.Observer = forbidden_observer

import cv2
import numpy as np
class RawYuyvDecoder:
    def decode(self, data):
        if len(data) != 1280*720*2:
            raise RuntimeError('Wrong YUYV frame size: '+str(len(data)))
        return cv2.cvtColor(np.frombuffer(data, dtype=np.uint8).reshape(720,1280,2), cv2.COLOR_YUV2BGR_YUY2)
io_module.JpegDecoder = RawYuyvDecoder
folder = Path(__file__).resolve().parent
args = argparse.Namespace(command='preview', output=str(folder/'live_idle_sessions'),
                          serial='AY6R463004K', fps=15)
app = module.App(args)
started = time.monotonic()
first_display = None
first_sequence = None
last_sequence = None
last_display = None
displayed_frames = 0
maximum_display_gap = 0.
paused = []
was_paused = False
closed = False
summary = {}
samples = (folder/'live_idle_samples.jsonl').open('w')

def finish(reason):
    global closed, summary
    if closed:
        return
    closed = True
    now = time.monotonic()
    camera = app.camera
    elapsed = now-first_display if first_display is not None else 0.
    summary = dict(success=(reason == 'completed' and elapsed >= 20 and app.failure is None
                            and not can_attempts and app.observer is None and not paused),
                   reason=reason, failure=app.failure, wall_duration_s=now-started,
                   live_display_duration_s=elapsed, first_sequence=first_sequence,
                   final_displayed_sequence=app.sequence, displayed_frames=displayed_frames,
                   maximum_display_gap_s=maximum_display_gap, pauses=paused,
                   quality_text=app.quality.get(), status_text=app.status.get(),
                   can_attempts=can_attempts, observer_absent=app.observer is None,
                   metadata=camera.metadata if camera else None,
                   session=str(app.session.path))
    app.close()
    summary['camera_released'] = camera is not None and camera.process is not None and camera.process.poll() is not None
    summary['jpeg_report'] = camera.jpeg_report if camera else None
    summary['success'] = summary['success'] and summary['camera_released']
    (folder/'live_idle_report.json').write_text(json.dumps(summary, indent=2)+'\n')
    samples.close()
    print(json.dumps({k:v for k,v in summary.items() if k not in ('metadata',)}, indent=2), flush=True)

def poll():
    global first_display, first_sequence, last_sequence, last_display, displayed_frames, maximum_display_gap, was_paused
    if closed:
        return
    now = time.monotonic()
    camera = app.camera
    latest = camera.latest if camera else None
    samples.write(json.dumps(dict(monotonic=now, displayed_sequence=app.sequence,
                                  latest_sequence=latest[0]['sequence'] if latest else None,
                                  latest_age_s=now-latest[0]['received_monotonic'] if latest else None,
                                  camera_waiting=app.camera_waiting,
                                  rejected=camera.jpeg_report['rejected'] if camera else None,
                                  error=app.failure or (camera.error if camera else None)))+'\n')
    if app.failure or (camera and camera.error):
        finish('application_error'); return
    if app.camera_waiting and not was_paused:
        paused.append(dict(monotonic=now, status=app.status.get()))
    was_paused = app.camera_waiting
    if app.sequence and app.sequence != last_sequence:
        if first_display is None:
            first_display=now; first_sequence=app.sequence
        if last_display is not None:
            maximum_display_gap=max(maximum_display_gap, now-last_display)
        last_display=now; last_sequence=app.sequence; displayed_frames+=1
    if first_display is not None and now-first_display >= 20:
        finish('completed'); return
    if now-started > 155:
        finish('test_timeout'); return
    app.root.after(50, poll)

app.root.protocol('WM_DELETE_WINDOW', lambda: finish('window_closed_early'))
app.root.after(50, app.start)
app.root.after(100, poll)
app.run()
if not summary:
    raise SystemExit('Preview closed before the endurance check completed')
raise SystemExit(0 if summary['success'] else 1)
