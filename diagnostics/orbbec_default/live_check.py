"""Camera-only live integration check; forbidden to open a CAN socket.

Run with the Y2 virtualenv and argument 'search' or 'playback'. This runs the
actual camera, preprocessing, model, display, and recording path. Only the quit
key is automated; no arm-control app or ROS hardware launcher is started.
"""
import importlib
import json
from pathlib import Path
import socket
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def audit(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.PF_CAN:
        raise RuntimeError('Camera-only check forbids physical CAN')


sys.addaudithook(audit)
import cv2
from camera_search.protocol import DetectionReceiver

mode = sys.argv[1]
assert mode in ('search', 'playback')
base = Path(__file__).resolve().parent
receiver = DetectionReceiver('/tmp/x5-camera-only-'+uuid.uuid4().hex[:8]+'.sock')
module = importlib.import_module('camera_'+mode+'.camera')
messages = []
started = None
display_count = 0
last_preview = None
old_wait = cv2.waitKey
old_show = cv2.imshow


def show(name, frame):
    global last_preview, display_count
    old_show(name, frame)
    display_count += 1
    last_preview = frame.copy()


def wait(delay):
    global started
    if started is None:
        started = time.monotonic()
    messages.extend(receiver.poll())
    key = old_wait(delay)
    return ord('q') if time.monotonic()-started >= 10 else key


cv2.waitKey, cv2.imshow = wait, show
sys.argv = [module.__file__, '--socket', str(receiver.path),
            '--output', str(base/f'{mode}.mp4')]
try:
    rc = module.main()
    messages.extend(receiver.poll())
    assert rc == 0, rc
    assert not any(m.get('state') == 'error' for m in messages)
    frames = [m for m in messages if m.get('kind') == 'frame']
    assert len(frames) >= 10 and display_count >= 60
    assert last_preview is not None
    cv2.imwrite(str(base/f'{mode}_preview.jpg'), last_preview)
    result = dict(mode=mode, exit_code=rc, physical_camera=True, can_forbidden=True,
                  default_camera_used=True, display_frames=display_count,
                  processed_frame_messages=len(frames),
                  elapsed_display_s=time.monotonic()-started,
                  image_shape=list(last_preview.shape),
                  video_bytes=(base/f'{mode}.mp4').stat().st_size)
    (base/f'{mode}_live.json').write_text(json.dumps(result, indent=2)+'\n')
    (base/f'{mode}_messages.json').write_text(json.dumps(messages, indent=2)+'\n')
    print('CAMERA-ONLY LIVE PASS: '+json.dumps(result), flush=True)
finally:
    receiver.close()
    cv2.waitKey, cv2.imshow = old_wait, old_show
