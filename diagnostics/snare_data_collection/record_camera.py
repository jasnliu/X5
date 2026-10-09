"""Passive camera evidence only; never imports arm or CAN control."""
import csv
from pathlib import Path
import signal
import sys
import time

import cv2

directory = Path(sys.argv[1])
stopping = False


def stop(*_):
    global stopping
    stopping = True


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
cap = cv2.VideoCapture('/dev/video4', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
for _ in range(8):
    ok, frame = cap.read()
if not ok or frame.mean() < 5:
    raise RuntimeError('RGB camera unavailable')
writer = cv2.VideoWriter(str(directory/'physical_video.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                         30., (frame.shape[1], frame.shape[0]))
if not writer.isOpened():
    raise RuntimeError('Video writer unavailable')
with (directory/'video_frames.csv').open('w') as out:
    rows = csv.writer(out)
    rows.writerow(['frame', 'monotonic_s', 'wall_s'])
    (directory/'camera_ready').write_text('RGB capture ready\n')
    index = 0
    while not stopping:
        ok, frame = cap.read()
        if not ok:
            if stopping:
                break  # SIGTERM may interrupt the blocking camera read.
            raise RuntimeError('Camera stopped')
        writer.write(frame)
        rows.writerow([index, time.monotonic(), time.time()])
        index += 1
writer.release()
cap.release()
print('Video frames saved:', index, flush=True)
