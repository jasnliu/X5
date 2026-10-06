"""Camera-only capture of every advertised color MJPEG size, no CAN access."""
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def audit(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.PF_CAN:
        raise RuntimeError('FOV tests forbid physical CAN')


sys.addaudithook(audit)
import cv2
from camera_search.device import resolve_camera

base = Path(__file__).resolve().parent
index = resolve_camera()
results = []
for w, h in [(640, 480), (640, 360), (1280, 720), (1920, 1080)]:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    try:
        assert cap.isOpened()
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, 30)
        assert cap.get(cv2.CAP_PROP_FRAME_WIDTH) == w
        assert cap.get(cv2.CAP_PROP_FRAME_HEIGHT) == h
        for _ in range(30):
            ok, frame = cap.read()
            assert ok and frame.shape == (h, w, 3)
        count, start = 0, time.monotonic()
        while time.monotonic()-start < 3:
            ok, frame = cap.read()
            assert ok and frame.shape == (h, w, 3)
            count += 1
        elapsed = time.monotonic()-start
        cv2.imwrite(str(base/f'{w}x{h}.jpg'), frame)
        result = dict(width=w, height=h, frames=count, elapsed_s=elapsed,
                      fps=count/elapsed, reported_fps=cap.get(cv2.CAP_PROP_FPS))
        results.append(result)
        print(result, flush=True)
    finally:
        cap.release()
(base/'mode_comparison.json').write_text(json.dumps(results, indent=2)+'\n')

# Match the unchanged static scene into full-resolution 16:9. This measures
# relative scene coverage, not calibrated absolute lens angles.
reference = cv2.imread(str(base/'1920x1080.jpg'))
sift = cv2.SIFT_create(nfeatures=8000)
ref_points, ref_descriptors = sift.detectAndCompute(reference, None)
coverage = []
import numpy as np
for w, h in [(640, 480), (640, 360), (1280, 720)]:
    frame = cv2.imread(str(base/f'{w}x{h}.jpg'))
    points, descriptors = sift.detectAndCompute(frame, None)
    matches = cv2.BFMatcher().knnMatch(descriptors, ref_descriptors, k=2)
    good = [a for a, b in matches if a.distance < .7*b.distance]
    source = np.float32([points[m.queryIdx].pt for m in good])
    target = np.float32([ref_points[m.trainIdx].pt for m in good])
    matrix, mask = cv2.estimateAffinePartial2D(source, target, method=cv2.RANSAC,
                                             ransacReprojThreshold=4.)
    assert matrix is not None and int(mask.sum()) > 40
    corners = np.float32([[[0, 0], [w, 0], [w, h], [0, h]]])
    mapped = cv2.transform(corners, matrix)[0]
    coverage.append(dict(size=[w, h], inliers=int(mask.sum()),
                         transform=matrix.tolist(),
                         corners_in_1920x1080=mapped.tolist(),
                         relative_width=float(np.ptp(mapped[:, 0])/1920),
                         relative_height=float(np.ptp(mapped[:, 1])/1080)))
(base/'relative_coverage.json').write_text(json.dumps(coverage, indent=2)+'\n')
print(json.dumps(coverage, indent=2))
