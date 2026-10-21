#!/usr/bin/env python3
"""Y2 live pipeline with one processed window and local detection status output."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

X5_ROOT = Path(__file__).resolve().parents[1]
if str(X5_ROOT) not in sys.path:
    sys.path.insert(0, str(X5_ROOT))

from camera_search.protocol import DetectionSender, has_required_detection
from camera_search.vision import cymbal_grid_geometry
from camera_search.device import add_camera_argument, open_camera


CYMBAL_GRID_COLOR = (255, 255, 255)
CYMBAL_TARGET_COLOR = (255, 0, 255)
CYMBAL_TARGET_ALPHA = 0.28


def draw_cymbal_target(preview, detections, cv2):
    """Overlay equal thirds and highlight the center cell; detections stay unchanged."""
    geometry = cymbal_grid_geometry(detections)
    if geometry is None:
        return preview
    height, width = preview.shape[:2]
    left, top, right, bottom = geometry["outer"]
    left = max(0, min(width - 1, left))
    right = max(0, min(width - 1, right))
    top = max(0, min(height - 1, top))
    bottom = max(0, min(height - 1, bottom))
    x1, x2 = (max(left, min(right, x)) for x in geometry["x_lines"])
    y1, y2 = (max(top, min(bottom, y)) for y in geometry["y_lines"])
    if right <= left or bottom <= top or x2 <= x1 or y2 <= y1:
        return preview

    overlay = preview.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), CYMBAL_TARGET_COLOR, -1)
    cv2.addWeighted(overlay, CYMBAL_TARGET_ALPHA, preview,
                    1.0 - CYMBAL_TARGET_ALPHA, 0.0, preview)

    # A dark outline plus a bright line keeps the thirds visible against both
    # dark cymbal edges and highlights.
    for x in (x1, x2):
        cv2.line(preview, (x, top), (x, bottom), (0, 0, 0), 4, cv2.LINE_AA)
        cv2.line(preview, (x, top), (x, bottom), CYMBAL_GRID_COLOR, 2, cv2.LINE_AA)
    for y in (y1, y2):
        cv2.line(preview, (left, y), (right, y), (0, 0, 0), 4, cv2.LINE_AA)
        cv2.line(preview, (left, y), (right, y), CYMBAL_GRID_COLOR, 2, cv2.LINE_AA)
    cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 0, 0), 5, cv2.LINE_AA)
    cv2.rectangle(preview, (x1, y1), (x2, y2), CYMBAL_TARGET_COLOR, 3, cv2.LINE_AA)
    label_y = max(22, y1 - 8)
    cv2.putText(preview, "CYMBAL TARGET", (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                .55, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(preview, "CYMBAL TARGET", (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                .55, CYMBAL_TARGET_COLOR, 2, cv2.LINE_AA)
    return preview


def _parser(default_root: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Processed Y2 camera for the X5 right-arm search")
    parser.add_argument("--socket", required=True)
    parser.add_argument("--root", type=Path, default=default_root)
    parser.add_argument("--lighting-profile", type=Path, default=None)
    parser.add_argument("--preprocessing", choices=("local", "reference", "none"), default=None)
    parser.add_argument("--no-normalization", action="store_true")
    add_camera_argument(parser)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--imgsz", type=int, default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=None,
                        help="RGB width (Orbbec full-FOV default: 1280; omitted partner dimension inferred as 16:9)")
    parser.add_argument("--height", type=int, default=None,
                        help="RGB height (Orbbec full-FOV default: 720; cropped modes are rejected)")
    parser.add_argument("--allow-low-light-fps", action="store_true")
    parser.add_argument("--process-every", type=int, default=1)
    parser.add_argument("--output", type=Path, default=None)
    return parser


def main() -> int:
    default_root = Path(__file__).resolve().parents[2] / "Y2"
    args = _parser(default_root).parse_args()
    if args.process_every < 1:
        raise SystemExit("--process-every must be at least 1")
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    sender = DetectionSender(args.socket)
    sender.status("starting", "Loading Y2 pose model")
    try:
        import cv2
        import realtime_detector as detector
        from camera_controls import set_auto_frame_rate
        from lighting_normalization import make_processor

        model_path = (args.model or detector.choose_default_model(root)).resolve()
        imgsz = detector.resolve_live_imgsz(args.imgsz, model_path)
        conf = detector.resolve_live_conf(args.conf)
        device = detector.resolve_device(args.device)
        mode = ("none" if args.no_normalization else
                (args.preprocessing or ("reference" if args.lighting_profile else "local")))
        normalizer = make_processor(mode, args.lighting_profile or root / "models" / "lighting_profile.json")
        model = detector.load_model(model_path)
        if getattr(model, "task", None) != "pose":
            raise RuntimeError("selected Y2 model is not a pose model and cannot provide the required tip")
        print(f"Preprocessing: {mode}", flush=True)
        print(f"Model: {model_path}; input: {imgsz}; required: drumstick + YOLO pose tip", flush=True)

        camera, camera_index = open_camera(args.camera, cv2=cv2,
                                          width=args.width, height=args.height)
        if sys.platform.startswith("linux"):
            try:
                settings = set_auto_frame_rate(camera_index, args.allow_low_light_fps)
                low_light = "enabled" if settings["current"] else "disabled"
                print(f"Camera: requested 30 FPS; automatic low-light FPS reduction {low_light}.", flush=True)
            except OSError as exc:
                print(f"Warning: could not set low-light frame-rate control: {exc}", flush=True)

        recording_path = args.output or detector.timestamped_output_path(root / "recordings")
        writer = None
        saver = detector.BadPicSaver(root / "bad pics")
        cv2.namedWindow(detector.WINDOW_NAME)
        cv2.setMouseCallback(detector.WINDOW_NAME, saver.on_mouse)
        worker = detector.InferenceWorker(model, imgsz, conf, device)
        worker.start()
        sender.status("ready", "Camera open; waiting for first inference")
        frame_count = 0
        displayed_at = time.perf_counter()
        display_frames = 0
        display_fps = 0.0
        inference_count = 0
        inference_started = time.perf_counter()
        previous_frame_id = None
        try:
            while True:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("camera frame read failed")
                captured_at = time.monotonic()
                processed = normalizer.apply(frame) if normalizer is not None else frame
                frame_count += 1
                display_frames += 1
                if frame_count % args.process_every == 0:
                    worker.submit(processed, raw_frame=frame, timestamp=captured_at)
                result = worker.snapshot_frame()
                worker_error = worker.error()
                if worker_error is not None:
                    raise RuntimeError(f"inference failed: {worker_error}")
                if result is not None and result.frame_id != previous_frame_id:
                    inference_count += 1
                    previous_frame_id = result.frame_id
                    sender.frame(result.frame_id, result.captured_at, result.detections)
                now = time.perf_counter()
                if now - displayed_at >= 1.0:
                    display_fps = display_frames / (now - displayed_at)
                    display_frames = 0
                    displayed_at = now
                inference_fps = inference_count / max(now - inference_started, 1e-9)
                display_frame, _display_raw, detections = detector.preview_source(
                    result, processed, frame, now=time.monotonic())
                preview = detector.draw_detections(display_frame, detections)
                draw_cymbal_target(preview, detections, cv2)
                cv2.putText(preview, f"Display FPS: {display_fps:.1f}", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .75, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(preview, f"Inference FPS: {inference_fps:.1f}", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, .75, (255, 255, 255), 2, cv2.LINE_AA)
                detected = has_required_detection(detections)
                label = "REQUIRED: DRUMSTICK + TIP" if detected else "Required: waiting for drumstick + tip"
                color = (0, 220, 0) if detected else (0, 165, 255)
                cv2.putText(preview, label, (15, 92), cv2.FONT_HERSHEY_SIMPLEX,
                            .65, color, 2, cv2.LINE_AA)
                button_box = detector.save_button_box(preview.shape)
                saver.update(display_frame, button_box)
                detector.draw_save_button(preview, button_box, saver.confirmation_active())
                if writer is None:
                    recording_path.parent.mkdir(parents=True, exist_ok=True)
                    height, width = preview.shape[:2]
                    writer = cv2.VideoWriter(
                        str(recording_path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (width, height)
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"could not open video writer: {recording_path}")
                writer.write(frame)
                cv2.imshow(detector.WINDOW_NAME, preview)
                if detector.is_exit_key(cv2.waitKey(1) & 0xFF):
                    break
        finally:
            worker.stop()
            camera.release()
            if writer is not None:
                writer.release()
                if recording_path.exists() and recording_path.stat().st_size > 0:
                    print(f"Saved recording to {recording_path}", flush=True)
            cv2.destroyAllWindows()
        sender.status("stopped", "Camera window closed")
        return 0
    except Exception as exc:
        sender.status("error", str(exc))
        print(f"Camera error: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())
