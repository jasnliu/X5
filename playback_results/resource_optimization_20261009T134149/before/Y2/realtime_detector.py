"""Responsive webcam detector for ride cymbals, snares, and drumsticks with tips."""

from __future__ import annotations

import argparse
import math
import threading
import time
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2

from lighting_normalization import make_processor
from camera_controls import set_auto_frame_rate


CLASS_NAMES = {0: "cymbal", 1: "drumstick", 2: "snare"}
CLASS_COLORS = {0: (0, 200, 255), 1: (0, 220, 0), 2: (255, 180, 0)}
DEFAULT_LIVE_IMGSZ = 416
LIVE_CHECKPOINT = Path("models/snare.pt")
LEGACY_POSE_CHECKPOINT = Path("models/live_pose_best.pt")
LIVE_CHECKPOINT_IMGSZ = 640
TIP_CONFIDENCE = 0.50
DEFAULT_LIVE_CONF = 0.50
HORIZONTAL_RECOVERY_MARGIN = 0.25
WINDOW_NAME = "Ride + Snare + Drumsticks Detector"
RAW_WINDOW_NAME = "Raw Camera - Before Normalization"
SAVE_BUTTON_LABEL = "Save Bad Pic"
SAVE_BUTTON_WIDTH = 170
SAVE_BUTTON_HEIGHT = 42
SAVE_BUTTON_MARGIN = 12
SAVE_CONFIRMATION_SECONDS = 1.5


def timestamped_output_path(directory: Path, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S_%f")
    return directory / f"{timestamp}.mp4"


def timestamped_snapshot_path(directory: Path, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S_%f")
    return directory / f"{timestamp}.png"


def save_bad_pic(frame, directory: Path, now: datetime | None = None) -> Path:
    """Save the normalized model-input frame as a lossless PNG."""
    directory.mkdir(parents=True, exist_ok=True)
    output = timestamped_snapshot_path(directory, now)
    if not cv2.imwrite(str(output), frame):
        raise RuntimeError(f"could not save bad picture: {output}")
    return output


def save_button_box(frame_shape) -> tuple[int, int, int, int]:
    """Return a top-right button rectangle that fits inside the frame."""
    frame_height, frame_width = frame_shape[:2]
    margin = min(SAVE_BUTTON_MARGIN, max(1, frame_width // 10), max(1, frame_height // 10))
    width = min(SAVE_BUTTON_WIDTH, max(1, frame_width - 2 * margin))
    height = min(SAVE_BUTTON_HEIGHT, max(1, frame_height - 2 * margin))
    right = frame_width - margin
    left = right - width
    top = margin
    bottom = top + height
    return left, top, right, bottom


def draw_save_button(frame, box: tuple[int, int, int, int], saved: bool = False) -> None:
    left, top, right, bottom = box
    fill_color = (40, 125, 40) if saved else (45, 45, 45)
    label = "Saved!" if saved else SAVE_BUTTON_LABEL
    cv2.rectangle(frame, (left, top), (right, bottom), fill_color, cv2.FILLED)
    cv2.rectangle(frame, (left, top), (right, bottom), (255, 255, 255), 2)
    text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)
    text_x = max(left + 4, left + (right - left - text_size[0]) // 2)
    text_y = min(bottom - 5, top + (bottom - top + text_size[1]) // 2)
    cv2.putText(frame, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                0.58, (255, 255, 255), 2, cv2.LINE_AA)


class BadPicSaver:
    """Hold the latest normalized model-input frame for the live save button."""

    def __init__(self, directory: Path):
        self.directory = directory
        self._processed_frame = None
        self._button_box: tuple[int, int, int, int] | None = None
        self._confirmation_until = 0.0

    def update(self, processed_frame, button_box: tuple[int, int, int, int]) -> None:
        self._processed_frame = processed_frame.copy()
        self._button_box = button_box

    def confirmation_active(self) -> bool:
        return time.monotonic() < self._confirmation_until

    def on_mouse(self, event, x, y, _flags, _parameter) -> None:
        if event != cv2.EVENT_LBUTTONUP or self._processed_frame is None or self._button_box is None:
            return
        left, top, right, bottom = self._button_box
        if not (left <= x <= right and top <= y <= bottom):
            return
        try:
            output = save_bad_pic(self._processed_frame, self.directory)
        except RuntimeError as exc:
            print(f"Error: {exc}")
            return
        self._confirmation_until = time.monotonic() + SAVE_CONFIRMATION_SECONDS
        print(f"Saved bad picture to {output}")


def is_exit_key(key: int) -> bool:
    return key in (ord("q"), 27, 24)


def cuda_available() -> bool:
    try:
        import torch
        return bool(torch.cuda.is_available())
    except (ImportError, AttributeError):
        return False


def resolve_device(requested: str) -> str:
    if requested != "auto":
        return requested
    return "0" if cuda_available() else "cpu"


def resolve_live_imgsz(requested: int | None, model_path: Path | None = None) -> int:
    if requested is not None:
        return requested
    if model_path is not None and Path(model_path).name in {
            LIVE_CHECKPOINT.name, LEGACY_POSE_CHECKPOINT.name}:
        return LIVE_CHECKPOINT_IMGSZ
    return DEFAULT_LIVE_IMGSZ


def resolve_live_conf(requested: float | None) -> float:
    return requested if requested is not None else DEFAULT_LIVE_CONF


def select_single_drumstick(detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Legacy utility; the live pipeline now retains multiple drumsticks."""
    drumstick_indices = [
        index for index, detection in enumerate(detections)
        if detection["class_id"] == 1
    ]
    if len(drumstick_indices) <= 1:
        return detections
    best_index = max(
        drumstick_indices,
        key=lambda index: detections[index]["confidence"],
    )
    return [
        detection for index, detection in enumerate(detections)
        if detection["class_id"] != 1 or index == best_index
    ]


def _crop_around_cymbal(frame, detection: dict[str, Any]):
    """Return an expanded cymbal crop and its top-left frame offset."""
    frame_height, frame_width = frame.shape[:2]
    left, top, right, bottom = detection["box"]
    box_width = max(1, right - left)
    box_height = max(1, bottom - top)
    margin_x = max(8, round(box_width * HORIZONTAL_RECOVERY_MARGIN))
    margin_y = max(8, round(box_height * HORIZONTAL_RECOVERY_MARGIN))
    crop_left = max(0, left - margin_x)
    crop_top = max(0, top - margin_y)
    crop_right = min(frame_width, right + margin_x)
    crop_bottom = min(frame_height, bottom + margin_y)
    if crop_right <= crop_left or crop_bottom <= crop_top:
        return None
    return frame[crop_top:crop_bottom, crop_left:crop_right].copy(), crop_left, crop_top


def _offset_detections(
        detections: list[dict[str, Any]], offset_x: int, offset_y: int,
        frame_shape) -> list[dict[str, Any]]:
    """Translate crop detections back into full-frame coordinates."""
    frame_height, frame_width = frame_shape[:2]
    translated = []
    for detection in detections:
        left, top, right, bottom = detection["box"]
        translated.append({
            **detection,
            "box": [
                max(0, min(frame_width, left + offset_x)),
                max(0, min(frame_height, top + offset_y)),
                max(0, min(frame_width, right + offset_x)),
                max(0, min(frame_height, bottom + offset_y)),
            ],
        })
        if detection.get("tip") is not None:
            x, y = detection["tip"]
            translated[-1]["tip"] = [
                max(0, min(frame_width - 1, x + offset_x)),
                max(0, min(frame_height - 1, y + offset_y)),
            ]
    return translated


def choose_default_model(root: str | Path) -> Path:
    root = Path(root)
    # Stable snapshot of the active pose run's best.pt, not its last.pt.
    # Keep older exports intact for explicit --model overrides/rollback.
    checkpoint = root / LIVE_CHECKPOINT
    if checkpoint.is_file():
        return checkpoint
    legacy_checkpoint = root / LEGACY_POSE_CHECKPOINT
    if legacy_checkpoint.is_file():
        return legacy_checkpoint
    exported = root / "models" / "exported"
    candidates = sorted(exported.rglob("*.xml")) if exported.is_dir() else []
    if not candidates and exported.is_dir():
        candidates = sorted(exported.rglob("*.onnx"))
    if candidates:
        return candidates[0]
    return root / "models" / "best.pt"


def _result_detections(result: Any) -> list[dict[str, Any]]:
    detections: list[dict[str, Any]] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return detections
    names = getattr(result, "names", CLASS_NAMES)
    xyxy = boxes.xyxy.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)
    keypoints = getattr(result, "keypoints", None)
    points = keypoints.data.cpu().numpy() if keypoints is not None else None
    for index, (coordinates, confidence, class_id) in enumerate(zip(xyxy, confidences, classes)):
        name = names.get(int(class_id), str(class_id)) if isinstance(names, dict) else names[int(class_id)]
        detections.append({
            "box": [int(round(value)) for value in coordinates],
            "class_id": int(class_id),
            "class_name": name,
            "confidence": float(confidence),
        })
        # One tip per stick; cymbal and snare keypoints are masked in training.
        # Never invent a tip from a box corner or fall back to OpenCV.
        if class_id == 1 and points is not None and index < len(points) and len(points[index]):
            point = points[index][0]
            if len(point) >= 3:
                x, y, score = map(float, point[:3])
                shape = getattr(result, "orig_shape", None)
                inside = x >= 0 and y >= 0 and (shape is None or (x < shape[1] and y < shape[0]))
                if all(math.isfinite(v) for v in (x, y, score)) and inside and score >= TIP_CONFIDENCE:
                    detections[-1].update(
                        tip=[int(x), int(y)], tip_confidence=score,
                        tip_status="observed", tip_source="yolo_pose",
                    )
    return detections


def draw_detections(frame, detections: list[dict[str, Any]]):
    output = frame.copy()
    for detection in detections:
        left, top, right, bottom = detection["box"]
        class_id = detection["class_id"]
        color = CLASS_COLORS.get(class_id, (255, 255, 255))
        label = f"{detection['class_name']} {detection['confidence']:.2f}"
        cv2.rectangle(output, (left, top), (right, bottom), color, 2)
        cv2.putText(output, label, (max(0, left), max(22, top - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        if class_id == 1 and detection.get("tip") is not None:
            tip = tuple(detection["tip"])
            status = detection.get("tip_status", "observed")
            tracked = status != "observed"
            tip_label = {"tracked": "tracked tip", "estimated": "estimated tip"}.get(status, "tip")
            if detection.get("tip_source") == "yolo_pose":
                tip_label = "YOLO tip"
            if detection.get("tip_source") == "line_endpoint":
                tip_label = "line tip (est.)"
            # Contrasting outline keeps the tip visible on light and dark regions.
            cv2.circle(output, tip, 7, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.circle(output, tip, 4, (255, 255, 255), 1 if tracked else -1, cv2.LINE_AA)
            cv2.putText(output, tip_label, (tip[0] + 10, tip[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(output, tip_label, (tip[0] + 10, tip[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
    return output


@dataclass(frozen=True)
class FrameResult:
    """One completed inference and the exact processed/raw images it describes.

    Images are worker-owned and must be treated as read-only by consumers.
    """

    frame: Any
    raw_frame: Any
    frame_id: int
    captured_at: float
    detections: list[dict[str, Any]]
    elapsed: float


def preview_source(result, processed, raw, *, now):
    """Pair overlays with their source image; discard results over 0.5 s old."""
    if result is None or not 0 <= now - result.captured_at <= .5:
        return processed, raw, []
    return result.frame, result.raw_frame, result.detections


class InferenceWorker:
    """Run inference on only the newest submitted frame."""

    def __init__(self, model: Any, imgsz: int, conf: float, device: str):
        self.model = model
        self.imgsz = imgsz
        self.conf = conf
        self.device = device
        self._condition = threading.Condition()
        self._pending = None
        self._latest: list[dict[str, Any]] = []
        self._result: FrameResult | None = None
        self._sequence = 0
        self._last_inference_time = 0.0
        self._error: Exception | None = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="inference", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit(self, frame, *, raw_frame=None, timestamp=None) -> None:
        captured_at = time.monotonic() if timestamp is None else timestamp
        owned_frame = frame.copy()
        owned_raw = raw_frame.copy() if raw_frame is not None else owned_frame
        with self._condition:
            self._sequence += 1
            self._pending = (owned_frame, owned_raw, self._sequence, captured_at)
            self._condition.notify()

    def snapshot(self) -> tuple[list[dict[str, Any]], float]:
        with self._condition:
            return list(self._latest), self._last_inference_time

    def snapshot_frame(self) -> FrameResult | None:
        with self._condition:
            return self._result

    def error(self) -> Exception | None:
        with self._condition:
            return self._error

    def stop(self) -> None:
        with self._condition:
            self._stop = True
            self._condition.notify()
        self._thread.join(timeout=2.0)

    def _recover_horizontal_drumstick(
            self, frame, detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
        cymbals = [detection for detection in detections if detection["class_id"] == 0]
        if not cymbals:
            return []
        crop_data = _crop_around_cymbal(frame, max(cymbals, key=lambda d: d["confidence"]))
        if crop_data is None:
            return []
        crop, offset_x, offset_y = crop_data
        results = self.model.predict(
            source=crop, imgsz=self.imgsz, conf=self.conf,
            device=self.device, verbose=False,
        )
        crop_detections = _result_detections(results[0]) if results else []
        crop_sticks = [d for d in crop_detections if d["class_id"] == 1]
        return _offset_detections(crop_sticks, offset_x, offset_y, frame.shape)

    def infer(self, frame, *, timestamp=None):
        """Synchronous live pipeline, also used by offline video verification.

        Only one caller may own inference/tracker state. Do not call this from
        outside while the worker thread is running.
        """
        started = time.perf_counter()
        results = self.model.predict(
            source=frame, imgsz=self.imgsz, conf=self.conf,
            device=self.device, verbose=False,
        )
        detections = _result_detections(results[0]) if results else []
        if not any(detection["class_id"] == 1 for detection in detections):
            detections = detections + self._recover_horizontal_drumstick(frame, detections)
        return detections, time.perf_counter() - started

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stop:
                    self._condition.wait(timeout=0.5)
                if self._stop:
                    return
                frame, raw_frame, frame_id, captured_at = self._pending
                self._pending = None
            try:
                detections, elapsed = self.infer(frame, timestamp=captured_at)
            except Exception as exc:
                with self._condition:
                    self._error = exc
                return
            with self._condition:
                self._latest = detections
                self._last_inference_time = elapsed
                self._result = FrameResult(frame, raw_frame, frame_id, captured_at,
                                           detections, elapsed)


def load_model(path: Path):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("Ultralytics is not installed; run: python -m pip install -r requirements.txt") from exc
    if not path.exists():
        raise FileNotFoundError(f"Model not found: {path}")
    return YOLO(str(path))


def configure_capture(camera, width: int, height: int, fourcc: str) -> None:
    """Request full-frame RGB capture; MJPEG permits 1080p over USB2.

    Gemini 2's 16:9 RGB modes expose its widest color FOV (86 x 55 degrees).
    Capture resolution is independent of YOLO's inference input size.
    """
    for prop, value, label in (
        (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc), fourcc),
        (cv2.CAP_PROP_FRAME_WIDTH, width, f"width {width}"),
        (cv2.CAP_PROP_FRAME_HEIGHT, height, f"height {height}"),
        (cv2.CAP_PROP_FPS, 30, "30 FPS"),
    ):
        if not camera.set(prop, value):
            print(f"Warning: camera did not accept {label} request.")
    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    code = int(camera.get(cv2.CAP_PROP_FOURCC))
    actual_fourcc = ''.join(chr((code >> (8 * i)) & 255) for i in range(4)).strip('\x00')
    print(f"Capture: {actual_width}x{actual_height}, {actual_fourcc}, "
          f"reported {camera.get(cv2.CAP_PROP_FPS):g} FPS "
          f"(requested {width}x{height} {fourcc} at 30 FPS)")
    if (actual_width, actual_height) != (width, height):
        print("Warning: camera negotiated a different resolution; widest RGB FOV is not guaranteed.")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live ride, snare, and multiple-drumstick pose detector")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--lighting-profile", type=Path, default=None,
                        help="training lighting reference; defaults to models/lighting_profile.json")
    parser.add_argument("--preprocessing", choices=("local", "reference", "none"), default=None,
                        help="local: reference-free contrast (default); reference: legacy matching; none: raw")
    parser.add_argument("--no-normalization", action="store_true",
                        help="disable lighting correction for comparison")
    parser.add_argument("--camera", type=int, default=4,
                        help="camera index (default: 4, current Orbbec Gemini RGB stream)")
    parser.add_argument("--model", type=Path, default=None,
                        help="checkpoint override (default: models/snare.pt)")
    parser.add_argument("--imgsz", type=int, default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=1920,
                        help="capture width (default: 1920; 16:9 gives widest Gemini RGB FOV)")
    parser.add_argument("--height", type=int, default=1080,
                        help="capture height (default: 1080; independent of --imgsz)")
    parser.add_argument("--fourcc", choices=("MJPG", "YUYV"), default="MJPG",
                        help="capture format (default: MJPG for USB2 bandwidth)")
    parser.add_argument("--allow-low-light-fps", action="store_true",
                        help="allow the camera to reduce FPS for longer exposures in dim light")
    parser.add_argument("--process-every", type=int, default=1)
    parser.add_argument("--save", action="store_true", help="kept for compatibility; recording is always enabled")
    parser.add_argument(
        "--output", type=Path, default=None,
        help="raw camera video path without overlays; defaults to recordings/<timestamp>.mp4",
    )
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    if args.process_every < 1:
        parser.error("--process-every must be at least 1")
    root = args.root.resolve()
    model_path = (args.model or choose_default_model(root)).resolve()
    imgsz = resolve_live_imgsz(args.imgsz, model_path)
    conf = resolve_live_conf(args.conf)
    device = resolve_device(args.device)
    try:
        mode = 'none' if args.no_normalization else (args.preprocessing or
                ('reference' if args.lighting_profile else 'local'))
        normalizer = make_processor(mode, args.lighting_profile or root / "models" / "lighting_profile.json")
        print(f"Preprocessing: {mode}")
        model = load_model(model_path)
        print(f"Model: {model_path}; input: {imgsz}; tip source: YOLO pose keypoints only")
        print(f"Classes: {getattr(model, 'names', CLASS_NAMES)}; multiple drumsticks enabled")
        if getattr(model, "task", None) != "pose":
            print("Warning: this model has no pose keypoints; tip markers will not be shown.")
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        print(f"Error: {exc}")
        return 1

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        print(f"Error: could not open camera index {args.camera}")
        camera.release()
        return 1

    print(f"Camera: index {args.camera}; default wide RGB mode is 1920x1080 MJPG (16:9).")
    configure_capture(camera, args.width, args.height, args.fourcc)
    if sys.platform.startswith("linux"):
        try:
            settings = set_auto_frame_rate(args.camera, args.allow_low_light_fps)
            mode = "enabled" if settings["current"] else "disabled"
            print(f"Camera: requested 30 FPS; automatic low-light FPS reduction {mode}.")
        except OSError as exc:
            print(f"Warning: could not set low-light frame-rate control: {exc}")

    recording_path = args.output or timestamped_output_path(Path("recordings"))
    writer = None
    bad_pic_saver = BadPicSaver(root / "bad pics")
    # Scale only the windows, not inference, saved snapshots, or recordings.
    for name in (WINDOW_NAME, RAW_WINDOW_NAME):
        cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(name, 960, round(960 * camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
                                        / max(1, camera.get(cv2.CAP_PROP_FRAME_WIDTH))))
    cv2.setMouseCallback(WINDOW_NAME, bad_pic_saver.on_mouse)
    worker = InferenceWorker(model, imgsz, conf, device)
    worker.start()
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
                print("Error: camera frame read failed")
                break
            captured_at = time.monotonic()
            processed = normalizer.apply(frame) if normalizer is not None else frame
            frame_count += 1
            display_frames += 1
            if frame_count % args.process_every == 0:
                worker.submit(processed, raw_frame=frame, timestamp=captured_at)
            result = worker.snapshot_frame()
            worker_error = worker.error()
            if worker_error is not None:
                print(f"Error: inference failed: {worker_error}")
                break
            if result is not None and result.frame_id != previous_frame_id:
                inference_count += 1
                previous_frame_id = result.frame_id
            now = time.perf_counter()
            if now - displayed_at >= 1.0:
                display_fps = display_frames / (now - displayed_at)
                display_frames = 0
                displayed_at = now
            inference_fps = inference_count / max(now - inference_started, 1e-9)
            display_frame, display_raw, detections = preview_source(
                result, processed, frame, now=time.monotonic())
            preview = draw_detections(display_frame, detections)
            cv2.putText(preview, f"Display FPS: {display_fps:.1f}", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(preview, f"Inference FPS: {inference_fps:.1f}", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
            button_box = save_button_box(preview.shape)
            # Save the exact normalized frame that detections and the main
            # preview describe. Keep raw frames exclusively in the MP4/raw view.
            bad_pic_saver.update(display_frame, button_box)
            draw_save_button(preview, button_box, bad_pic_saver.confirmation_active())
            if writer is None:
                recording_path.parent.mkdir(parents=True, exist_ok=True)
                height, width = preview.shape[:2]
                writer = cv2.VideoWriter(
                    str(recording_path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (width, height)
                )
                if not writer.isOpened():
                    raise RuntimeError(f"Error: could not open video writer: {recording_path}")
            writer.write(frame)
            cv2.imshow(WINDOW_NAME, preview)
            # Compare the same captured instant in both windows, with no
            # normalization or drawing applied to the camera's original image.
            cv2.imshow(RAW_WINDOW_NAME, display_raw)
            key = cv2.waitKey(1) & 0xFF
            if is_exit_key(key):
                break
    finally:
        worker.stop()
        camera.release()
        if writer is not None:
            writer.release()
            if recording_path.exists() and recording_path.stat().st_size > 0:
                print(f"Saved recording to {recording_path}")
            else:
                print(f"Error: recording was not saved: {recording_path}")
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
