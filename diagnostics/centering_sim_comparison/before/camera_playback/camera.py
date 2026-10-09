#!/usr/bin/env python3
"""Playback-only camera overlay for visual diagnostics and pink-zone alignment.

The original camera process remains unchanged. This wrapper observes the same
inference results and adds preview text only. Visual cymbal motion is diagnostic;
ST7 microphone events, not this overlay, decide whether a strike hit the cymbal.
Detection messages sent to the arm controller are still produced by the original
DetectionSender implementation.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys


X5_ROOT = Path(__file__).resolve().parents[1]
if str(X5_ROOT) not in sys.path:
    sys.path.insert(0, str(X5_ROOT))

from camera_search import camera as base_camera
from camera_search.vision import (
    cymbal_grid_geometry,
    select_observed_stick,
    valid_point,
)


# A single size-change score is |delta width| + |delta height|.  Consecutive
# detector jitter at the old per-dimension limit (4 px in each direction)
# therefore remains stable at a score of 8; a hit must exceed that combined
# amount.  Keep this named constant close to the overlay so observed scores can
# be used to tune it later.
MIN_TOTAL_DIMENSION_CHANGE_PX = 8
STABLE_COLOR = (0, 220, 0)
CHANGING_COLOR = (0, 165, 255)
MISSING_COLOR = (160, 160, 160)
PINK_INSIDE_COLOR = (0, 220, 0)
PINK_OUTSIDE_COLOR = (0, 0, 255)
PINK_UNKNOWN_COLOR = (0, 165, 255)


@dataclass(frozen=True)
class BoxChangeStatus:
    visible: bool
    width_px: int | None = None
    height_px: int | None = None
    delta_width_px: int | None = None
    delta_height_px: int | None = None
    total_change_px: int | None = None
    changing: bool = False
    baseline: bool = False


class CymbalBoxChangeMonitor:
    """Measure combined size change between consecutive cymbal detections."""

    def __init__(self, minimum_change_px: int = MIN_TOTAL_DIMENSION_CHANGE_PX):
        if isinstance(minimum_change_px, bool) or int(minimum_change_px) < 0:
            raise ValueError("Minimum total size change must be a nonnegative pixel count")
        self.minimum_change_px = int(minimum_change_px)
        self.previous_dimensions = None
        self.status = BoxChangeStatus(False)

    def update(self, detections) -> BoxChangeStatus:
        geometry = cymbal_grid_geometry(detections)
        if geometry is None:
            self.previous_dimensions = None
            self.status = BoxChangeStatus(False)
            return self.status
        left, top, right, bottom = geometry["outer"]
        width = right - left
        height = bottom - top
        if self.previous_dimensions is None:
            self.status = BoxChangeStatus(
                True, width_px=width, height_px=height, baseline=True,
            )
        else:
            previous_width, previous_height = self.previous_dimensions
            delta_width = width - previous_width
            delta_height = height - previous_height
            total_change = total_dimension_change(delta_width, delta_height)
            changing = total_change > self.minimum_change_px
            self.status = BoxChangeStatus(
                True,
                width_px=width,
                height_px=height,
                delta_width_px=delta_width,
                delta_height_px=delta_height,
                total_change_px=total_change,
                changing=changing,
            )
        self.previous_dimensions = (width, height)
        return self.status


def total_dimension_change(delta_width_px, delta_height_px):
    """Combine width and height movement into one non-cancelling pixel score."""
    return abs(delta_width_px) + abs(delta_height_px)


def status_lines(status: BoxChangeStatus,
                 minimum_change_px: int = MIN_TOTAL_DIMENSION_CHANGE_PX):
    if not status.visible:
        return (
            "CYMBAL BOX: NOT DETECTED",
            f"Total size-change threshold: >{minimum_change_px}px",
            MISSING_COLOR,
        )
    dimensions = f"W={status.width_px}px  H={status.height_px}px"
    if status.baseline:
        return (
            "CYMBAL SIZE CHANGE: BASELINE",
            dimensions + f"  movement threshold >{minimum_change_px}px",
            STABLE_COLOR,
        )
    score = f"TOTAL CHANGE={status.total_change_px}px"
    if status.changing:
        return (
            "VISUAL CYMBAL MOVEMENT (NOT HIT TRIGGER): " + score,
            dimensions + "  ST7/TONOR sound decides hits",
            CHANGING_COLOR,
        )
    return (
        "CYMBAL STABLE: " + score,
        dimensions + f"  ignored at <= {minimum_change_px}px",
        STABLE_COLOR,
    )


def draw_box_change_status(preview, status: BoxChangeStatus, cv2,
                           minimum_change_px: int = MIN_TOTAL_DIMENSION_CHANGE_PX):
    """Draw two status lines without modifying detections or protocol data."""
    heading, detail, color = status_lines(status, minimum_change_px)
    for text, origin, scale in ((heading, (15, 124), .65), (detail, (15, 150), .52)):
        cv2.putText(preview, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(preview, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 2, cv2.LINE_AA)
    return preview


def pink_zone_membership(detections) -> bool | None:
    """Return visible-pink membership for the same directly observed tip sent to control."""
    geometry = cymbal_grid_geometry(detections)
    stick = select_observed_stick(detections)
    if geometry is None or stick is None:
        return None
    tip = valid_point(stick.get("tip"))
    if tip is None:
        return None
    x, y = tip
    left, top, right, bottom = geometry["target"]
    return left <= x <= right and top <= y <= bottom


def pink_zone_status(detections):
    """Return the camera-preview text and color for visible pink-zone membership."""
    membership = pink_zone_membership(detections)
    if membership is True:
        return "STICK TIP IN PINK ZONE: YES", PINK_INSIDE_COLOR
    if membership is False:
        return "STICK TIP IN PINK ZONE: NO", PINK_OUTSIDE_COLOR
    return "STICK TIP IN PINK ZONE: UNKNOWN", PINK_UNKNOWN_COLOR


def draw_pink_zone_status(preview, detections, cv2):
    """Draw live visible-pink membership without changing detection data."""
    label, color = pink_zone_status(detections)
    origin = (15, 182)
    cv2.putText(preview, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                .60, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(preview, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                .60, color, 2, cv2.LINE_AA)
    return preview


_monitor = CymbalBoxChangeMonitor()
_base_draw_cymbal_target = base_camera.draw_cymbal_target
_BaseDetectionSender = base_camera.DetectionSender


class PlaybackDetectionSender(_BaseDetectionSender):
    def frame(self, frame_id, captured_at, detections):
        _monitor.update(detections)
        return super().frame(frame_id, captured_at, detections)


def draw_playback_cymbal_target(preview, detections, cv2):
    _base_draw_cymbal_target(preview, detections, cv2)
    draw_box_change_status(preview, _monitor.status, cv2)
    return draw_pink_zone_status(preview, detections, cv2)


def main() -> int:
    # These replacements exist only inside the separate playback camera process.
    # The original module/file and the detection datagram format are untouched.
    base_camera.DetectionSender = PlaybackDetectionSender
    base_camera.draw_cymbal_target = draw_playback_cymbal_target
    try:
        return base_camera.main()
    except KeyboardInterrupt:
        # ROS launch intentionally stops this support process after the control
        # window exits. The base pipeline's finally block saves/releases video.
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
