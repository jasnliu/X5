"""Small local datagram protocol shared by the ROS and Y2 Python environments."""
from __future__ import annotations

import json
import socket
import time
from pathlib import Path
from typing import Any

from .vision import (
    cymbal_grid_geometry,
    select_observed_stick,
    valid_box,
    valid_point,
)

PROTOCOL_VERSION = 2
MAX_MESSAGE_BYTES = 8192
CAMERA_FRESH_SECONDS = 0.5  # Match Y2's live-preview freshness limit.


def has_required_detection(detections: list[dict[str, Any]]) -> bool:
    """A qualifying frame has one drumstick box and its fresh YOLO tip."""
    return select_observed_stick(detections) is not None


class DetectionSender:
    def __init__(self, path: str | Path):
        self.path = str(path)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def send(self, message: dict[str, Any]) -> None:
        payload = json.dumps(
            {"version": PROTOCOL_VERSION, **message},
            separators=(",", ":"), allow_nan=False,
        ).encode()
        if len(payload) > MAX_MESSAGE_BYTES:
            raise ValueError("Camera status message is too large")
        try:
            self.socket.sendto(payload, self.path)
        except (FileNotFoundError, ConnectionRefusedError, BlockingIOError):
            # The control window may still be starting or may already be closing.
            pass

    def status(self, state: str, detail: str = "") -> None:
        self.send({"kind": "status", "state": state, "detail": detail,
                   "sent_at": time.monotonic()})

    def frame(self, frame_id: int, captured_at: float,
              detections: list[dict[str, Any]]) -> None:
        sticks = [d for d in detections if d.get("class_id") == 1]
        observed_stick = select_observed_stick(detections)
        cymbal = cymbal_grid_geometry(detections)
        age = time.monotonic() - float(captured_at)
        fresh = 0 <= age <= CAMERA_FRESH_SECONDS
        required = fresh and observed_stick is not None
        self.send({
            "kind": "frame",
            "frame_id": int(frame_id),
            "captured_at": float(captured_at),
            "sent_at": time.monotonic(),
            "drumstick": bool(sticks),
            "tip": any(d.get("tip") is not None for d in sticks),
            "required": required,
            "cymbal": cymbal is not None,
            "cymbal_box": None if cymbal is None else list(cymbal["outer"]),
            "target_box": None if cymbal is None else list(cymbal["target"]),
            "drumstick_box": (None if observed_stick is None else
                               list(valid_box(observed_stick.get("box")))),
            "tip_point": (None if observed_stick is None else
                           list(valid_point(observed_stick.get("tip")))),
        })

    def close(self) -> None:
        self.socket.close()


class DetectionReceiver:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        if len(str(self.path).encode()) >= 100:
            raise ValueError("Detection socket path is too long")
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(str(self.path))
        self.socket.setblocking(False)
        self.state = "starting"
        self.detail = "Waiting for the first processed camera result"
        self.frame_id = -1
        self.received_at = 0.0
        self.captured_at = 0.0
        self.drumstick = False
        self.tip = False
        self.required = False
        self.cymbal = False
        self.cymbal_box = None
        self.target_box = None
        self.drumstick_box = None
        self.tip_point = None

    def poll(self) -> list[dict[str, Any]]:
        messages = []
        while True:
            try:
                payload = self.socket.recv(MAX_MESSAGE_BYTES)
            except BlockingIOError:
                break
            try:
                message = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(message, dict) or message.get("version") != PROTOCOL_VERSION:
                continue
            kind = message.get("kind")
            if kind == "status" and message.get("state") in {"starting", "ready", "error", "stopped"}:
                self.state = message["state"]
                self.detail = str(message.get("detail", ""))[:500]
                messages.append(message)
            elif (kind == "frame" and isinstance(message.get("frame_id"), int)
                  and not isinstance(message.get("frame_id"), bool)
                  and message["frame_id"] >= self.frame_id):
                cymbal_box = valid_box(message.get("cymbal_box"))
                target_box = valid_box(message.get("target_box"))
                drumstick_box = valid_box(message.get("drumstick_box"))
                tip_point = valid_point(message.get("tip_point"))
                normalized = dict(message)
                normalized["cymbal"] = (message.get("cymbal") is True
                                         and cymbal_box is not None and target_box is not None)
                normalized["cymbal_box"] = cymbal_box
                normalized["target_box"] = target_box
                normalized["drumstick_box"] = drumstick_box
                normalized["tip_point"] = tip_point
                normalized["required"] = (message.get("required") is True
                                           and drumstick_box is not None and tip_point is not None)
                self.state = "ready"
                self.frame_id = message["frame_id"]
                self.received_at = time.monotonic()
                captured_at = message.get("captured_at")
                self.captured_at = float(captured_at) if isinstance(captured_at, (int, float)) else 0.0
                self.drumstick = message.get("drumstick") is True
                self.tip = message.get("tip") is True
                self.required = normalized["required"]
                self.cymbal = normalized["cymbal"]
                self.cymbal_box = cymbal_box
                self.target_box = target_box
                self.drumstick_box = drumstick_box
                self.tip_point = tip_point
                messages.append(normalized)
        return messages

    def fresh(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (self.state == "ready"
                and 0 <= now - self.received_at <= CAMERA_FRESH_SECONDS
                and 0 <= now - self.captured_at <= CAMERA_FRESH_SECONDS)

    def close(self) -> None:
        self.socket.close()
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
