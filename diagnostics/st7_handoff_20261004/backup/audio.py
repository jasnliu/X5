"""Local datagram protocol for the separate ST7 cymbal-sound process."""
from __future__ import annotations

import json
import math
import socket
import time
from pathlib import Path
from typing import Any


AUDIO_PROTOCOL_VERSION = 2
AUDIO_MAX_MESSAGE_BYTES = 4096
AUDIO_HEARTBEAT_TIMEOUT_SECONDS = 2.0


class AudioSender:
    """Best-effort sender used by the non-ROS ST7 bridge process."""

    def __init__(self, path: str | Path):
        self.path = str(path)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def send(self, message: dict[str, Any]) -> None:
        payload = json.dumps(
            {"version": AUDIO_PROTOCOL_VERSION, **message},
            separators=(",", ":"),
            allow_nan=False,
        ).encode()
        if len(payload) > AUDIO_MAX_MESSAGE_BYTES:
            raise ValueError("Audio status message is too large")
        try:
            self.socket.sendto(payload, self.path)
        except (FileNotFoundError, ConnectionRefusedError, BlockingIOError):
            # The control window may still be opening or may already be closed.
            pass

    def status(self, state: str, detail: str = "") -> None:
        self.send({
            "kind": "status",
            "state": state,
            "detail": detail,
            "sent_at": time.monotonic(),
        })

    def hit(
            self, event_at: float, detected_at: float, score: float,
            normality_score: float,
    ) -> None:
        self.send({
            "kind": "hit",
            "event_at": float(event_at),
            "detected_at": float(detected_at),
            "score": float(score),
            "normality_score": float(normality_score),
        })

    def close(self) -> None:
        self.socket.close()

    def progress(self, finalized_at: float, captured_at: float) -> None:
        self.send({'kind': 'progress', 'finalized_at': finalized_at,
                   'captured_at': captured_at, 'sent_at': time.monotonic()})


class AudioReceiver:
    """Receive readiness heartbeats and timestamped ST7 HIT events."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        if len(str(self.path).encode()) >= 100:
            raise ValueError("Audio socket path is too long")
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            self.socket.bind(str(self.path))
        except Exception:
            self.socket.close()
            raise
        self.socket.setblocking(False)
        self.state = "starting"
        self.detail = "Starting ST7 with the TONOR microphone"
        self.received_at = 0.0
        self.finalized_at = None
        self.progress_received_at = None

    def poll(self) -> list[dict[str, Any]]:
        messages = []
        while True:
            try:
                payload = self.socket.recv(AUDIO_MAX_MESSAGE_BYTES)
            except BlockingIOError:
                break
            try:
                message = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if (not isinstance(message, dict)
                    or message.get("version") != AUDIO_PROTOCOL_VERSION):
                continue
            kind = message.get("kind")
            if kind == 'progress':
                finalized, captured = message.get('finalized_at'), message.get('captured_at')
                if (all(isinstance(v, (float, int)) and not isinstance(v, bool)
                        and math.isfinite(v) for v in (finalized, captured))
                        and finalized <= captured <= time.monotonic()+.05
                        and (self.finalized_at is None or finalized >= self.finalized_at)):
                    self.finalized_at = float(finalized)
                    self.progress_received_at = time.monotonic()
                    messages.append(message)
                continue
            if (kind == "status"
                    and message.get("state") in {"starting", "ready", "error", "stopped"}):
                self.state = message["state"]
                self.detail = str(message.get("detail", ""))[:500]
                self.received_at = time.monotonic()
                messages.append(message)
                continue
            if kind != "hit":
                continue
            values = (
                message.get("event_at"),
                message.get("detected_at"),
                message.get("score"),
                message.get("normality_score"),
            )
            if (not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(float(value)) for value in values)
                    or not 0.0 <= float(values[2]) <= 1.0
                    or not 0.0 <= float(values[3]) <= 100.0):
                continue
            normalized = dict(message)
            normalized["event_at"] = float(values[0])
            normalized["detected_at"] = float(values[1])
            normalized["score"] = float(values[2])
            normalized["normality_score"] = float(values[3])
            messages.append(normalized)
        return messages

    def ready(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.state == "ready"
            and 0.0 <= now - self.received_at <= AUDIO_HEARTBEAT_TIMEOUT_SECONDS
        )

    def close(self) -> None:
        self.socket.close()
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
