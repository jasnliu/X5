"""USB-serial control for the ESP32 hi-hat motor used during 100 BPM playback."""
from __future__ import annotations

import glob
import os
from pathlib import Path
import termios
import time


ESP_BAUD = 115200
ESP_HEARTBEAT_SECONDS = 0.1
ESP_STARTUP_SECONDS = 0.3
ESP_PROBE_SECONDS = 0.5
MOTOR2_TARGET_DEGREES = 100
DEFAULT_ESP_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def find_esp_port(requested: str | Path | None = DEFAULT_ESP_PORT) -> str:
    """Resolve the configured ESP32 port, or uniquely auto-detect one."""
    if requested is not None and str(requested).strip().lower() != "auto":
        port = str(Path(requested).expanduser())
        if not Path(port).exists():
            raise FileNotFoundError(f"ESP32 serial port does not exist: {port}")
        return port
    candidates = sorted(set(
        glob.glob("/dev/serial/by-id/*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/cu.usbmodem*")
    ))
    # A by-id link and its tty target identify the same device. Prefer the
    # stable by-id name and remove duplicate real paths before deciding.
    unique = {}
    for candidate in candidates:
        unique.setdefault(os.path.realpath(candidate), candidate)
        if "/dev/serial/by-id/" in candidate:
            unique[os.path.realpath(candidate)] = candidate
    ports = sorted(unique.values())
    if not ports:
        raise FileNotFoundError("No ESP32 serial port found")
    if len(ports) != 1:
        raise RuntimeError(
            "Multiple ESP32 serial ports found; pass --esp-port: "
            + ", ".join(ports)
        )
    return ports[0]


def configure_serial(fd: int) -> None:
    """Apply the proven motor_beat 115200-baud raw serial configuration."""
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNPAR
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


class HiHatController:
    """Drive only motor 2 of the copied motor_beat firmware.

    Explicit beat-program edges close on 2/4 and open on 1/3. The legacy
    send_beat API starts closed for finite collection cycles. Closure targets
    100 degrees; open targets zero. Motor-1 kick commands are never sent.
    """

    def __init__(self, port: str | Path | None = DEFAULT_ESP_PORT):
        self.requested_port = port
        self.port: str | None = None
        self.fd: int | None = None
        self.state = "starting"
        self.detail = "ESP32 hi-hat serial connection has not opened"
        self.opened_at: float | None = None
        self.next_heartbeat_at: float | None = None
        self.initialized = False
        self.firmware_verified = False
        self.next_probe_at: float | None = None
        self.sequence_active = False
        self.next_beat_closes = True
        self.beat_count = 0
        self.motor2_closed = False
        self.buffer = bytearray()

    @property
    def enabled(self) -> bool:
        return True

    def ready(self) -> bool:
        return self.fd is not None and self.state == "ready"

    def connect(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        try:
            self.port = find_esp_port(self.requested_port)
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                configure_serial(fd)
            except Exception:
                os.close(fd)
                raise
        except Exception as exc:
            self.state = "error"
            self.detail = str(exc)
            return False
        self.fd = fd
        self.state = "starting"
        self.detail = f"Warming up ESP32 hi-hat on {self.port} at {ESP_BAUD} baud"
        self.opened_at = now
        self.next_heartbeat_at = now
        return True

    def _fail(self, exc: Exception | str) -> None:
        self.state = "error"
        self.detail = str(exc)
        self.sequence_active = False

    def _write(self, command: bytes) -> None:
        if self.fd is None:
            raise RuntimeError("ESP32 hi-hat serial connection is closed")
        try:
            written = os.write(self.fd, command)
        except OSError as exc:
            self._fail(exc)
            raise RuntimeError("ESP32 hi-hat serial write failed: " + str(exc)) from exc
        if written != len(command):
            exc = RuntimeError("ESP32 hi-hat serial write was incomplete")
            self._fail(exc)
            raise exc

    def _send_stop_probe(self, now: float) -> None:
        self._write(b"S")
        self.initialized = True
        self.next_probe_at = now + ESP_PROBE_SECONDS
        self.detail = f"Verifying motor_beat firmware on {self.port}"

    def _mark_ready(self) -> None:
        self.firmware_verified = True
        self.state = "ready"
        self.detail = (
            f"ESP32 hi-hat ready on {self.port} at {ESP_BAUD} baud; "
            f"motor 2 alternates 0°/{MOTOR2_TARGET_DEGREES}°"
        )

    def _handle_line(self, text: str) -> None:
        if text.startswith("READY motor_beat"):
            if not self.initialized:
                self._write(b"S")
                self.initialized = True
            self._mark_ready()
        elif self.initialized and text.startswith("STOP -- all motors released"):
            self._mark_ready()
        elif text.startswith(("FAULT", "LINK LOST")):
            self._fail(text)

    def _drain(self) -> None:
        if self.fd is None:
            return
        while True:
            try:
                chunk = os.read(self.fd, 256)
            except BlockingIOError:
                break
            except OSError as exc:
                self._fail("ESP32 hi-hat serial read failed: " + str(exc))
                break
            if not chunk:
                break
            self.buffer.extend(chunk)
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            self._handle_line(line.decode("utf-8", "replace").strip())

    def tick(self, now: float | None = None) -> None:
        if self.fd is None or self.state == "error":
            return
        now = time.monotonic() if now is None else float(now)
        try:
            if self.next_heartbeat_at is None or now >= self.next_heartbeat_at:
                self._write(b"H")
                self.next_heartbeat_at = now + ESP_HEARTBEAT_SECONDS
            startup_elapsed = (
                self.opened_at is not None
                and now - self.opened_at >= ESP_STARTUP_SECONDS
            )
            if (not self.firmware_verified and startup_elapsed
                    and (not self.initialized or self.next_probe_at is None
                         or now >= self.next_probe_at)):
                self._send_stop_probe(now)
            self._drain()
        except Exception as exc:
            self._fail(exc)

    def start_sequence(self) -> None:
        if not self.ready():
            raise RuntimeError("ESP32 hi-hat is not ready")
        # Firmware encoder zero is the open position. The first arm beat closes
        # to 100 degrees, and the second returns to zero.
        self.sequence_active = True
        self.next_beat_closes = True
        self.beat_count = 0
        self.motor2_closed = False

    def send_beat(self) -> bytes:
        if not self.sequence_active:
            raise RuntimeError("ESP32 hi-hat beat sequence is not active")
        command = b"C" if self.next_beat_closes else b"O"
        self._write(command)
        self.motor2_closed = self.next_beat_closes
        self.next_beat_closes = not self.next_beat_closes
        self.beat_count += 1
        return command

    def send_state(self, closed: bool) -> bytes:
        """Explicit scheduled edge; same firmware targets as send_beat()."""
        if not self.sequence_active or not self.ready():
            raise RuntimeError("ESP32 hi-hat beat sequence is not active/ready")
        command = b"C" if closed else b"O"
        self._write(command)
        self.motor2_closed = bool(closed)
        self.next_beat_closes = not closed
        self.beat_count += 1
        return command

    def stop_sequence(self) -> None:
        if self.fd is not None and self.sequence_active:
            # Return motor 2 to encoder zero; the firmware releases it once the
            # zero deadband is reached. Heartbeats continue while the GUI runs.
            self._write(b"O")
        self.sequence_active = False
        self.next_beat_closes = True
        self.motor2_closed = False

    def release_all(self) -> None:
        """Immediately release both firmware outputs (Emergency Relax behavior)."""
        if self.fd is not None:
            self._write(b"S")
        self.sequence_active = False
        self.next_beat_closes = True
        self.motor2_closed = False

    def close(self) -> None:
        if self.fd is None:
            return
        try:
            os.write(self.fd, b"S")
        except OSError:
            pass
        try:
            os.close(self.fd)
        finally:
            self.fd = None
            self.sequence_active = False
            self.state = "stopped"
            self.detail = "ESP32 hi-hat serial connection closed; motors released"
