"""One-way, nonblocking event copies for an optional visual-only process."""
import json
import socket

from .audio import AudioSender, AUDIO_PROTOCOL_VERSION


class MirroredAudioSender(AudioSender):
    """Deliver control first, then a best-effort copy. Never wait for a viewer.

    Missing, closed or overloaded visual sockets cannot affect the original
    detector/control protocol. The viewer has no path to send motor commands.
    """
    def __init__(self, path, instrument='ride', visual_path=None):
        super().__init__(path, instrument)
        self.visual_path = str(visual_path) if visual_path is not None else None
        self.visual_socket = None
        if self.visual_path is not None:
            try:
                self.visual_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.visual_socket.setblocking(False)
            except OSError:
                if self.visual_socket is not None:
                    self.visual_socket.close()
                self.visual_socket = None

    def send(self, message):
        super().send(message)  # Preserve original validation/delivery semantics.
        if self.visual_socket is not None and message.get('kind') in {'hit', 'status'}:
            try:
                payload = json.dumps({'version': AUDIO_PROTOCOL_VERSION, **message,
                                      'instrument': self.instrument},
                                     separators=(',', ':'), allow_nan=False).encode()
                self.visual_socket.sendto(payload, self.visual_path)
            except (OSError, TypeError, ValueError):
                pass  # Drawing is never a reason to fault calibration or motion.

    def close(self):
        try:
            super().close()
        finally:
            if self.visual_socket is not None:
                self.visual_socket.close()
                self.visual_socket = None
