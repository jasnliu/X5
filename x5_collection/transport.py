"""Collection-local gain audit and CENTER-ONLY entry into J7 MIT mode."""
import csv
import json
import math
import struct
import threading
import time

from camera_playback.playback_transport import PlaybackMotors, RetrySocket
from camera_playback.mit_strike import Joint7Channel, Joint7Worker, StrikeSettings
from camera_playback.recording_only import refresh_feedback
from smooth_playback.hardware import AuditedMotors, CaptureSocket
from safe_zone.encoder import FRAME


class CollectionMotors(PlaybackMotors):
    def start_audit(self, directory):
        self.directory = directory
        self.phase = 'CONNECT'
        self.parameter_values, self.gains_original = {}, {}
        self.gains_restored = True
        self.feedback_file = (directory/'feedback.csv').open('w', buffering=131072)
        self.command_file = (directory/'commands.jsonl').open('w', buffering=65536)
        self.feedback = csv.writer(self.feedback_file)
        self.feedback.writerow(['monotonic_s', 'phase', 'side', 'motor', 'position_rad',
                                'state', 'velocity_raw', 'torque_raw', 'temperature_c'])
        self.sockets = {side: CaptureSocket(s, side, self) for side, s in self.sockets.items()}

    # Reuse the existing bounded, readback-verified volatile gain operations.
    set_playback_speeds = AuditedMotors.set_playback_speeds
    read_position_gain = AuditedMotors.read_position_gain
    apply_position_gain_cap = AuditedMotors.apply_position_gain_cap
    _apply_position_gains = AuditedMotors._apply_position_gains
    restore_position_gains = AuditedMotors.restore_position_gains
    verify_original_gains_at_center = AuditedMotors.verify_original_gains_at_center

    def _send(self, side, frame):
        cid, _, data = FRAME.unpack(frame)
        kind, motor = (cid >> 24) & 31, cid & 255
        if kind in (17, 18) and data[:2] == b'\x1e\x70':
            if side != 'right' or motor not in range(1, 8) or self.right_joint7_session is not None:
                raise RuntimeError('Gain access outside collection playback ownership')
            if kind == 18:
                value = struct.unpack('<f', data[4:])[0]
                original = self.gains_original.get(motor)
                if original is None or not math.isfinite(value) or not original*.125 <= value <= original:
                    raise RuntimeError('Gain write outside saved original bounds')
            self.sockets[side].send(frame)
        else:
            super()._send(side, frame)
        self.command_file.write(json.dumps(dict(t=time.monotonic(), phase=self.phase,
            side=side, kind=kind, motor=motor, frame=frame.hex()))+'\n')

    def enter_mit_at_center(self):
        evidence = self.center_evidence()  # verifies ALL seven joints settled
        channel = Joint7Channel(self.sockets['right'].getsockname()[0])
        channel.socket = RetrySocket(channel.socket)  # stationary setup only
        helper = Joint7Worker('can0', self.states['right', 7][0], -1.4, 1.401,
                             StrikeSettings(), None, threading.Event(), lambda s: None)
        try:
            # The existing mode handoff disables J7 briefly. It is ONLY
            # permitted here, at fresh verified customized right center.
            helper._prepare(channel)
            if channel.mode != 0 or channel.sample.state != 2:
                raise RuntimeError('Center MIT entry was not confirmed')
        finally:
            channel.close()
        refresh_feedback(self, .08)
        self.request_right_joint7_mode_readback(clear=True)
        deadline = time.monotonic()+.5
        while self.right_joint7_mode_readback() != 0:
            self.poll()
            if time.monotonic() > deadline:
                raise RuntimeError('Parent MIT readback not confirmed at center')
            time.sleep(.002)
        # CSP's last torque includes its position-servo correction and is not
        # a reliable gravity bias at this upper joint limit. Start with zero
        # feed-forward and let the bounded measured-error hold identify load.
        self.begin_mit_center_return(0.)
        (self.directory/'center_mode_transfer.json').write_text(json.dumps(
            dict(center=evidence, powered_after=True, mode=0), indent=2)+'\n')

    def close(self):
        try:
            super().close()
        finally:
            for name in ('feedback_file', 'command_file'):
                if hasattr(self, name):
                    getattr(self, name).close()
