"""Only permitted outgoing packet: SDK-equivalent RobStride state request.
No enable, disable, mode, zeroing, gravity compensation or command interface.
"""
import fcntl
import math
import os
import re
import select
import socket
import struct
import time

FRAME = struct.Struct('=IB3x8s')
EFF = 0x80000000
PAYLOAD = b'\x01' + b'\x00' * 7

# Right joint 5 has the same measured travel as the known-good left joint 5,
# but its reported range is shifted by +8.5 degrees.  Keep this as a software
# zero correction; no motor's stored zero is rewritten.
RIGHT_JOINT5_ZERO_OFFSET = math.radians(-8.5)

def encoder_to_joint(side, motor, encoder_angle):
    """Convert a motor encoder angle to the joint angle used by RViz/control."""
    joint_angle = -float(encoder_angle)
    if side == 'right' and motor == 5:
        joint_angle += RIGHT_JOINT5_ZERO_OFFSET
    return joint_angle

def joint_to_motor(side, motor, joint_angle):
    """Inverse of encoder_to_joint for motor position commands."""
    offset = RIGHT_JOINT5_ZERO_OFFSET if side == 'right' and motor == 5 else 0.
    return offset - float(joint_angle)

def request_frame(motor):
    if motor not in range(1, 9): raise ValueError('Invalid motor')
    return FRAME.pack(EFF | (2 << 24) | (0xfd << 8) | motor, 8, PAYLOAD)

def decode(data):
    if len(data) != FRAME.size: raise RuntimeError('Malformed CAN frame')
    cid, dlc, payload = FRAME.unpack(data)
    if cid & 0x60000000: raise RuntimeError('CAN error or RTR frame')
    if not cid & EFF: return None
    kind, motor = (cid >> 24) & 31, (cid >> 8) & 255
    if kind == 21: raise RuntimeError('Motor fault frame')
    if kind != 2 or motor not in range(1, 9): return None
    if dlc != 8: raise RuntimeError('Short state feedback')
    if (cid >> 16) & 63: raise RuntimeError(f'Motor {motor} reports fault bits')
    if (cid >> 22) & 3: raise RuntimeError(f'Motor {motor} is NOT disabled; no motor state was changed')
    angle = int.from_bytes(payload[:2], 'big') / 65535 * 25.14 - 12.57
    return motor, angle

class Observer:
    def __init__(self, left='can1', right='can0'):
        if left == right: raise ValueError('Left and right CAN interfaces must differ')
        self.sockets = {}
        self.locks = []
        self.tx_count = 0
        try:
            for side, bus in [('left', left), ('right', right)]:
                if not re.fullmatch(r'[a-zA-Z0-9_-]+', bus): raise ValueError('Bad interface')
                fd = os.open('/tmp/openarmx-cartesian-' + bus + '.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                self.locks.append(fd)
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[side] = s
                # CAN_RAW_ERR_FILTER=2 from linux/can/raw.h; not exposed by Python.
                s.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack('=I', 0x1fffffff))
                s.bind((bus,))
                s.setblocking(False)
        except Exception:
            self.close()
            raise

    def sample(self):
        # Discard old feedback; each returned batch must follow new requests.
        for s in self.sockets.values():
            for _ in range(1024):
                try: decode(s.recv(16))
                except BlockingIOError: break
            else: raise RuntimeError('CAN receive flood')
        for s in self.sockets.values():
            for motor in range(1, 9):
                frame = request_frame(motor)
                if s.send(frame) != 16: raise RuntimeError('Short CAN write')
                self.tx_count += 1
        state = {side: {} for side in self.sockets}
        deadline = time.monotonic() + .15
        while time.monotonic() < deadline:
            ready, _, _ = select.select(list(self.sockets.values()), [], [], max(0, deadline-time.monotonic()))
            for side, s in self.sockets.items():
                if s in ready:
                    value = decode(s.recv(16))
                    if value: state[side][value[0]] = value[1]
            if all(len(v) == 8 for v in state.values()):
                q = {}
                for side, readings in state.items():
                    q.update({f'openarmx_{side}_joint{i}': encoder_to_joint(side, i, readings[i]) for i in range(1, 8)})
                    q[f'openarmx_{side}_finger_joint1'] = max(0., min(.044, -.044 * readings[8] / 1.0472))
                return q
        raise RuntimeError('Missing fresh feedback: ' + str({k: sorted(set(range(1,9))-v.keys()) for k,v in state.items()}))

    def close(self):
        for s in self.sockets.values(): s.close()
        self.sockets.clear()
        for fd in self.locks: os.close(fd)
        self.locks.clear()
