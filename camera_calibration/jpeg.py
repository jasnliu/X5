"""Strict JPEG decoding: reject libjpeg warnings instead of accepting partial pixels."""
import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import uuid

import numpy as np


class CorruptJpegError(ValueError):
    pass


def build_decoder():
    source = Path(__file__).with_name('jpeg_decoder.c')
    build = source.parent / 'build'
    binary = build / 'libx5jpeg.so'
    stamp = build / 'jpeg_signature.txt'
    signature = hashlib.sha256(source.read_bytes()).hexdigest()
    if binary.is_file() and stamp.is_file() and stamp.read_text() == signature:
        return binary
    build.mkdir(parents=True, exist_ok=True)
    temp = build / ('.jpeg-' + uuid.uuid4().hex + '.so')
    try:
        result = subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-fPIC', '-shared',
                                 str(source), '-ljpeg', '-o', str(temp)],
                                capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise RuntimeError('Strict JPEG decoder compilation failed (requires libjpeg headers):\n' + result.stderr)
        os.replace(temp, binary)
        stamp.write_text(signature)
    finally:
        temp.unlink(missing_ok=True)
    return binary


class JpegDecoder:
    def __init__(self):
        self.library = ctypes.CDLL(str(build_decoder()))
        self.function = self.library.x5_decode_jpeg
        self.function.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t,
                                  ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t]
        self.function.restype = ctypes.c_int

    def decode(self, data, width=1280, height=720):
        if not data:
            raise CorruptJpegError('Empty JPEG frame')
        if not 0 < width <= 8192 or not 0 < height <= 8192:
            raise ValueError('Invalid image dimensions')
        image = np.empty((height, width, 3), dtype=np.uint8)
        message = ctypes.create_string_buffer(512)
        status = self.function(data, len(data), image.ctypes.data, image.nbytes,
                               width, height, message, len(message))
        if status:
            reason = message.value.decode('utf-8', errors='replace')
            if status == 1:
                raise CorruptJpegError(reason)
            raise RuntimeError(reason or 'Strict JPEG decoder failed')
        return image
