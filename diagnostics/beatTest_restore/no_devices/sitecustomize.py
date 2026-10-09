"""Offline verification guard only; never used by the normal launcher."""
import os
import socket
import sys


def forbid_physical_devices(event, args):
    if event == 'socket.__new__' and args[1] == socket.AF_CAN:
        raise AssertionError('Restoration verification forbids physical CAN')
    if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        path = os.fsdecode(args[0])
        if path.startswith(('/dev/tty', '/dev/video', '/dev/snd', '/dev/bus/usb')):
            raise AssertionError('Restoration verification forbids physical devices')


sys.addaudithook(forbid_physical_devices)
