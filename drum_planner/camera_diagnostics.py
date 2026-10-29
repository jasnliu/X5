"""Read-only startup diagnostics for the planner's native-depth bridge.

Never opens a camera, resets USB, signals another process or changes permissions.
Device paths are resolved afresh by calibrated serial, not saved USB/video IDs.
"""
import os
from pathlib import Path
import re


def camera_holders(serial, *, ignored_pids=(), sys_root=Path('/sys'),
                   proc_root=Path('/proc'), dev_root=Path('/dev')):
    """Best-effort list of processes with handles to this camera (not a lock).

    A handle alone does not prove exclusive ownership. Only call this to add
    context to an actual SDK busy error; never use it as a capture veto.
    """
    nodes = set()
    try:
        devices = list((sys_root / 'bus/usb/devices').iterdir())
    except OSError:
        return []
    for device in devices:
        try:
            if (device / 'serial').read_text().strip() != serial:
                continue
            bus = int((device / 'busnum').read_text())
            number = int((device / 'devnum').read_text())
            nodes.add(str(dev_root / f'bus/usb/{bus:03d}/{number:03d}'))
            usb_path = device.resolve()
            # OpenCV/V4L2 users hold /dev/videoN instead of the raw USB node.
            for video in (sys_root / 'class/video4linux').glob('video*'):
                if usb_path in (video / 'device').resolve().parents:
                    nodes.add(str(dev_root / video.name))
        except (OSError, ValueError, RuntimeError):
            continue  # Device may disappear while diagnosing the failure.
    if not nodes:
        return []
    ignored = {os.getpid(), *ignored_pids}
    holders = []
    try:
        processes = sorted((p for p in proc_root.iterdir() if p.name.isdecimal()),
                           key=lambda p: int(p.name))
    except OSError:
        return []
    for process in processes:
        pid = int(process.name)
        if pid in ignored:
            continue
        try:
            with os.scandir(process / 'fd') as entries:
                matched = False
                for entry in entries:
                    try:
                        if os.readlink(entry.path) in nodes:
                            matched = True
                            break
                    except OSError:
                        continue
            if not matched:
                continue
            try:
                name = os.readlink(process / 'exe')
            except OSError:
                name = (process / 'comm').read_text().strip()
            # No command-line arguments: they may contain unrelated secrets.
            name = ''.join(c if c.isprintable() else '?' for c in name)[:300]
            holders.append(f'PID {pid}: {name}')
            if len(holders) == 8:
                break
        except OSError:
            continue  # Other users' processes may not be inspectable.
    return holders


def failure_message(error, folder, serial, *, ignored_pids=()):
    """Keep the parser/validation error and expose the bridge's real cause."""
    log = Path(folder).resolve() / 'depth_stderr.log'
    detail = ''
    try:
        with log.open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - 4096))
            lines = stream.read().decode('utf-8', errors='replace').splitlines()
        detail = ' | '.join(line.strip() for line in lines[-8:] if line.strip())[-1600:]
        detail = ''.join(c if c.isprintable() else '?' for c in detail)
    except OSError:
        pass
    message = str(error)
    if re.search(r'uvc_open.*Return Code:\s*-6\b', detail, re.IGNORECASE):
        message = (f'Camera {serial} is busy (UVC error -6). Close the other camera '
                   'application, then rerun plan_drums to acquire a NEW scene. '
                   'No other process was stopped and no saved scene was substituted.')
        holders = camera_holders(serial, ignored_pids=ignored_pids)
        if holders:
            message += '\nProcesses with camera handles: ' + '; '.join(holders)
        else:
            message += '\nNo other owner was visible; the USB interface may be claimed by a driver or inaccessible process.'
        message += '\nStream error: ' + str(error)
    elif re.search(r'uvc_open.*Return Code:\s*-3\b', detail, re.IGNORECASE):
        message = (f'Camera {serial}: USB access denied (UVC error -3). Check camera '
                   'device permissions/udev rules; the planner did not change them.\n' + message)
    if detail:
        message += '\nCamera SDK: ' + detail
    return message + '\nCamera log: ' + str(log)
