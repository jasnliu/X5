"""start_beat startup selection; sysfs listing never opens cameras or motors."""
import argparse
import json
from pathlib import Path
import sys

from camera_search.device import CAMERA_CONFIG, DEFAULT_CAMERA, camera_argument, resolve_camera


def _read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return ''


def list_cameras(*, sysfs=Path('/sys/class/video4linux'), dev=Path('/dev'),
                 config_path=CAMERA_CONFIG):
    config = json.loads(Path(config_path).read_text())
    cameras = []
    for entry in Path(sysfs).glob('video[0-9]*'):
        if _read(entry/'index') != '0':
            continue  # Exclude UVC metadata nodes, not additional video inputs.
        interface = (entry/'device').resolve()
        usb = next((p for p in [interface, *interface.parents] if (p/'idVendor').is_file()), None)
        identity = {key: _read(usb/key) if usb else '' for key in ('idVendor', 'idProduct', 'serial')}
        matching_device = all(identity[key] == config[field] for key, field in
                              [('idVendor', 'vendor_id'), ('idProduct', 'product_id'), ('serial', 'serial')])
        default = matching_device and _read(interface/'bInterfaceNumber') == config['usb_interface']
        node = Path(dev)/entry.name
        aliases = []
        for kind in ('by-id', 'by-path'):
            for alias in sorted((Path(dev)/'v4l'/kind).glob('*')):
                if alias.resolve() == node.resolve():
                    aliases.append(str(alias))
        cameras.append(dict(index=int(entry.name.removeprefix('video')), device=str(node),
                            name=_read(entry/'name') or entry.name, serial=identity['serial'],
                            default=default, non_rgb=matching_device and not default,
                            aliases=aliases))
    return sorted(cameras, key=lambda row: row['index'])


def print_cameras(cameras, *, output=print):
    output('Available local video inputs (listing only; no streams or arm controllers opened):')
    if not cameras:
        output('  No capture devices found. Connect a camera; close OrbbecViewer if using Gemini.')
    for row in cameras:
        suffix = (' [Orbbec RGB default: --camera orbbec]' if row['default'] else
                  ' [Orbbec depth/IR, not the RGB input]' if row['non_rgb'] else '')
        output(f"  {row['index']}: {row['name']} — {row['device']}{suffix}")
        if row['serial']:
            output(f"     Serial: {row['serial']}")
        # Show one preferred stable path, not all aliases for the same device.
        if row['aliases'] and not row['default']:
            output(f"     Stable path: {row['aliases'][0]}")
    output('Select with --camera N or --camera /dev/v4l/by-id/...; '
           'use --select-camera for an interactive startup picker.')


def choose_camera(*, cameras=None, input_fn=None, output=print):
    """Resolve a user's selection before launch; never switch a running arm's camera."""
    if not sys.stdin.isatty():
        raise RuntimeError('--select-camera needs an interactive terminal; use --list-cameras then --camera SOURCE')
    cameras = list_cameras() if cameras is None else cameras
    print_cameras(cameras, output=output)
    if not cameras:
        raise RuntimeError('No camera available; no controllers were launched')
    input_fn = input if input_fn is None else input_fn
    while True:
        try:
            answer = input_fn('Camera index/path (Enter=orbbec, q=cancel): ').strip()
        except (EOFError, KeyboardInterrupt):
            raise SystemExit('Camera selection cancelled; no controllers were launched') from None
        if answer.lower() in ('q', 'quit', 'cancel'):
            raise SystemExit('Camera selection cancelled; no controllers were launched')
        try:
            source = camera_argument(answer or DEFAULT_CAMERA)
            index = resolve_camera(source)
            row = next((r for r in cameras if r['index'] == index), None)
            if row is None:
                raise ValueError(f'/dev/video{index} is not one of the listed capture devices')
        except (argparse.ArgumentTypeError, RuntimeError, ValueError) as exc:
            output(f'Invalid camera selection: {exc}')
            continue
        if row['default']:
            source = DEFAULT_CAMERA  # Gemini by-id links can alias depth/IR; retain identity selector.
        elif isinstance(source, int) and row['aliases']:
            source = row['aliases'][0]
        output(f"Selected camera: {row['name']} ({source})")
        return source
