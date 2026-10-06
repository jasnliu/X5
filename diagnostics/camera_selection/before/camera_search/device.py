"""Shared camera selection; imports and argument parsing never open hardware."""
import argparse
import json
from pathlib import Path


DEFAULT_CAMERA = 'orbbec'
CAMERA_CONFIG = Path(__file__).resolve().parents[1]/'config/camera.json'


def camera_argument(value):
    """Keep numeric --camera overrides, plus a stable default camera selector."""
    if value == DEFAULT_CAMERA:
        return value
    try:
        number = int(value)
        if number >= 0:
            return number
    except (TypeError, ValueError):
        pass
    raise argparse.ArgumentTypeError('camera must be "orbbec" or a nonnegative camera index')


def add_camera_argument(parser):
    parser.add_argument('--camera', type=camera_argument, default=DEFAULT_CAMERA,
                        help='orbbec (default: configured Gemini 2 RGB), or an explicit numeric index')


def resolve_camera(source=DEFAULT_CAMERA, *, sysfs=Path('/sys/class/video4linux'),
                   config_path=CAMERA_CONFIG):
    """Find the configured RGB interface by USB identity, not enumeration order.

    Gemini depth/IR nodes can advertise MJPEG too, so a generic first-Orbbec or
    first-MJPEG match is insufficient. Interface 04 is this camera's RGB stream;
    index 0 selects capture rather than its paired metadata node. The by-id link
    is also ambiguous across this device's multiple interfaces.
    """
    source = camera_argument(source)
    if source != DEFAULT_CAMERA:
        return source
    config = json.loads(Path(config_path).read_text())
    found = []
    for entry in Path(sysfs).glob('video[0-9]*'):
        try:
            interface = (entry/'device').resolve()
            usb = interface.parent
            if ((entry/'index').read_text().strip() != '0'
                    or (interface/'bInterfaceNumber').read_text().strip() != config['usb_interface']):
                continue
            if all((usb/key).read_text().strip() == config[field] for key, field in
                   [('idVendor', 'vendor_id'), ('idProduct', 'product_id'), ('serial', 'serial')]):
                found.append(int(entry.name.removeprefix('video')))
        except (FileNotFoundError, NotADirectoryError):
            continue  # Includes a camera disconnected during enumeration.
    if len(found) != 1:
        raise RuntimeError(
            f"{config['name']} ({config['serial']}) RGB capture interface not uniquely available. "
            'Connect that camera and close OrbbecViewer/other camera programs. '
            'No fallback camera was selected; use --camera N for an explicit override.')
    return found[0]


def is_configured_camera(index):
    """A numeric override pointing to this Gemini must keep its full FOV too."""
    try:
        return resolve_camera(DEFAULT_CAMERA) == index
    except (RuntimeError, FileNotFoundError):
        return False


def full_fov_size(width=None, height=None):
    """Only verified native 16:9 modes preserve this Gemini's entire RGB view."""
    config = json.loads(CAMERA_CONFIG.read_text())
    if width is None and height is None:
        width, height = config['rgb_width'], config['rgb_height']
    elif width is None:
        width = height*16//9
    elif height is None:
        height = width*9//16
    if [width, height] not in config['rgb_full_fov_sizes'] or width*9 != height*16:
        choices = ', '.join(f'{w}x{h}' for w, h in config['rgb_full_fov_sizes'])
        raise ValueError(f'Orbbec maximum RGB FOV requires a verified 16:9 mode: {choices}. '
                         f'{width}x{height} is cropped or unsupported; no narrow-FOV fallback is allowed.')
    return width, height


def open_camera(source=DEFAULT_CAMERA, *, cv2=None, width=None, height=None, fps=30):
    """Open the selected color stream; return capture and resolved V4L index."""
    if cv2 is None:
        import cv2
    index = resolve_camera(source)
    is_orbbec = source == DEFAULT_CAMERA or is_configured_camera(index)
    if is_orbbec:
        width, height = full_fov_size(width, height)
    camera = (cv2.VideoCapture(index, cv2.CAP_V4L2) if is_orbbec
              else cv2.VideoCapture(index))
    try:
        if not camera.isOpened():
            raise RuntimeError(f'Could not open camera /dev/video{index}; close other camera programs')
        if is_orbbec:
            # Native color MJPEG fits the currently connected USB 2 link at 30 FPS.
            if not camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')):
                raise RuntimeError('Orbbec RGB camera did not accept its native MJPEG format')
        if width:
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not camera.set(cv2.CAP_PROP_FPS, fps):
            print(f'Warning: camera did not accept the {fps} FPS request.', flush=True)
        if is_orbbec:
            actual_size = (int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if actual_size != (width, height):
                raise RuntimeError(f'Orbbec full-FOV mode {width}x{height} was not accepted '
                                   f'(received {actual_size[0]}x{actual_size[1]}); refusing cropped capture')
        print(f'Camera: {source} -> /dev/video{index}'
              + (f' (Orbbec Gemini 2 RGB, full FOV, {width}x{height})' if is_orbbec else ''), flush=True)
        return camera, index
    except BaseException:
        camera.release()
        raise
