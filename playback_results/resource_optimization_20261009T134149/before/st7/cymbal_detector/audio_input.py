"""Share the named TONOR through PipeWire instead of opening raw ALSA at 16 kHz."""
from contextlib import contextmanager
import json
import os
import subprocess
import sys


TONOR_SOURCE = (
    'alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo'
)


def find_tonor_source() -> str:
    try:
        result = subprocess.run(
            ['pactl', 'list', 'short', 'sources'], check=True, text=True,
            capture_output=True, timeout=5,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError('Cannot select TONOR through PipeWire; check that PipeWire '
                           'and pactl are available.') from exc
    sources = [parts[1] for line in result.stdout.splitlines()
               if len(parts := line.split()) >= 2
               and 'tonor' in parts[1].lower() and not parts[1].endswith('.monitor')]
    if TONOR_SOURCE in sources:
        return TONOR_SOURCE
    if len(sources) == 1:
        return sources[0]
    if not sources:
        raise RuntimeError('No TONOR microphone capture source is connected to PipeWire.')
    raise RuntimeError('Multiple TONOR capture sources found; explicitly select a source '
                       'with PIPEWIRE_PROPS/PULSE_SOURCE and --device pipewire.')


@contextmanager
def microphone_input(device):
    """Select only this process's stream; never change the desktop default source.

    Explicit numeric/other device selections and X5's already-routed `pipewire`
    streams are unchanged. Enter before importing sounddevice/initializing audio.
    """
    if not (sys.platform.startswith('linux') and isinstance(device, str)
            and 'tonor' in device.lower()):
        yield device, None
        return
    source = find_tonor_source()
    overrides = {
        'PULSE_SOURCE': source,
        'PIPEWIRE_PROPS': json.dumps({'application.name': 'ST7-TONOR-detector',
                                      'target.object': source}),
    }
    previous = {key: os.environ.get(key) for key in overrides}
    os.environ.update(overrides)
    try:
        yield 'pipewire', source
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
