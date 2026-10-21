"""Refuse heavy perception startup when Linux already has too little headroom."""
from pathlib import Path


MIN_AVAILABLE_BYTES = 2 * 1024**3


def check_memory(path=Path('/proc/meminfo')):
    # MemAvailable includes reclaimable cache; MemFree alone is misleading.
    # This is a pre-launch check only, never a reason to kill an active motor
    # controller or relax an arm. Swap is not counted as real-time headroom.
    try:
        lines = path.read_text().splitlines()
    except FileNotFoundError:
        return  # Non-Linux development hosts have no /proc/meminfo.
    for line in lines:
        if line.startswith('MemAvailable:'):
            available = int(line.split()[1]) * 1024
            if available < MIN_AVAILABLE_BYTES:
                raise RuntimeError(
                    f'Not enough available RAM to safely start beat perception: '
                    f'{available/1024**3:.2f} GiB available; at least 2 GiB required. '
                    'Close memory-heavy applications and retry. No controllers started.')
            return
    raise RuntimeError('Cannot verify available RAM before beat startup')
