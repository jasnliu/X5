"""All data diversity is specified before enabling a motor."""
import random

DEPTHS = (10., 10.5, 11., 11.5, 12.)
RINGDOWN_SECONDS = 6.
TONOR = 'alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo'
CSV_FIELDS = ('recording_file', 'hit_time_seconds', 'label')


def make_plan(seed=42):
    rng = random.Random(seed)
    rows = []
    # First encounter of each depth is isolated, before any partial returns.
    for repetition in range(6):
        depths = list(DEPTHS)
        rng.shuffle(depths)
        for depth in depths:
            count = 1 if repetition < 4 else 2
            rows.append(dict(block=len(rows)+1, depth_deg=depth, strikes=count,
                             kind='isolated' if count == 1 else 'double',
                             pair_interval_s=.2 if count == 2 else None,
                             ringdown_s=RINGDOWN_SECONDS))
    return rows
