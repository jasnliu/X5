"""Finite snare dataset, with the original ride collector's five depths."""
import random

CLOSE_DEGREES = 90
SNARE_DEGREES = 11.
SNARE_MAX_DEGREES = 14.
OPEN_AFTER_SECONDS = .6
CLOSURE_INTERVAL_SECONDS = 1.2
OPEN_SETTLE_SECONDS = 1.5
COUNTS = dict(snare_only=20, snare_ride=8, snare_hihat=8, snare_both=4,
              ride_only=6, hihat_only=6, ride_hihat=4, background=4)
TEST_COUNTS = dict(snare_only=4, snare_ride=2, snare_hihat=1, snare_both=1,
                   ride_only=1, hihat_only=1, ride_hihat=1, background=1)


def make_plan(seed=42):
    rows = []
    ride_index = 0
    for kind, count in COUNTS.items():
        for index in range(count):
            test = index < TEST_COUNTS[kind]
            snare = kind.startswith('snare_')
            double = kind == 'snare_only' and index in (0, 4, 5, 6, 7)
            ride = 'ride' in kind or kind == 'snare_both'
            hihat = 'hihat' in kind or kind == 'snare_both'
            depth = (10., 10.5, 11., 11.5, 12.)[ride_index % 5] if ride else None
            if ride: ride_index += 1
            rows.append(dict(kind=kind, split='test' if test else 'development',
                snare_hits=(2 if double else 1) if snare else 0,
                snare_degrees=SNARE_DEGREES if snare else None,
                ride_hits=int(ride), ride_depth_deg=depth, hihat_closures=int(hihat),
                ride_offset_s=(-.35, 0., .25)[index % 3],
                hihat_offset_s=(0., -.3, .25)[index % 3],
                duration_s=7.5 if double else 6.5))
    rng = random.Random(seed)
    rng.shuffle(rows)
    rows.sort(key=lambda r: r['split'] == 'test')
    for block, row in enumerate(rows, 1): row['block'] = block
    assert len(rows) == 60 and sum(r['snare_hits'] for r in rows) == 45
    return rows


def cycle_commands(start, closures):
    if closures not in (1, 2):
        raise ValueError('Only one or two hi-hat closures per block')
    return [(start+i*CLOSURE_INTERVAL_SECONDS+delta, command)
            for i in range(closures) for delta, command in ((0., b'B'), (OPEN_AFTER_SECONDS, b'O'))]
