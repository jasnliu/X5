"""Finite, unchanged swing close/open cycles; ride overlap is still positive."""
import random

CLOSE_DEGREES = 100
OPEN_AFTER_SECONDS = .6
CLOSURE_INTERVAL_SECONDS = 1.2
OPEN_SETTLE_SECONDS = 1.5


def make_plan():
    rng = random.Random(42)
    counts = [1]*16+[2]*4
    rng.shuffle(counts)
    clean = [dict(block=i+1, kind='hihat_only', closures=n) for i,n in enumerate(counts)]
    depths = [10.,10.5,11.,11.5,12.]*2
    offsets = [-.4,-.2,0.,.2,.4]*2
    pairs = list(zip(depths,offsets));rng.shuffle(pairs)
    mixed = [dict(block=i+21,kind='mixed',closures=1,strikes=1,depth_deg=d,
                  ride_command_offset_s=o) for i,(d,o) in enumerate(pairs)]
    return dict(clean=clean,mixed=mixed,background_count=10,background_seconds=6.,
                ride_negative_ids=[f'r{i}' for i in (*range(1,13),*range(21,29))])


def cycle_commands(start, closures):
    if closures not in (1,2):
        raise ValueError('Only one or two hi-hat closures per block')
    return [(start+i*CLOSURE_INTERVAL_SECONDS+delta, command)
            for i in range(closures) for delta,command in ((0.,b'C'),(OPEN_AFTER_SECONDS,b'O'))]
