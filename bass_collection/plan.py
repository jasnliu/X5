"""The approved 60 positive bass clips; fixed K pulse, varied context/timing."""
from collections import Counter

PULSE = dict(command='K', direction='clockwise', duty=180, pwm_bits=8,
             pwm_hz=20000, duration_ms=50, return_mode='coast')
COUNTS = dict(bass_only=12, bass_hihat=12, bass_ride=12, bass_snare=8,
              bass_ride_hihat=8, bass_ride_hihat_snare=8)
SPLITS = ('train', 'validation', 'test')


def make_plan():
    rows = []
    for kind, count in COUNTS.items():
        for i in range(count):
            split = 'train' if i < count-4 else ('validation' if i < count-2 else 'test')
            single = kind == 'bass_only' and i % 2 == 0
            rows.append(dict(kind=kind, split=split, bass_hits=1 if single else 4,
                bpm=(80,100,120)[i % 3], offset_s=(0.,-.075,.075)[i % 3],
                ride_hits=2 if 'ride' in kind else 0, ride_depth_deg=10.5 if 'ride' in kind else None,
                snare_hits=2 if 'snare' in kind else 0,
                hihat_closures=2 if 'hihat' in kind else 0,
                duration_s=8., bass_pulse=dict(PULSE)))
    rows.sort(key=lambda r:SPLITS.index(r['split']))
    for block,row in enumerate(rows,1): row['block']=block
    assert Counter(r['kind'] for r in rows)==COUNTS
    assert Counter(r['split'] for r in rows)==dict(train=36,validation=12,test=12)
    return rows


def schedule(row, nominal):
    """Absolute request times. Acoustic labels must be determined separately."""
    beat=60./row['bpm']; offset=row['offset_s']
    accessory=[(nominal+i*beat,'K') for i in range(row['bass_hits'])]
    arms=[]
    # Two background hits, at beats 2 and 4, leaving full returns between strokes.
    for i in (1,3):
        at=nominal+i*beat+offset
        if row['hihat_closures']: accessory.extend(((at,'B'),(at+beat,'O')))
        if row['ride_hits']: arms.append((at,'ride'))
        if row['snare_hits']: arms.append((at,'snare'))
    return sorted(accessory),sorted(arms)
