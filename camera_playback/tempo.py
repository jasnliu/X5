"""Beat tempo only: never scale calibration, playback or individual motor strokes."""
import math

DEFAULT_BPM = 100.0
MIN_BPM = 20.0
MAX_BPM = 180.0
# Engineering estimate, not a physical safety/success guarantee. The shortest
# ride gap is 20/BPM seconds; the existing partial rebound alone takes 110 ms,
# before allowing for the next downstroke/catch and real feedback overhead.
ESTIMATED_PRACTICAL_MAX_BPM = 120.0

CONTINUOUS_WAIT_PHASE = 'SWING STRIKE WAITING'
CONTINUOUS_OUT_PHASE = 'SWING STRIKE MOVING OUT'
CONTINUOUS_RETURN_PHASE = 'SWING STRIKE RETURNING'


def parse_bpm(value):
    try:
        bpm = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f'Enter a BPM from {MIN_BPM:g} to {MAX_BPM:g}') from None
    if isinstance(value, bool) or not math.isfinite(bpm) or not MIN_BPM <= bpm <= MAX_BPM:
        raise ValueError(f'Enter a finite BPM from {MIN_BPM:g} to {MAX_BPM:g}')
    return bpm


def swing_events(bpm=DEFAULT_BPM):
    """Same six-event triplet bar, scaled by the selected quarter-note BPM."""
    period = 60.0 / parse_bpm(bpm)
    triplet = period / 3.0
    return (
        ('beat 1', True, period),
        ('beat 2', True, 2.0 * triplet),
        ('extra after beat 2', False, triplet),
        ('beat 3', True, period),
        ('beat 4', True, 2.0 * triplet),
        ('extra after beat 4', False, triplet),
    )
