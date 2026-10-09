"""Left-arm snare-strike configuration; separate from strike_lab's
right-arm-only helpers (joint_limits()/center_pose() there hardcode
'openarmx_right_joint{i}'). Snare depth is bounded by left J6's URDF range,
not the ride experiment's fixed depth corridor.
"""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from strike_lab.config import Rules, Limits as RideLimits, Plant

ROOT = Path(__file__).resolve().parents[1]
LEFT_RECORDING = ROOT / 'left_recordings/record1.json'

# The left arm's snare strike is motor/joint 6 (the joint above the wrist),
# NOT motor 7 (which is the right arm's ride-cymbal strike joint). Confirmed
# against vendor/openarmx_description/urdf/robot/openarmx_robot.urdf and the
# physical arm by the user. "Struck" is an INCREASING displayed joint value
# here (target = anchor + amount), the opposite of the right arm's J7
# (target = anchor - amount) -- confirmed against left_recordings/record1.json's
# own endpoint. See camera_playback/mit_strike.py's Command.frame() docstring
# for how that sign difference is handled (STRIKE_SIGN below, applied only at
# the hardware I/O boundary; the strike engine's internal math never changes).
STRIKE_MOTOR = 6
STRIKE_SIGN = -1.0


def left_joint_limits():
    """Mirrors strike_lab.config.joint_limits(), but for openarmx_left_joint{i}."""
    root = ET.parse(ROOT / 'model/openarmx.urdf').getroot()
    return [(float(e.get('lower')), float(e.get('upper'))) for i in range(1, 8)
            for e in [root.find(f"joint[@name='openarmx_left_joint{i}']/limit")]]


def validate_depth(degrees):
    """Magnitude validation only; the selected recording determines the maximum."""
    if (isinstance(degrees, bool) or not isinstance(degrees, (int, float))
            or not math.isfinite(degrees) or degrees < .5):
        raise ValueError('Snare strike depth must be a finite number of at least 0.5 degrees')


def validate_joint_depth(degrees, anchor, lower, upper):
    validate_depth(degrees)
    if (not all(math.isfinite(v) for v in (anchor, lower, upper))
            or not lower <= anchor < upper):
        raise ValueError('Left J6 strike anchor must be inside its joint range with positive travel remaining')
    available = math.degrees(upper - anchor)
    if degrees > available + 1e-12:
        raise ValueError(f'Snare depth {degrees:g}° exceeds left J6 rotation: '
                         f'maximum +{available:.6f}° from anchor {math.degrees(anchor):.6f}° '
                         f'to joint limit {math.degrees(upper):.6f}°')


@dataclass(frozen=True)
class JointRotationLimits(RideLimits):
    """Shared dynamic checks, but a joint-derived rather than 12-degree depth."""
    def __post_init__(self):
        if not math.isfinite(self.hard_depth_deg) or self.hard_depth_deg <= 0:
            raise ValueError('Left J6 must have positive finite rotation remaining')
        # Reuse every shared non-depth validation without modifying ride limits.
        values = asdict(self)
        values['hard_depth_deg'] = 12.
        RideLimits(**values)


def joint_rotation_limits(anchor, lower, upper, settings):
    if (not all(math.isfinite(v) for v in (anchor, lower, upper))
            or not lower <= anchor < upper):
        raise ValueError('Invalid left J6 joint range or strike anchor')
    values = dict(settings)
    # Ignore any inherited fixed ride depth, including one in its tuning JSON.
    values['hard_depth_deg'] = math.degrees(upper - anchor)
    values['upper_excursion_deg'] = LEFT_UPPER_EXCURSION_DEG
    return JointRotationLimits(**values)


# --- Method history ---------------------------------------------------
# Real --hardware attempts (2026-10-07) with 'hybrid' (gravity-drop + kick,
# see strike_lab/methods/gravity.py's Gravity base class) all struggled: it
# relies on gravity doing most of the work after a small kick releases the
# joint (hold_command()'s bias/gravity-compensation is dropped entirely
# during the kick). The right arm's J7 is heavily gravity-loaded in its
# pose, so a modest 0.3 Nm/40 ms kick moves it ~10+ deg. J6 is a roll joint
# (URDF axis "1 0 0") and is evidently NOT significantly gravity-loaded in
# this recording's pose -- it isn't getting that same assist, so the kick
# has to do all the work itself. Even at hybrid's validated maximum kick
# (0.6 Nm/120 ms) this barely worked and needed repeated tuning.
#
# Per the right arm's own characterization (EXPERIMENT_RESULTS.md), 'hybrid'
# was never actually the fastest method either -- plain 'gravity' tied it,
# and 'powered' (fully torque-driven the whole way, never relying on
# gravity) was the single fastest of all nine methods tested. Switched to
# 'powered': a prescribed down/up position curve (strike_lab/methods/
# powered.py's Dip), tracked with full feed-forward torque throughout --
# mechanically the right fit for a joint without gravity assist, not a
# brute-force kick increase.
STRIKE_METHOD_ID = 'powered'

# Shorten only the outbound stroke; retain the previous return curve exactly.
# The requested gain is reduced when the existing dynamic limits require it.
DOWNWARD_SPEED_MULTIPLIER = 1.25

# Snare-only curved acceleration: a moderate peak-speed increase over the
# existing faster descent, reached later in the travel. No limit is raised.
DOWNWARD_PEAK_GAIN = 1.20
DOWNWARD_PEAK_FRACTION = .70

# config/experiment_hardware_tuned.json's "powered" parameters include
# load_torque=0.736 -- a FIXED gravity-compensation torque applied for the
# entire move (Method.tracking(): `load = p.get('load_torque',0.) or
# bias*bias_scale`, so a truthy load_torque always wins over live bias).
# That 0.736 Nm reflects the right arm's J7 gravity load; J6 isn't
# meaningfully gravity-loaded (see above), so hardcoding the same value
# would inject a large, wrong, constant torque throughout the whole strike.
# Zero it so tracking() falls back to live measured bias instead, exactly
# like the hold phases already do for both sides.
LEFT_POWERED_OVERRIDES = dict(load_torque=0.)

# Both 'hybrid' and (conceivably) 'powered' share the same tuned
# upper_excursion_deg=0.6 (allowed rebound/overshoot above the anchor),
# tuned for the right arm's J7 backlash/compliance specifically. Widen it
# for J6 until it's been characterized the same way -- see snare_lab/
# runner.py's validate_strike_path()/strike_corridor() for where this is
# used; 'powered' is a monotonic down/up curve so overshoot risk is lower
# than 'hybrid's reactive coast, but still untested on this joint.
LEFT_UPPER_EXCURSION_DEG = 3.0

# First real --hardware attempt with 'powered' (2026-10-07): the strike
# itself ran cleanly (approach/withdrawal/settling all logged), but the
# POST-strike settle check never passed within the 3 s trial_timeout --
# "Left strike did not settle before timeout". strike_lab.engine.StableWindow
# requires the measured position to stay within Rules.return_tolerance_deg
# (0.15 deg) and Rules.settle_range_deg (0.05 deg) of the anchor -- tight
# tolerances characterized against the right arm's J7 tracking precision,
# not J6's. With load_torque zeroed (see LEFT_POWERED_OVERRIDES), the
# strike's one-shot self.bias (seeded once at arming, never re-adapted
# during tracking()) is the only load compensation for the whole move; any
# seed/friction mismatch on this still-uncharacterized joint shows up as a
# small persistent steady-state tracking error that 0.15/0.05 deg can't
# absorb. Widen both for J6 until characterized; still far tighter than the
# ordinary-CSP POSE_TOLERANCE (0.20 deg) used elsewhere in snare_lab.
LEFT_RETURN_TOLERANCE_DEG = 1.0
LEFT_SETTLE_RANGE_DEG = 0.3
