"""Shared experiment contract, separate from all beat modes."""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TARGET_DEG = 10.0
ZONE_DEG = 9.0
TARGET = math.radians(TARGET_DEG)
METHOD_VERSION = 1


def joint_limits():
    root = ET.parse(ROOT / 'model/openarmx.urdf').getroot()
    return [(float(e.get('lower')), float(e.get('upper'))) for i in range(1, 8)
            for e in [root.find(f"joint[@name='openarmx_right_joint{i}']/limit")]]


def center_pose():
    return [0.0]*6 + [joint_limits()[6][1]]


@dataclass(frozen=True)
class Rules:
    acceptance_deg: float = .2
    minimum_entry_speed: float = .20  # rad/s; initial common screening threshold.
    maximum_approach_seconds: float = .8
    return_tolerance_deg: float = .15
    settle_seconds: float = .040
    settle_range_deg: float = .05
    maximum_feedback_gap: float = .010
    maximum_cycle_seconds: float = 1.5
    maximum_prezone_pause: float = .030
    maximum_upper_overshoot_deg: float = .15
    depth_match_deg: float = .10

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError('Scoring settings must be finite and positive')
        if self.settle_seconds < .04:
            raise ValueError('Settling must include at least 40 ms of encoder history')
        if self.acceptance_deg >= TARGET_DEG-ZONE_DEG:
            raise ValueError('Acceptance must stay strictly inside the 9-degree zone boundary')


@dataclass(frozen=True)
class Limits:
    hz: float = 500.
    velocity: float = 3.0
    acceleration: float = 80.
    jerk: float = 8000.
    torque: float = 3.0
    torque_slew: float = 300.
    hard_depth_deg: float = 12.
    upper_excursion_deg: float = .2
    held_joint_drift_deg: float = .75
    feedback_timeout: float = .020
    control_timeout: float = .020
    trial_timeout: float = 3.

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError('Motion limits must be finite and positive')
        if not 100 <= self.hz <= 1000 or self.torque > 12 or self.velocity > 10:
            raise ValueError('Unsupported experiment limits')
        if not 10.2 < self.hard_depth_deg <= 15:
            raise ValueError('Hard depth must bound the ten-degree experiment corridor')


@dataclass(frozen=True)
class Plant:
    inertia: float = .02
    gravity: float = .16
    friction: float = .02
    delay: float = .004
    encoder_step: float = 25.14/65535

    def __post_init__(self):
        if not all(math.isfinite(v) and v >= 0 for v in asdict(self).values()) or self.inertia <= 0:
            raise ValueError('Invalid synthetic plant')
