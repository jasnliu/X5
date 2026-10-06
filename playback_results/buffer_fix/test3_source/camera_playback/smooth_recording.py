"""Recording-only adapter for playback.sh's physically tested paced_precise method.

No centering, gripper, alignment, strike, enable, or relax workflow lives here.
The GUI owns the CAN locks and gripper; a spawned worker temporarily owns only
right joints 1-7's CSP targets, speed limits, and volatile position gains.
"""
from dataclasses import dataclass, fields
import hashlib
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import PPoly, PchipInterpolator, make_interp_spline
from scipy.ndimage import gaussian_filter1d

from .trajectory import PlaybackTrajectory, load_playback_trajectory
from smooth_playback.trajectory import Geometry, make_trajectory, validate
from smooth_playback.retime_prototype import fit


RECORD3_DURATION_RATIO = 4.8 / 3.841315
JOINT_SPEEDS = [.4, .4, .4, .8, .4, .4, .4]


@dataclass(frozen=True)
class SmoothRecording(PlaybackTrajectory):
    smooth_motion: object
    geometry: object

    def joints_at(self, elapsed):
        return self.smooth_motion.at(elapsed), elapsed >= self.duration_s


def load_smooth_recording(path, model, zone, lower, upper, center, speed,
                          playback_speed=.8, progress=None):
    """Keep the selected file and existing preflight, then fit the tested method.

    The timing ratio is fixed to the tested record3 ratio, not an arbitrarily
    slow global stretch. Unsafe/unfit recordings fail preflight before enable.
    Simulation continues using the original loader and follower unchanged.
    """
    old = load_playback_trajectory(path, model, zone, lower, upper, center, speed,
                                   playback_speed=playback_speed, progress=progress)
    g = Geometry.__new__(Geometry)
    g.model, g.zone = model, zone
    g.lower, g.upper, g.center = map(np.array, (lower, upper, center))
    duration = old.original_duration_s
    raw_t, raw_q = old.source_times, old.joints
    if progress:
        progress("Fitting playback.sh's corridor-constrained 200 Hz trajectory")
    if duration <= 0:
        # The existing selector accepts single-frame recordings. Preserve that
        # contract with an exact stationary target (no divide-by-zero/LP).
        curves = [PPoly(np.array([[float(v)]]), [0., 1.]) for v in old.first_joints]
        motion = make_trajectory('paced_precise', curves, 0., old.first_joints,
                                 old.last_joints, RECORD3_DURATION_RATIO)
        motion.metadata.update(command_hz=200, position_gain_cap=10., precise_endpoint=True,
            firmware_joint_speeds_rad_s=JOINT_SPEEDS.copy(),
            source_sha256=hashlib.sha256(old.source.read_bytes()).hexdigest(),
            **validate(motion, g, raw_t, raw_q))
        data = {f.name: getattr(old, f.name) for f in fields(PlaybackTrajectory)}
        data['time_scale'] = RECORD3_DURATION_RATIO
        return SmoothRecording(**data, smooth_motion=motion, geometry=g)
    grid = np.linspace(0, duration, int(np.ceil(duration/.01))+1)
    values = np.column_stack([np.interp(grid, raw_t, raw_q[:, j]) for j in range(7)])
    audit = []
    base = None
    for sigma in (.14, .10, .07, .04):
        filtered = gaussian_filter1d(values, sigma/(grid[1]-grid[0]), axis=0, mode='nearest')
        knots = np.linspace(0, duration, max(2, int(np.ceil(duration/.20))+1))
        q = np.column_stack([np.interp(knots, grid, filtered[:, j]) for j in range(7)])
        q[0], q[-1] = old.first_joints, old.last_joints
        curves = [PPoly.from_spline(make_interp_spline(
            knots, q[:, j], k=5, bc_type=([(1, 0.), (2, 0.)], [(1, 0.), (2, 0.)])))
            for j in range(7)]
        candidate = make_trajectory('smooth', curves, duration, old.first_joints, old.last_joints)
        try:
            validate(candidate, g, raw_t, raw_q)
        except ValueError as exc:
            audit.append(dict(sigma=sigma, accepted=False, reason=str(exc)))
        else:
            audit.append(dict(sigma=sigma, accepted=True))
            base = candidate
            break
    if base is None:
        raise ValueError(f'No smooth path stayed within the original recording corridor: {audit}')
    if np.max(np.ptp(raw_q, axis=0)) < 1e-12:
        # A stationary recording has no arc length to retime.
        motion = make_trajectory('paced_precise', base.curves, duration,
                                 base.first, base.last, RECORD3_DURATION_RATIO)
    else:
        curves, times, phase = fit(base, amax=1., vmax=[.35, .35, .35, .70, .35, .35, .35],
                                   smoothing=2, edge=.08)
        # Normalize the polynomial domain to the source duration. validate()
        # includes raw source timestamps in its check grid; they must not fall
        # beyond the retimed polynomial domain for longer/paused recordings.
        factor = duration/float(times[-1])
        curves = [PPoly(p.c / factor**np.arange(p.c.shape[0]-1, -1, -1)[:, None],
                        p.x*factor) for p in curves]
        motion = make_trajectory('paced_precise', curves, duration,
                                 base.first, base.last, RECORD3_DURATION_RATIO)
        motion.reference_map = PchipInterpolator(times*factor, phase)
    for key, limit in zip(('peak_velocity_rad_s', 'peak_acceleration_rad_s2',
                           'peak_piecewise_jerk_rad_s3'), (.72, 1.5, 40.)):
        if motion.metadata[key] > limit:
            raise ValueError(f'Smooth playback at near-original speed exceeds {key}: '
                             f'{motion.metadata[key]:.3f} > {limit}')
    motion.metadata.update(validate(motion, g, raw_t, raw_q),
        command_hz=200, position_gain_cap=10., precise_endpoint=True,
        firmware_joint_speeds_rad_s=JOINT_SPEEDS.copy(), gaussian_sigma_s=sigma,
        limits_rad=[.72, 1.5, 40.], acceleration_continuous_into_hold=True,
        smoothing_search=audit, original_recording_duration_s=duration,
        recording_duration_ratio=RECORD3_DURATION_RATIO,
        source_sha256=hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest())
    # Existing preflight checked original center returns; check the new curve too.
    g.check_line(g.center, motion.first)
    for t in np.linspace(0, motion.duration, 31):
        g.check_line(motion.at(t), g.center)
    data = {f.name: getattr(old, f.name) for f in fields(PlaybackTrajectory)}
    data['time_scale'] = RECORD3_DURATION_RATIO
    return SmoothRecording(**data, smooth_motion=motion, geometry=g)
