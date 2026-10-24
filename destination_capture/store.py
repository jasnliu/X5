"""One-sample recording persistence with automatic, non-overwriting names."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import tempfile

from motion_recording.recording import MotionRecording, gripper_name, joint_names


def _check_joint_limits(state, model, side):
    """Use the default planner's bounds in saved encoder coordinates.

    The planner embeds right-arm calibration offsets in its model and maps
    limits as [lower - offset, upper - offset]. Left limits remain unchanged.
    Do not use the motion recorder's encoder tolerance: a moving planner joint
    must be inside the actual interval, and accepted measurements stay exact.
    """
    offsets = {}
    if side == 'right':
        from drum_planner.common import calibration
        cal = calibration()
        if cal['model_sha256'] != model.digest:
            raise ValueError('Capture and planner robot models differ; restart destination capture')
        offsets = cal['kinematic_correction']['offsets_rad']
    joints = {joint.get('name'): joint for joint in model.joints}
    for number, name in enumerate(joint_names(side), 1):
        limit = joints[name].find('limit')
        if limit is None:
            raise ValueError(f'Missing planner joint limits for {side.title()} J{number}')
        offset = float(offsets.get(name, 0.))
        lower = float(limit.get('lower')) - offset
        upper = float(limit.get('upper')) - offset
        if not (math.isfinite(lower) and math.isfinite(upper) and lower <= upper):
            raise ValueError(f'Invalid planner joint limits for {side.title()} J{number}')
        value = float(state[name])
        if not lower <= value <= upper:
            raise ValueError(
                f'{side.title()} J{number} out of range: {math.degrees(value):+.2f}°; '
                f'allowed {math.degrees(lower):+.2f}° to {math.degrees(upper):+.2f}°. '
                'Reposition the arm and capture again.')


class DestinationStore:
    def __init__(self, root, side):
        if side not in ('right', 'left'):
            raise ValueError('Choose right or left')
        self.side = side
        self.folder = Path(root).expanduser().resolve() / side
        self.prefix = 'destination' + ('R' if side == 'right' else 'L')
        self.pattern = re.compile(re.escape(self.prefix) + r'([1-9][0-9]*)\.json$')
        self.folder.mkdir(parents=True, exist_ok=True)

    def next_number(self):
        numbers = [int(m.group(1)) for p in self.folder.iterdir()
                   if (m := self.pattern.fullmatch(p.name))]
        return max(numbers, default=0) + 1

    def save(self, state, model, *, captured_at_utc, feedback_age_s,
             batch_duration_s, bus):
        """Keep measured joint coordinates exactly; no clipping or route generation.

        Reusing the recording schema makes the planner's existing endpoint loader
        compatible without altering it. There is exactly ONE sample, at time zero.
        """
        from safe_zone.geometry import LEFT_TCP, RIGHT_TCP

        names = (*joint_names(self.side), gripper_name(self.side))
        if not all(math.isfinite(float(state[name])) for name in names):
            raise ValueError('Destination feedback contains nonfinite values')
        # Reject before constructing or writing a recording. A failed capture
        # consumes no filename and leaves existing destinations untouched.
        _check_joint_limits(state, model, self.side)
        tcp_name = RIGHT_TCP if self.side == 'right' else LEFT_TCP
        take = MotionRecording(model.digest, self.side)
        take.start(captured_at_utc)
        take.add(0., state, model.transforms(state)[tcp_name][:3, 3])
        take.stop()
        payload = take.as_dict()
        payload.update(capture_kind='destination', captured_at_utc=captured_at_utc,
                       coordinate_convention='encoder_to_joint', tcp_frame=tcp_name,
                       tcp_reference_frame='world',
                       capture=dict(bus=bus, feedback_age_s=feedback_age_s,
                                    batch_duration_s=batch_duration_s,
                                    motor_state_commands=0))
        # Write and flush a complete JSON first. Hard-link publication is atomic
        # and refuses to overwrite even if another process captures concurrently.
        fd, temporary = tempfile.mkstemp(prefix='.destination-', suffix='.tmp', dir=self.folder)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2, allow_nan=False)
                handle.write('\n')
                handle.flush()
                os.fsync(handle.fileno())
            while True:
                path = self.folder / f'{self.prefix}{self.next_number()}.json'
                try:
                    os.link(temporary, path)
                    return path
                except FileExistsError:
                    continue
        finally:
            os.unlink(temporary)
