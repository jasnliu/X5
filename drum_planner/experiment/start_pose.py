"""One-shot selected-arm encoder capture; never imported by planning workers.

The observer can only send state requests, and closes without changing torque.
The other arm is copied from an explicitly identified saved state, not queried.
"""
import argparse
import copy
from pathlib import Path
import time

import numpy as np

from ..common import DEFAULT_CALIBRATION, ROOT, atomic_json, digest, read_state


def capture(folder, side, base_state, calibration=DEFAULT_CALIBRATION):
    from safe_zone.encoder import SingleArmObserver
    from ..read_only import QuerySocket, _BusRecovered

    if side not in ('left', 'right'):
        raise ValueError('Choose left or right')
    base_state = Path(base_state).resolve()
    state = read_state(base_state)
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    bus = 'can1' if side == 'left' else 'can0'
    observer = gate = None
    samples = []
    error = None
    print(f'READ ONLY: {side} arm on {bus}; encoder requests only. No enable, motion, gripper or relax commands.', flush=True)
    try:
        with (folder/'can_query_audit.jsonl').open('x') as tx, (folder/'can_rx_audit.jsonl').open('x') as rx:
            try:
                observer = SingleArmObserver(side=side, bus=bus)
                gate = QuerySocket(observer.socket, bus, tx, rx)
                observer.socket = gate
                began = time.monotonic()
                while len(samples) < 20:
                    if time.monotonic()-began > 4.:
                        raise RuntimeError('Could not collect a stable selected-arm snapshot within four seconds')
                    start = time.monotonic()
                    try:
                        joints = observer.sample()
                    except _BusRecovered:
                        continue
                    end = time.monotonic()
                    feedback = [dict(gate.feedback[m]) for m in range(1, 9)]
                    if any(not start <= v['request_monotonic'] <= v['received_monotonic'] <= end
                           or v['mode'] != 0 or v['fault_bits'] for v in feedback):
                        raise RuntimeError('Incomplete, stale or enabled selected-arm feedback; no motor state was changed')
                    samples.append(dict(start=start, end=end, joints=dict(joints),
                                        motor8_feedback=joints.motor8_feedback, feedback=feedback))
                    time.sleep(max(0., .05-(time.monotonic()-start)))
            finally:
                if observer:
                    observer.close()
        names = [f'openarmx_{side}_joint{i}' for i in range(1, 8)]
        spread = np.degrees(np.ptp([[s['joints'][n] for n in names] for s in samples], axis=0))
        if not np.isfinite(spread).all() or max(spread) > .35:
            raise ValueError(f'{side.title()} arm moved during capture; hold it at rest and retry')
        sample = samples[-1]
        other = 'right' if side == 'left' else 'left'
        # Preserve only the state-schema inputs, not old live-evidence claims.
        result = {key: copy.deepcopy(state[key]) for key in (
            'schema', 'coordinate_convention', 'left_rad', 'right_rad',
            'left_gripper_opening_m', 'right_gripper_closed_motor_joint_rad')}
        result[side+'_rad'] = [sample['joints'][n] for n in names]
        if side == 'left':
            result['left_gripper_opening_m'] = sample['joints']['openarmx_left_finger_joint1']
        result.update(preview_only=True, measured_side=side, source='One-shot selected-arm read-only encoder snapshot',
                      captured_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                      side_sources={side: 'live encoder snapshot', other: str(base_state)},
                      static_state_sha256=digest(base_state), model_sha256=digest(ROOT/'model/openarmx.urdf'),
                      encoder_source_sha256=digest(ROOT/'safe_zone/encoder.py'), calibration_sha256=digest(calibration),
                      motor_telemetry={side: sample['feedback']}, motor8_feedback=sample['motor8_feedback'],
                      synchronization=dict(batch_count=len(samples), joint_spread_deg=spread.tolist(),
                                           camera_synchronized=False, scene='separately saved frozen scene'),
                      motor_commands=0, gripper_commands=0, motion_authorized=False, hardware_execution_available=False)
        atomic_json(folder/'measured_start.json', result)
        read_state(folder/'measured_start.json')
        return folder/'measured_start.json'
    except BaseException as exc:
        error = str(exc) or type(exc).__name__
        raise
    finally:
        atomic_json(folder/'capture_report.json', dict(side=side, bus=bus, other_arm_queried=False,
                    query_packets=gate.sent if gate else 0, samples=len(samples), error=error,
                    motor_commands=0, enable_commands=0, disable_commands=0, relax_commands=0,
                    socket_closed=observer is None or observer.socket is None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['left', 'right'], required=True)
    parser.add_argument('--state', required=True, help='Saved state supplying the unchanged opposite arm')
    parser.add_argument('--output', required=True)
    parser.add_argument('--calibration', default=str(DEFAULT_CALIBRATION))
    args = parser.parse_args()
    print(capture(args.output, args.side, args.state, args.calibration))


if __name__ == '__main__':
    main()
