"""Read-only acceptance summary from real GUI feedback plus independent CAN log."""
import json,math,re,sys
from pathlib import Path
import numpy as np
from camera_playback.left_hold import LEFT_CENTER
from smooth_playback.trajectory import Geometry
run=Path(sys.argv[1]);log=(run/'app.log').read_text()
evidence=Path(re.search(r'PHYSICAL PLAYBACK EVIDENCE: (.+)',log)[1].strip())
result=json.loads((evidence/'result.json').read_text())
can=json.loads((run/'can_audit.json').read_text())
rows=[json.loads(s) for s in (evidence/'telemetry.jsonl').read_text().splitlines()]
snare=[r['snare'] for r in rows if r['snare'] is not None]
phases=[r['phase'] for r in result['phases']]
assert result['success'] and result['all_16_disabled'] and result['snare_mode_prepared']
assert not result['faults'] and can['motor_fault_frames']==0 and not can['unproven_center_disable_commands']
assert result['left_recording_complete'] and result['right_endpoint_reached']
assert all(not s['moving'] and not s['scheduled'] and not s['error'] and s['completed']==0 for s in snare)
assert 'LEFT RECORDING PLAYBACK' in phases and 'RECORDING PLAYBACK' in phases and phases[-1]=='RELAXED'
assert 'LEFT RETURN: recording start verified; centering, then relaxing' in log
assert 'LEFT RETURN halted' not in log and 'DUAL ARM FAULT' not in log and 'PLAYBACK VERIFY FAULT' not in log
endpoints=[json.loads(s.split('ENDPOINT VERIFIED ',1)[1]) for s in log.splitlines() if 'ENDPOINT VERIFIED {' in s]
assert len(endpoints)==3 # left forward, right forward, left reverse
centers={'left':LEFT_CENTER,'right':Geometry().center}
summary=dict(success=True,evidence=str(evidence),beat_skipped=True,physical=True,
    fault_frames=can['motor_fault_frames'],application_faults=len(result['faults']),
    verified_recording_endpoints=3,all_16_disabled=True,
    max_center_error_deg={s:math.degrees(max(abs(np.array(result['disabled_proofs'][s]['q'])-q))) for s,q in centers.items()},
    measured_joint_motion_span_deg={s:np.degrees(np.ptp(np.array([[r['states'][f'{s}{i}'][0] for i in range(1,8)] for r in rows if all(f'{s}{i}' in r['states'] for i in range(1,8))]),axis=0)).tolist() for s in centers},
    snare_hits=0,snare_tx_pressure_retries=max(s.get('queue_retries',0) for s in snare),
    snare_max_command_gap_s=max(s['max_gap'] for s in snare),
    feedback_frames=can['feedback_frames'],max_temperature_c=can['maximum_temperature_c'])
(run/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
