"""Verify archived fresh-launch evidence without opening camera/CAN."""
import json
from pathlib import Path
from drum_planner.common import atomic_json,arm_joints,digest
from drum_planner.read_only import ALLOWED
base=Path(__file__).resolve().parent;results=[]
for name in ('live-right-1','live-left-1','live-right-2'):
    p=base/name
    a=json.loads((p/'acquisition/acquisition_report.json').read_text())
    c=json.loads((p/'context.json').read_text())
    s=json.loads(Path(a['state']).read_text());scene=json.loads(Path(a['scene']).read_text())
    req=json.loads((p/'initial/backend_request.json').read_text())
    view=json.loads((p/'experiment_view_report.json').read_text())
    assert a['passed'] and a['camera_released']
    assert a['telemetry']['thread_stopped'] and a['telemetry']['observer']['closed']
    assert a['state_sha256']==digest(a['state']) and a['scene_sha256']==digest(a['scene'])
    assert scene['measured_state_sha256']==a['state_sha256']
    assert s['synchronization']['planning_frame_count']==5
    for proof in [s['synchronization']]+s['synchronization']['all_five_frames']:
        assert proof['stationary_joints']==arm_joints(a['moving_side'])
        assert proof['nearest_midpoint_skew_s']<=.1 and len(proof['bracketing_indices'])==2
    assert c['input_origin']=='fresh_startup' and c['state']==a['state'] and c['scene']==a['scene']
    for side in ('left','right'):
        key='start' if side==a['moving_side'] else side
        assert req[key]==s[side+'_rad']
    assert view['generations']==0 and view['selected_destination'] is None
    queries=[json.loads(line) for line in (p/'acquisition/can_query_audit.jsonl').read_text().splitlines()]
    assert queries and all(bytes.fromhex(q['packet_hex']) in ALLOWED for q in queries)
    assert set(q['bus'] for q in queries)=={'can0','can1'}
    assert all(q['operation']=='state_request_only' for q in queries)
    for job in ('setup','acquisition_job','initialize_job'):
        j=json.loads((p/job/'job.json').read_text());assert j['exit_code']==0
        families=j['containment'].get('RestrictAddressFamilies','')
        assert ('AF_CAN' in families)==(job=='acquisition_job'),families
    results.append(dict(session=name,passed=True,side=a['moving_side'],started_utc=a['started_utc'],
        acquisition_s=a['elapsed_s'],state_sha256=a['state_sha256'],scene_sha256=a['scene_sha256'],
        raw_depth_sha256=scene['files_sha256']['raw_depth.npz'],
        first_frame_monotonic=scene['frames'][0]['received_monotonic'],
        last_frame_monotonic=scene['frames'][-1]['received_monotonic'],
        query_only_packets=sum(q['event']=='sent' for q in queries),motor_commands=0))
assert len({r['state_sha256'] for r in results})==len(results)
assert len({r['scene_sha256'] for r in results})==len(results)
assert len({r['raw_depth_sha256'] for r in results})==len(results)
ordered=sorted(results,key=lambda r:r['first_frame_monotonic'])
assert all(a['last_frame_monotonic']<b['first_frame_monotonic'] for a,b in zip(ordered,ordered[1:]))
atomic_json(base/'acquisition_validation.json',dict(passed=True,sessions=results))
print(json.dumps(results,indent=2))
