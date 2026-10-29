"""Reproduce the two additional saved-input CLI integrations. No live devices."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
os.chdir(ROOT)
H=Path(__file__).resolve().parent
tag=sys.argv[1] if len(sys.argv)>1 else 'replay'
assert tag and all(c.isalnum() or c in '_-' for c in tag)
state=ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json'
scene=state.parent/'camera/scene.json'
cal=ROOT/'calibration_sessions/20261024T211629Z-1227b7b1-reprocessed/calibration.json'
f=H/'inputs';results=[]
for name,side,extra in [
    ('left_sequence','left',['--start',str(f/'L1.json'),'--destinations',str(f/'L1.json'),
        str(f/'L_near.json'),str(f/'L1.json'),'--motion-profile',str(f/'left_unequal.json'),
        '--timing-quality','balanced','--headless']),
    ('cli_ui','right',['--start',str(f/'R1.json'),'--destinations',str(f/'R2.json'),
        '--timing-quality','balanced','--seconds','15'])]:
    folder=H/(tag+'_'+name)
    cmd=['./plan_drums.sh','experiment','--'+side,'--offline','--state',str(state),
         '--scene',str(scene),'--calibration',str(cal),'--output',str(folder),*extra]
    began=time.monotonic()
    with (H/(tag+'_'+name+'.log')).open('w') as log:
        subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
    elapsed=time.monotonic()-began
    if name=='left_sequence':
        result=json.loads((folder/'results.json').read_text())['workspace_curve']
        p=json.loads(Path(result['plan']).read_text());sequence=p['checkpoint_sequence']
        assert p['status']=='planned'
        assert sequence['segments'][0]['timing']['duration_s']==0
        assert sequence['return_leg']['timing']['duration_s']==0
        assert sequence['hold_at_each_checkpoint_s']==2
        detail=dict(movement_s=sequence['movement_outward_s'])
    else:
        p=json.loads((folder/'experiment_view_report.json').read_text())
        assert p['generations']==1 and p['method_states']=={'workspace_curve':'ready'}
        assert p['selected_checkpoints']==[str(f/'R2.json')]
        assert p['simulated_start']['destination_source']==str(f/'R1.json')
        detail=dict(generations=p['generations'],mesh_builds=p['mesh_builds'])
    assert p['can_access'] is False and p['motor_commands']==0
    results.append(dict(case=name,passed=True,process_s=elapsed,command=cmd,**detail))
    print(name,'PASS',detail,flush=True)
(H/('supplemental_'+tag+'.json')).write_text(json.dumps(results,indent=2)+'\n')
