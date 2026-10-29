"""Actual new CLI runs, serial, saved scenes only. No hardware access."""
import json,subprocess,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from joint_limit_lab.schema import read,write,defaults
from drum_planner.experiment.motion_settings import settings
H=Path(__file__).resolve().parent
if len(sys.argv)>1:
    tag=sys.argv[1]
    assert tag and all(c.isalnum() or c in '_-' for c in tag)
    H=H/tag;H.mkdir(exist_ok=False)
prior=read(ROOT/'diagnostics/experiment_controls_20261025/headless_final_results.json')
plans={r['case']:r['plan'] for r in prior};rows=[]
def run(name,args):
    command=['./test_joint_limits.sh',*args];started=time.monotonic()
    with (H/(name+'.log')).open('w') as f:subprocess.run(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=600)
    rows.append(dict(case=name,command=command,elapsed_s=time.monotonic()-started,passed=True))
    write(H/'cli_results.json',rows);print(name,'PASS',flush=True)
for side,case in [('right','right_balanced'),('left','left_near')]:
    c=defaults(side);c.update(test_levels=[.5,1.],repeats=2)
    write(H/(side+'_config.json'),c)
    folder=H/('prepared_'+side)
    run('prepare_'+side,['prepare','--config',str(H/(side+'_config.json')),'--plan',plans[case],'--output',str(folder)])
    c=read(folder/'campaign.json');assert c['trials'] and c['hardware_execution_available'] is False
    run('simulate_'+side,['simulate','--campaign',str(folder/'campaign.json'),'--output',str(H/('traces_'+side))])
    run('analyze_'+side,['analyze','--campaign',str(folder/'campaign.json'),'--traces',str(H/('traces_'+side)),
        '--output',str(H/('analysis_'+side)),'--allow-synthetic-profile'])
    r=read(H/('analysis_'+side)/'report.json');assert r['synthetic'] and not r['hardware_limits_known']
    profile=H/(side+'_original_profile.json');write(profile,settings(side))
    run('routes_'+side,['routes','--plan',plans[case],'--profile',str(profile),'--output',str(H/('route_'+side))])
    result=read(H/('route_'+side)/'report.json')
    assert result['geometric_path_unchanged'] and not result['motion_authorized']
run('demo_right',['demo','--right','--output',str(H/'demo_right')])
run('demo_left',['demo','--left','--output',str(H/'demo_left')])
run('demo_failure',['demo','--right','--failure','tracking','--output',str(H/'demo_failure')])
r=read(H/'demo_failure/analysis/report.json');assert r['profile'] and not r['hardware_limits_known']
assert all(g['highest_complete_level']==.75 for g in r['groups'])
print('ALL CLI CASES PASSED',len(rows),flush=True)
