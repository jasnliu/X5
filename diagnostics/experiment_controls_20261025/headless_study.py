"""Actual fresh CLI integration on frozen inputs, serial and no hardware."""
import json,os,subprocess,time,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from drum_planner.common import atomic_json,endpoint,read_state,DEFAULT_CALIBRATION
from drum_planner.experiment.motion_settings import settings,FIELDS
H=Path('diagnostics/experiment_controls_20261025').resolve();fixtures=H/'inputs';fixtures.mkdir(exist_ok=True)
tag=os.environ.get('X5_STUDY_RUN','')
state=Path('diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json').resolve()
scene=state.parent/'camera/scene.json';original=read_state(state)
for side,names in [('right',['R1','R2']),('left',['L1'])]:
 for name in names:
  if not (fixtures/f'{name}.json').exists():(fixtures/f'{name}.json').write_bytes(Path(f'destinations/{side}/destination{name}.json').read_bytes())
near=json.loads((fixtures/'L1.json').read_text());near['samples'][-1]['positions_rad'][5]+=float(np.radians(1.))
atomic_json(fixtures/'L_near.json',near)
p=settings('right');p['name']='Hypothetical distal speed - SIMULATION ONLY'
p['velocity_rad_s'][4:]=[1.2,2.4,2.4];p['acceleration_rad_s2'][4:]=[4.8,19.2,19.2];p['jerk_rad_s3'][4:]=[48.,384.,384.]
atomic_json(fixtures/'right_distal.json',p)
p=settings('left');p['name']='Unequal test constraints - SIMULATION ONLY'
for d,key in enumerate(FIELDS):p[key]=[v*k**(d+1) for v,k in zip(p[key],[.8,1.1,.9,.7,1.4,1.5,1.2])]
atomic_json(fixtures/'left_unequal.json',p)
R1=str(fixtures/'R1.json');R2=str(fixtures/'R2.json');L1=str(fixtures/'L1.json');LN=str(fixtures/'L_near.json')
cases=[('right_default','right','default',[R1],'balanced',None),
 ('right_fast','right',R1,[R2],'fast',None),('right_balanced','right',R1,[R2],'balanced',None),
 ('right_thorough','right',R1,[R2],'thorough',None),('right_reverse','right',R2,[R1],'balanced',None),
 ('right_stationary','right',R1,[R1],'balanced',None),
 ('right_sequence_custom','right',R1,[R1,R2,R1],'balanced',fixtures/'right_distal.json'),
 ('left_stationary','left',L1,[L1],'balanced',None),
 ('left_near','left',L1,[LN],'balanced',None),
 ('left_near_custom','left',L1,[LN],'thorough',fixtures/'left_unequal.json'),
 ('left_default_blocked','left','default',[L1],'balanced',None)]
rows=[]
for name,side,start,targets,quality,profile in cases:
 out=H/('runs_'+tag if tag else 'runs')/name;out.parent.mkdir(exist_ok=True)
 cmd=['./plan_drums.sh','experiment','--'+side,'--offline','--state',str(state),'--scene',str(scene),
      '--calibration',str(DEFAULT_CALIBRATION),'--start',start,'--destinations',*targets,'--timing-quality',quality,'--headless','--output',str(out)]
 if profile:cmd+=['--motion-profile',str(profile)]
 print('RUN',name,flush=True);t=time.monotonic()
 with (H/((tag+'_' if tag else '')+name+'.log')).open('w') as f:proc=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=240)
 row=dict(case=name,side=side,returncode=proc.returncode,cold_process_s=time.monotonic()-t,command=cmd)
 try:
  result=json.loads((out/'results.json').read_text())['workspace_curve'];plan=Path(result['plan']);obj=json.loads(plan.read_text())
  req=json.loads((plan.parent/'backend_request.json').read_text());context=json.loads((out/'context.json').read_text())
  row.update(plan=str(plan),status=obj['status'])
  assert proc.returncode==0 and obj['motion_authorized'] is False and obj['hardware_execution_available'] is False
  sidekey=side+'_rad';spawn=original[sidekey] if start=='default' else endpoint(start,side=side).tolist()
  assert req['start']==spawn and req['scene_robot_start']==original[sidekey]
  opposite='left' if side=='right' else 'right';assert req[opposite]==original[opposite+'_rad']
  if name=='left_default_blocked':
   assert obj['status']=='no_feasible_route'
   row['passed']=True;row['expected_blocked']=True
  else:
   assert obj['status']=='planned',obj.get('details')
   assert obj['cycle'][0]['positions_rad']==obj['cycle'][-1]['positions_rad']==spawn
   sequence=obj.get('checkpoint_sequence')
   legs=([Path(s['plan']) for s in sequence['segments']]+[Path(sequence['return_leg']['plan'])]) if sequence else [plan]
   for leg in legs:
    local=json.loads(leg.read_text());rq=json.loads((leg.parent/'backend_request.json').read_text())
    assert rq['scene_robot_start']==original[sidekey] and rq[opposite]==original[opposite+'_rad']
    limits=np.array([context['motion_settings'][key] for key in FIELDS]);assert local['timing']['limits']==limits.tolist()
    assert np.all(np.array(local['timing']['peaks'])<=limits)
   row.update(passed=True,movement_s=(sequence['movement_outward_s'] if sequence else obj['duration_outward_s']),
              return_s=sequence['return_leg']['timing']['duration_s'] if sequence else obj['duration_outward_s'],
              retiming=obj['experiment']['retiming'])
 except Exception as exc:row.update(passed=False,error=str(exc))
 rows.append(row);atomic_json(H/('headless_'+tag+'_results.json' if tag else 'headless_results.json'),rows);print('RESULT',name,row['passed'],row.get('movement_s'),flush=True)
print('RESULTS',sum(r['passed'] for r in rows),'/',len(rows),flush=True)
