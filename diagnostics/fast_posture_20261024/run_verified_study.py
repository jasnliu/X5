"""Fresh-process actual --fast CLI study; saved scene only, serial workers."""
import json,os,subprocess,time
from pathlib import Path
ROOT=Path.cwd();HERE=ROOT/'diagnostics/fast_posture_20261024';out=HERE/'verified';out.mkdir(exist_ok=False)
state='diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json'
scene='diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/camera/scene.json'
variants=HERE/'variants';variants.mkdir(exist_ok=True)
for name,side,joint,degrees in [('R2_plus1','right',0,1.),('R2_minus1','right',0,-1.),('L1_j6_plus1','left',5,1.)]:
 import math
 path=ROOT/f'destinations/{side}'/('destinationR2.json' if side=='right' else 'destinationL1.json')
 obj=json.loads(path.read_text());obj['samples']=obj['samples'][-1:];obj['samples'][0]['positions_rad'][joint]+=math.radians(degrees)
 obj['source']='OFFLINE synthetic endpoint perturbation; not a captured drum destination'
 (variants/(name+'.json')).write_text(json.dumps(obj,indent=2)+'\n')
cases=[('right_full','right','full-pose',['R1','R2']),('right_full_reverse','right','full-pose',['R2','R1']),
 ('right_axis','right','tip-axis',['R1','R2']),('right_axis_reverse','right','tip-axis',['R2','R1']),
 ('left_full','left','full-pose',['L1']),('left_axis','left','tip-axis',['L1']),
 ('right_plus1','right','full-pose',['R1',str(variants/'R2_plus1.json')]),
 ('right_minus1','right','full-pose',['R1',str(variants/'R2_minus1.json')]),
 ('left_plus1','left','full-pose',[str(variants/'L1_j6_plus1.json')])]
rows=[]
for name,side,kind,targets in cases:
 target=out/name;log=out/(name+'.log');t=time.monotonic()
 command=['./plan_drums.sh','--fast','--'+side,'--offline','--state',state,'--scene',scene,'--destinations',*targets,
          '--task',kind,'--compare','--headless','--output',str(target)]
 with log.open('w') as f:r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=240)
 row=dict(case=name,side=side,task=kind,command=command,returncode=r.returncode,cold_process_wall_s=time.monotonic()-t)
 files=list((target/'plans').glob('*/fast_report.json'))
 if files:
  report=json.loads(files[0].read_text());row.update(report=str(files[0]),plan=str(files[0].with_name('plan.json')),
   transfer_s=report.get('transfer_movement_s'),reference=report.get('reference'),error=report.get('error'))
 rows.append(row);(out/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
 print(name,'code',r.returncode,'transfer',row.get('transfer_s'),'cold',round(row['cold_process_wall_s'],3),'error',row.get('error'),flush=True)
