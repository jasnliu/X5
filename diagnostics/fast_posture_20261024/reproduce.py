"""Re-run the nine --fast comparisons using immutable targets and saved scene."""
import argparse,hashlib,json,math,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
provenance=json.loads((HERE/'inputs/provenance.json').read_text())
for name,spec in provenance.items():
 if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=spec['sha256']:raise ValueError('Frozen input changed: '+name)
state=ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json'
scene=state.parent/'camera/scene.json';inputs=HERE/'inputs'
variants={}
for name,side,joint,delta in [('R2_plus1','right',0,1.),('R2_minus1','right',0,-1.),('L1_j6_plus1','left',5,1.)]:
 obj=json.loads((inputs/('destinationR2.json' if side=='right' else 'destinationL1.json')).read_text())
 obj['samples']=obj['samples'][-1:];obj['samples'][0]['positions_rad'][joint]+=math.radians(delta)
 obj['source']='OFFLINE synthetic perturbation, not a captured destination'
 path=out/(name+'.json');path.write_text(json.dumps(obj,indent=2)+'\n');variants[name]=path
r1=inputs/'destinationR1.json';r2=inputs/'destinationR2.json';l1=inputs/'destinationL1.json'
cases=[('right_full','right','full-pose',[r1,r2]),('right_full_reverse','right','full-pose',[r2,r1]),
 ('right_axis','right','tip-axis',[r1,r2]),('right_axis_reverse','right','tip-axis',[r2,r1]),
 ('left_full','left','full-pose',[l1]),('left_axis','left','tip-axis',[l1]),
 ('right_plus1','right','full-pose',[r1,variants['R2_plus1']]),
 ('right_minus1','right','full-pose',[r1,variants['R2_minus1']]),
 ('left_plus1','left','full-pose',[variants['L1_j6_plus1']])]
rows=[]
for name,side,task,targets in cases:
 command=[ROOT/'plan_drums.sh','--fast','--'+side,'--offline','--state',state,'--scene',scene,
          '--destinations',*targets,'--task',task,'--compare','--headless','--output',out/name]
 t=time.monotonic()
 with (out/(name+'.log')).open('w') as log:
  result=subprocess.run(list(map(str,command)),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=240)
 row=dict(case=name,returncode=result.returncode,cold_wall_s=time.monotonic()-t)
 rows.append(row);(out/'results.json').write_text(json.dumps(rows,indent=2)+'\n');print(row,flush=True)
raise SystemExit(0 if all(r['returncode']==0 for r in rows) else 1)
