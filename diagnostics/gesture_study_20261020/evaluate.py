import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from drum_planner import style
from drum_planner.experiment import gesture
from drum_planner.experiment.fluidity import resample
from drum_planner.experiment.runtime import Oracle
from drum_planner.experiment.config import no_can
from drum_planner.common import atomic_json,ROOT
no_can()
base=ROOT/'diagnostics/gesture_study_20261020';old=ROOT/'diagnostics/baseline_record3_comparison_20261020T060550Z'
ref=np.load(old/'paths.npz')['record3_raw'];ref_features=json.loads((old/'features.json').read_text())['record3_raw']
def residual(a):
 a=resample(a,201);s=np.linspace(0,1,len(a))[:,None]
 return a-((1-s)*a[0]+s*a[-1])
def metrics(q,fs):
 out={}
 for name in ('wrist','tip'):
  a=np.array(fs[name+'_xyz_m']);b=np.array(ref_features[name+'_xyz_m'])
  out[name+'_raw_progress_rms_cm']=100*float(np.sqrt(np.mean(np.sum((a-b)**2,axis=1))))
  out[name+'_chord_residual_rms_cm']=100*float(np.sqrt(np.mean(np.sum((residual(a)-residual(b))**2,axis=1))))
 out['joint_chord_residual_rms_deg']=float(np.degrees(np.sqrt(np.mean((residual(q)-residual(ref))**2))))
 return out
for file in sorted(Path(sys.argv[1]).glob('*/result.json')):
 folder=file.parent;result=json.loads(file.read_text());ev=folder/'evaluation.json'
 if ev.exists():continue
 o=Oracle(folder/'acquisition_request.json',folder)
 try:
  rows={}
  for method,row in result['selected'].items():
   if row is None:rows[method]=None;continue
   cand=json.loads((folder/row['file']).read_text());q=style.sample_path(np.array([p['positions_rad'] for p in cand['outward']]))
   fs=gesture.fk(o,q)
   rows[method]=dict(index=row['index'],source=row['source'],gesture=row['gesture'],duration_s=row['duration_s'],
      clearance_m=row['metrics']['baseline']['minimum_distal_padding_estimate_m'],
      proximity=row['metrics']['baseline']['components']['obstacle_proximity'],similarity=metrics(q,fs))
  atomic_json(ev,dict(case=result['case'],seed=result['seed'],methods=rows,recording_used_only_after_planning=True))
  print(folder.name,json.dumps({m:None if r is None else [r['source'],round(r['gesture']['joint_excess_deg'],1),round(r['gesture']['wrist']['detour_ratio'],3),round(r['gesture']['tip']['positive_x_goal_overshoot_m']*100,1),round(r['similarity']['tip_chord_residual_rms_cm'],2),round(r['duration_s'],1)] for m,r in rows.items()}),flush=True)
 finally:o.close()
