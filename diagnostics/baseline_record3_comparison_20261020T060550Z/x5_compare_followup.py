import sys,json
from pathlib import Path
import numpy as np
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner.experiment.config import no_can
no_can()
from drum_planner.experiment.runtime import Oracle
from drum_planner.common import atomic_json
from drum_planner.experiment.fluidity import curve_shape
D=Path(Path('/tmp/x5_comparison_task').read_text().strip());p=json.loads((D/'report.json').read_text());f=json.loads((D/'features.json').read_text());qs=np.load(D/'paths.npz')
o=Oracle(D/'backend_request.json',D/'followup');more={}
try:
 for name,q in qs.items():
  check=o.check(q)
  bad=np.flatnonzero(np.logical_not(check['valid']));runs=[]
  for part in np.split(bad,np.where(np.diff(bad)>1)[0]+1):
   if len(part):runs.append(dict(first=int(part[0]),last=int(part[-1]),start_joint_arc_fraction=float(part[0]/800),end_joint_arc_fraction=float(part[-1]/800)))
  details={}
  for index in sorted(set([0,800,*([int(bad[0]),int(bad[-1])] if len(bad) else [])])):
   details[str(index)]=o.check(q[index:index+1])
  xyz=np.array(f[name]['tip_xyz_m']);w=np.array(f[name]['wrist_xyz_m'])
  j5=q[:,4]
  more[name]=dict(collision_ranges=runs,key_pose_checks=details,
   positive_x_overshoot_tip_m=float(max(0,xyz[:,0].max()-xyz[-1,0])),positive_x_overshoot_wrist_m=float(max(0,w[:,0].max()-w[-1,0])),
   joint5_min_deg=float(np.degrees(j5.min())),joint5_max_deg=float(np.degrees(j5.max())),
   tip_horizontal_travel_m=float(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).sum()),
   constant_speed_joint_only_duration_lower_bound_s=float(np.max(abs(np.diff(q,axis=0)),axis=1).sum()/.6),
   joint_bending_scale_sweep={str(s):curve_shape(q,s)['total_turn_rad'] for s in (.025,.05,.1,.2,.4)})
 atomic_json(D/'supplement.json',more)
 print(json.dumps(more,indent=2),flush=True)
finally:o.close()
