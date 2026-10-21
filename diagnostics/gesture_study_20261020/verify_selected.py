import sys,json,copy,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from scipy.interpolate import PPoly
from drum_planner import planning
from drum_planner.common import atomic_json,digest
from drum_planner.experiment.config import no_can,METHODS
from drum_planner.workflow import simulate
import xml.etree.ElementTree as ET
no_can();root=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
passed=[];started=time.monotonic()
for file in sorted(root.glob('*/result.json')):
 result=json.loads(file.read_text());req=json.loads((file.parent/'acquisition_request.json').read_text())
 limits={j.attrib['name']:(float(j.find('limit').attrib['lower']),float(j.find('limit').attrib['upper'])) for j in ET.parse(req['urdf']).getroot().findall('joint') if j.find('limit') is not None and 'lower' in j.find('limit').attrib}
 ids={result['selected'][m]['index'] for m in METHODS if result['selected'].get(m)}
 if len([m for m in METHODS if result['selected'].get(m)])!=len(METHODS):raise ValueError('Missing survivor result')
 for index in sorted(ids):
  row=next(r for r in result['rows'] if r['index']==index);c=json.loads((file.parent/row['file']).read_text());tim=c['timing'];scale=tim['time_scale'];duration=tim['duration_s']
  curves=[PPoly(np.asarray(coef),np.asarray(breaks),extrapolate=False) for coef,breaks in zip(tim['coefficients'],tim['polynomial_breaks_unscaled_s'])]
  peaks=np.array([[planning.extrema(pp,k)/scale**k for pp in curves] for k in (1,2,3)])
  assert np.all(peaks<=np.array([.6,1.2,6.])[:,None]+1e-8),peaks
  for j,pp in enumerate(curves):
   lo,hi=limits[f'openarmx_right_joint{j+1}'];roots=pp.derivative().roots(extrapolate=False);roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])];v=pp(np.r_[pp.x,roots])
   assert v.min()>=lo-1e-9 and v.max()<=hi+1e-9
  grid=np.r_[np.arange(0,duration,.002),duration];q=np.column_stack([pp(grid/scale) for pp in curves])
  assert np.allclose(q[0],result['case']['start'],atol=1e-9,rtol=0)
  assert np.allclose(q[-1],result['case']['goal'],atol=1e-9,rtol=0)
  verify=copy.deepcopy(req);verify.update(check_states=q.tolist(),check_path=True,stop_on_invalid=True)
  folder=out/file.parent.name/str(index);folder.mkdir(parents=True,exist_ok=True)
  if (folder/'verified.json').exists():passed.append(json.loads((folder/'verified.json').read_text()));continue
  r=planning.backend(verify,folder,'production_check')
  assert r['status']=='checked' and len(r['checks'])==len(q) and all(c['valid'] for c in r['checks']),r.get('checks',[])[-1:]
  cycle=simulate(dict(status='planned',motion_authorized=False,outward=c['outward'],cycle=planning.cycle(c['outward']),start_state={'right_rad':result['case']['start']}))
  report=dict(case=result['case']['id'],seed=result['seed'],index=index,samples=len(q),passed=True,cycle=cycle,peaks=peaks.tolist(),candidate_sha256=digest(file.parent/row['file']),unchanged_production_backend=True,original_acquisition_self_filter=True)
  atomic_json(folder/'verified.json',report);passed.append(report)
  print('PRODUCTION RECHECK PASSED',file.parent.name,index,len(q),'samples',flush=True)
atomic_json(out/'verification.json',dict(passed=True,unique_trajectories=len(passed),samples=sum(r['samples'] for r in passed),elapsed_s=time.monotonic()-started,results=passed))
