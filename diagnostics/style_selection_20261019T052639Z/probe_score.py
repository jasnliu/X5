import json,time,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from drum_planner import planning,style
from drum_planner.execution import Curve
D=Path(__file__).parent;old=Path('planner_runs/20261019T050214Z-current-plan-dd599f/plan');p=json.loads((old/'plan.json').read_text());r=json.loads((old/'backend_request.json').read_text());c=Curve(p);q=style.sample_path([c.at(t) for t in np.linspace(0,c.duration,4001)]);r['style_paths']=[q.tolist()];planning.build_moveit();t=time.monotonic();result=planning.backend(r,D/'probe');print('seconds',time.monotonic()-t);print(result['status']);print(json.dumps(style.metrics(q,result['style_features'][0]),indent=2));(D/'probe_metrics.json').write_text(json.dumps(style.metrics(q,result['style_features'][0]),indent=2))
