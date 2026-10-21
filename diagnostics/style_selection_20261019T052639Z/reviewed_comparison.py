import json,sys,pathlib
import numpy as np
ROOT=pathlib.Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import planning,style
from drum_planner.execution import Curve
D=pathlib.Path(__file__).parent;folder=D/'saved_scene_plan'
p=json.load(open(folder/'plan.json'));req=json.load(open(folder/'backend_request.json'))
curve=Curve(p)
planned=style.sample_path([curve.at(t) for t in np.linspace(0,curve.duration,4001)])
recorded=style.sample_path([s['positions_rad'] for s in json.load(open(ROOT/'recordings/record3.json'))['samples']])
report=planning.backend(dict(req,style_paths=[planned.tolist(),recorded.tolist()]),D/'reviewed_comparison','style')
assert report['status']=='scored'
a,b=[style.metrics(q,f) for q,f in zip([planned,recorded],report['style_features'])]
scales=style.normalizers(a)
result=dict(planned=style.score(a,scales),recorded=style.score(b,scales),
 planned_metrics=a,recorded_metrics=b,
 note='Style comparison only; recorded start differs and record3 has not become a valid replacement path. No hardware.')
assert result['recorded']['total']<result['planned']['total']
(D/'reviewed_comparison.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
