"""Compact portable laws/inputs; leave bulk local run folders untouched."""
import json,hashlib,shutil
from pathlib import Path
base=Path(__file__).resolve().parent;root=base.parents[1];out=base/'evidence';out.mkdir(exist_ok=True)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
index=[]
mission=base/('verified3' if (base/'verified3').exists() else 'final3')
paths=[(n,mission/n/'plan.json') for n in ('checkpoint_1','checkpoint_2','return_to_start')]
paths += [('left',base/'left2/plan.json')]
paths += [(n,base/n/'plan.json') for n in ('right-minus','right-plus','left-minus','left-plus')]
for name,path in paths:
 obj=json.loads(path.read_text());req=json.loads((path.parent/'backend_request.json').read_text());side=obj['moving_side']
 for key in ('urdf','srdf'):
  original=Path(req[key]);model=out/'models'/(sha(original)+'.'+key);model.parent.mkdir(exist_ok=True)
  shutil.copy2(original,model);req[key]=str(model)
 source=dict(path_waypoints=obj['path_waypoints'],timing=obj['source_timing'],request=req,model=obj['model'])
 payload=dict(source=source,law=obj['timing'],proof=obj['final_curve_check'],profile=obj['experiment']['profile'],
  origin=str(path.relative_to(root)),origin_sha256=sha(path),backend=obj['experiment']['retiming']['actual_backend'])
 target=out/(name+'.json');target.write_text(json.dumps(payload,indent=2)+'\n')
 index.append(dict(name=name,file=str(target.relative_to(root)),sha256=sha(target)))
(out/'index.json').write_text(json.dumps(index,indent=2)+'\n')
