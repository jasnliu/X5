import sys,json,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment.search import rank,FAMILIES
from drum_planner.common import digest,atomic_json
p=Path(__file__).resolve().parent;changes=[]
for stage in ('refine','heldout'):
 for f in (p/stage).glob('*/result.json'):
  d=json.loads(f.read_text())
  if d.get('selection_precision_digits')==8:continue
  seeds,_=rank(d['rows'],'compact',d['parameters'])
  old_seeds=sorted(seeds,key=lambda r:(r['score'],r['index']))
  assert [r['index'] for r in seeds[:2]]==[r['index'] for r in old_seeds[:2]],'Generator seed would change; regenerate this case'
  atomic_json(f.with_name('result_before_precision_fix.json'),d)
  for m in FAMILIES:
   rows,_=rank(d['rows'],m,d['parameters']);new=rows[0] if rows else None
   if new and d['selected'][m]['index']!=new['index']:
    changes.append(dict(stage=stage,case=d['case']['id'],seed=d['seed'],method=m,old=d['selected'][m]['index'],new=new['index'],score_difference=new['score']-d['selected'][m]['score']))
   d['selected'][m]=new
  d.update(selection_precision_digits=8,selection_source_sha256=digest(Path('drum_planner/experiment/search.py')),
           generation_seeds_unchanged_by_precision_fix=True)
  atomic_json(f,d)
  e=f.with_name('evaluation.json')
  if e.exists():e.rename(e.with_name('evaluation_before_precision_fix.json'))
atomic_json(p/'precision_fix.json',dict(changes=changes,all_33_generator_seed_choices_unchanged=True,
    reason='Round soft scores to 8 decimals (as production does) before candidate-index tie-break; no elapsed-time preference, objective weights or recording-similarity changes'))
print(json.dumps(changes,indent=2))
