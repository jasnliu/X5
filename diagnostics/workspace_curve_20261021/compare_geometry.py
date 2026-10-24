import json
from pathlib import Path
import numpy as np
BASE=Path('planner_runs/workspace-curve-study-20261021')
for case in sorted(BASE.iterdir()):
 if not case.is_dir() or not (case/'reference_results.json').exists():continue
 refs=json.loads((case/'reference_results.json').read_text());p=json.loads(Path(refs['single_arc']['plan']).read_text());r=p['experiment']['report']['candidates'][0]
 print(case.name)
 for label,row in [('single_arc',r)]+[(f.parent.name, min(json.loads(f.read_text())['candidates'], key=lambda x:x['score'])) for f in case.glob('workspace-*/result.json') if json.loads(f.read_text())['candidates']]:
  g=row['gesture'];m=row['metrics'];print(label,'wrist/tip turn',round(g['wrist']['broad_turning_deg'],1),round(g['tip']['broad_turning_deg'],1),'variation',round(m['spatial']['wrist']['curvature_variation'],2),'length',round(g['wrist']['length_m'],3),'reverse',g['reversal_count'],'proximity',round(m['baseline']['components']['obstacle_proximity'],3))
