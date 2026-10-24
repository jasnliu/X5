"""Add only small reproducible evidence; no staging, deletion or Git changes."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[2];base=Path(__file__).resolve().parent
manifest=root/'config/git_retained_artifacts.json';data=json.loads(manifest.read_text());existing={r['path'] for r in data['artifacts']}
files=[]
files += [p for p in base.iterdir() if p.is_file() and p.suffix in ('.py','.sh')]
files += [base/n for n in ('REPORT.md','summary.json','before_hashes.json','preservation.json','pytest-final.log')]
files += [base/'before'/n for n in ('AGENTS.md','DRUM_PLANNER.md','DRUM_PLANNER_EXPERIMENTS.md',
 'drum_planner/cli.py','drum_planner/experiment/worker.py','drum_planner/experiment/app.py')]
files += list((base/'evidence').rglob('*.json'))+list((base/'evidence/models').glob('*.urdf'))+list((base/'evidence/models').glob('*.srdf'))
for n in ('timing20','timing24','timing32'):files.append(base/n/'report.json')
for n in ('verified1','verified2','verified3','verified-cpu','left2','right-minus','right-plus','left-minus','left-plus','fallback'):
 files.append(base/n/'cold.json')
 for name in ('job.json','containment.json','worker.log'):files.append(base/(n+'-job')/name)
for n in ('ui-right','ui-left','ui-cpu'):
 files.append(base/n/'destination_ui_verification.json');files.append(base/n/'destination_dropdown.png')
for n in ('checkpoints-right','checkpoints-left'):
 files.append(base/n/'checkpoint_ui_verification.json');files.append(base/n/'checkpoints.png')
files += [base/'launch2/cold_launch.json',base/'old_cpu/elapsed.json',base/'old_cpu/command.json']
files += [base/'old_cpu/job'/name for name in ('worker.log','job.json','containment.json')]
files += list((base/'cases').glob('*/context.json'))+list((base/'cases').glob('*/endpoint.json'))+list((base/'cases').glob('*/selected_destinations/*/*.json'))
for p in files:
 assert p.is_file(),p
 name=str(p.relative_to(root))
 if name not in existing:
  data['artifacts'].append(dict(path=name,reason='Speed-first GPU experiment: reproducible cold timings, exact law quality, software-only UI and preservation evidence'))
  existing.add(name)
manifest.write_text(json.dumps(data,indent=2)+'\n')
print('Curated new study:',len(files),'files,',sum(p.stat().st_size for p in files),'bytes; bulk runs remain ignored')
