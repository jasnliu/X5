"""Retain exact benchmark inputs, deduplicating against already curated bytes."""
import hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];BASE=Path(__file__).parent
manifest=json.loads((ROOT/'config/git_retained_artifacts.json').read_text())
before=json.loads((BASE/'before_hashes.json').read_text())
curated={r['path'] for r in manifest['artifacts']};known={sha:ROOT/p for p,sha in before.items() if p in curated}
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def freeze(value,provenance):
    old=Path(value);sha=digest(old)
    target=known.get(sha,BASE/'replay_inputs/assets'/(sha+old.suffix))
    if not target.exists():target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(old,target)
    assert digest(target)==sha
    provenance[str(old)]=dict(sha256=sha,replay=str(target.resolve()))
    return str(target.resolve())
for case in json.loads((BASE/'benchmark-2/report.json').read_text())['rows']:
 name=case['case'];source=json.loads((BASE/'benchmark-2'/name/'source.json').read_text());provenance={}
 for key in ('urdf','srdf'):source['request'][key]=freeze(source['request'][key],provenance)
 for key in ('points_file','free_depth_file'):
  if source['request']['scene'].get(key):source['request']['scene'][key]=freeze(source['request']['scene'][key],provenance)
 folder=BASE/'replay_inputs';folder.mkdir(exist_ok=True)
 (folder/(name+'.json')).write_text(json.dumps(source,indent=2)+'\n')
 (folder/(name+'-io.json')).write_text(json.dumps(provenance,indent=2)+'\n')
print('Frozen replay inputs; added asset bytes',sum(p.stat().st_size for p in (BASE/'replay_inputs/assets').glob('*')))
