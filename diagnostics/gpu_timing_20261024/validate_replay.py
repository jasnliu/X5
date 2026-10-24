"""Recheck saved GPU laws using retained byte-identical replay inputs."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.timing import benchmark
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();no_can()
a.output.mkdir(parents=True,exist_ok=False);base=Path(__file__).parent;rows=[]
for row in json.loads((base/'benchmark-2/report.json').read_text())['rows']:
 name=row['case'];source=json.loads((base/'replay_inputs'/(name+'.json')).read_text())
 law=json.loads((base/'benchmark-2'/name/'law.json').read_text())
 proof=benchmark.validate(law,source,a.output/name)
 rows.append(dict(case=name,proof=proof));print('PASS replay',name,flush=True)
atomic_json(a.output/'verification.json',dict(passed=True,rows=rows,motor_commands=0,camera_access=False))
