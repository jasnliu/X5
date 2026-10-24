"""Revalidate all retained laws without rerunning optimization or any devices."""
import json,sys
from pathlib import Path
import numpy as np
from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.experiment.config import no_can
from drum_planner.timing import core,benchmark
from drum_planner import planning


def main(output):
    no_can();out=Path(output);base=Path(__file__).resolve().parent
    report=json.loads((base/'report.json').read_text());results={}
    for case,methods in report['rows'].items():
        spec=report['inputs'][case];file=ROOT/spec['file']
        assert digest(file)==spec['sha256']
        source=json.loads(file.read_text());source['request']=benchmark.benchmark_request(source)
        results[case]={}
        for method,row in methods.items():
            law=json.loads((ROOT/row['law']).read_text())
            assert law['duration_s']==row['duration_s']
            proof=benchmark.validate(law,source,out/case/method)
            cycle=planning.cycle(core.samples(law,source))
            assert np.all(np.diff([p['time_s'] for p in cycle])>0)
            results[case][method]=proof
            print('PASS',case,method,flush=True)
    atomic_json(out/'verification.json',dict(passed=True,cases=results,motor_commands=0,live_camera=False))


if __name__=='__main__':main(sys.argv[1])
