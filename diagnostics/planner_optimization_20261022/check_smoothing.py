"""Native left-v2 selection/world reuse parity, including pure timing cache hits."""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from verify import numerical
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.timing import smooth_v2,benchmark,core


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    no_can();a.output.mkdir(parents=True,exist_ok=False);rows=[]
    for case in ('single_arc','clearance'):
        base=ROOT/'diagnostics/corner_smoothing_20261021'
        source=json.loads((base/'inputs'/('left_'+case+'.json')).read_text())
        reference=json.loads((base/'selected'/('left_'+case)/'law.json').read_text())
        for phase in ('cold','warm'):
            began=time.monotonic()
            rounded,law,proof,details=smooth_v2.choose(source,a.output/(case+'-'+phase),lambda x:print(x,flush=True))
            assert law is not None and numerical(law)==numerical(reference),case
            independent=benchmark.validate(law,rounded,a.output/(case+'-'+phase+'-independent'))
            assert independent==proof,(case,'reusing the loaded world changed its proof')
            if phase=='warm':assert details['attempts'][-1]['math_cache']['cached']
            rows.append(dict(case=case,phase=phase,elapsed_s=time.monotonic()-began,exact_law=True,
                identical_independent_proof=True,math_cache=details['attempts'][-1]['math_cache']))
    atomic_json(a.output/'verification.json',dict(rows=rows,camera_access=False,motor_commands=0))
    print('NATIVE LEFT SMOOTHING PARITY PASS',flush=True)


if __name__=='__main__':main()
