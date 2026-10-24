"""Software-only corner smoothing comparisons on frozen generated paths."""
import argparse,json,time,traceback
from pathlib import Path
from ..common import ROOT,atomic_json,digest
from ..experiment.config import no_can
from ..experiment.runtime import Oracle
from . import core,adaptive,benchmark,rounding,motion_quality

BASE=ROOT/'diagnostics/corner_smoothing_20261021'


def variants(phase='initial'):
    if phase=='refined':
        yield dict(kind='local_c2',fraction=.45,simplify=1e-12)
        for fraction in (.45,.49):yield dict(kind='local_c3',fraction=fraction,simplify=1e-12)
        for width in (.01,.02,.04,.06):yield dict(kind='overlap_c3',fraction=width,simplify=1e-12)
        return
    for eps in (1e-12,.0005,.002):yield dict(kind='quintic',fraction=.45,simplify=eps)
    for kind in ('local_c2','local_c3'):
        for fraction in (.25,.45,.49):yield dict(kind=kind,fraction=fraction,simplify=.0005)
    for eps in (.002,.004):yield dict(kind='local_c3',fraction=.49,simplify=eps)


def run(folder,cases,levels,phase='initial'):
    no_can();out=Path(folder);out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((BASE/'inputs.json').read_text());rows={}
    for case in cases:
        spec=manifest[case];file=ROOT/spec['file'];assert digest(file)==spec['sha256']
        source=json.loads(file.read_text());source['request']=benchmark.benchmark_request(source)
        work=out/case;work.mkdir(exist_ok=True);atomic_json(work/'request.json',source['request'])
        oracle=Oracle(work/'request.json',work);results=[]
        rows[case]=dict(v1=motion_quality.law_metrics(source['v1_law']),variants=results)
        try:
            for i,params in enumerate(variants(phase)):
                started=time.monotonic();row=dict(index=i,params=params);results.append(row)
                print(f'\nROUNDING {case} {i}: {params}',flush=True)
                try:
                    pp,q=rounding.propose(source,**params)
                    ok,proof=rounding.inspect(pp,source,oracle);row['geometry']=proof
                    if not ok:row['status']='rejected_geometry';print('REJECT',proof.get('reason'),flush=True);continue
                    rounded=rounding.as_source(pp,source)
                    law=adaptive.optimize(rounded,levels=levels,log=lambda s:print(s,flush=True))
                    quality=motion_quality.law_metrics(law)
                    target=work/f'variant_{i:02d}';target.mkdir(exist_ok=True)
                    verification=benchmark.validate(law,rounded,target/'check')
                    atomic_json(target/'source.json',rounded);atomic_json(target/'law.json',law)
                    row.update(status='valid',quality=quality,proof=verification,source=str(target/'source.json'),law=str(target/'law.json'))
                    print('VALID',json.dumps(quality),flush=True)
                except Exception as e:
                    traceback.print_exc();row.update(status='rejected_numerical',error=str(e))
                finally:
                    row['elapsed_s']=time.monotonic()-started;atomic_json(out/'partial.json',rows)
        finally:oracle.close()
    reference=json.loads((ROOT/'left_recordings/record1.json').read_text())
    result=dict(cases=rows,reference_recording={str(w):motion_quality.recording_metrics(reference,w) for w in (.1,.2,.3)},
        reference_used_after_generation_only=True,motor_commands=0,live_camera=False,levels=list(levels),phase=phase)
    atomic_json(out/'report.json',result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    p.add_argument('--cases',default='left_single_arc,left_clearance');p.add_argument('--levels',default='8,16,32')
    p.add_argument('--phase',choices=['initial','refined'],default='initial')
    a=p.parse_args();run(a.output,a.cases.split(','),tuple(map(int,a.levels.split(','))),a.phase)
