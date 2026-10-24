"""Isolated real-driver fallback and in-flight GPU cancellation, no hardware."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.timing import gpu_integration as adapter,core
from drum_planner.timing.vulkan import device,optimizer
import numpy as np

BASE=Path(__file__).parent

def fallback(out,side):
    no_can();out.mkdir(parents=True)
    source=json.loads((ROOT/f'diagnostics/planner_performance_20261021/inputs/{side}_single_arc.json').read_text())
    winner=dict(path=source['path_waypoints'],timing=source['timing'],duration_s=source['timing']['duration_s'])
    points,law,proof,meta=adapter.retime(winner,{'pool_key':'real-driver-fallback-test'},source['request'],
        {'experiment_gpu_timing':True,'pool_root':str(out/'pools')},out,lambda s:print(s,flush=True))
    assert meta['actual_backend']=='cpu' and not meta['gpu_used']
    # validate() raises on any collision; successful proofs count checked samples.
    assert 'CPU FALLBACK' in meta['label'] and proof['samples_checked']>1
    atomic_json(out/'result.json',dict(meta=meta,proof=proof,duration_s=law['duration_s'],motor_commands=0))

def busy(out):
    no_can();source=json.loads((ROOT/'diagnostics/planner_performance_20261021/inputs/right_single_arc.json').read_text())
    path=core.original(source);base=source['timing']['time_scale']/core.RESERVE
    _,_,_,B,q,w=optimizer.arrays(path,base,192)
    with device.Device(B,q,w,np.ones(192)) as gpu:
        gpu.step();atomic_json(out/'gpu_started.json',dict(device=gpu.name))
        while True:gpu.step()

def wait(job):
    while job.poll() is None:time.sleep(.2)
    assert job.poll()==0,job.log_path

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['parent','fallback','busy'],default='parent');p.add_argument('--side',default='right');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.mode=='fallback':fallback(a.output,a.side)
    elif a.mode=='busy':busy(a.output)
    else:
        no_can();a.output.mkdir(parents=True,exist_ok=False)
        for side in ('right','left'):
            command=['/usr/bin/env','VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json',PYTHON,__file__,
                '--mode','fallback','--side',side,'--output',a.output/side]
            job=Job(command,a.output/(side+'-job'),timeout=600)
            try:wait(job)
            finally:job.close()
        out=a.output/'cancel';out.mkdir()
        job=Job([PYTHON,__file__,'--mode','busy','--output',out],out/'job',timeout=60)
        try:
            end=time.monotonic()+30
            while not (out/'gpu_started.json').exists() and time.monotonic()<end:
                assert job.poll() is None;time.sleep(.1)
            assert (out/'gpu_started.json').exists()
            start=time.monotonic();job.cancel()
            while job.poll() is None and time.monotonic()-start<10:time.sleep(.1)
            assert job.reaped and job.cancelled
            atomic_json(out/'verification.json',dict(cancel_elapsed_s=time.monotonic()-start,
                device_available_after=device.probe(),reaped=job.reaped,motor_commands=0))
        finally:job.close()
        print('PASS real unavailable-driver fallback both arms and in-flight GPU cancellation',flush=True)
