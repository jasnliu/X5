"""Device-free reproduction: production timing versus a healthy virtual CAN bus."""
import json,sys,time,threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from drum_planner.experiment.config import no_can
no_can()
from safe_zone import encoder
from drum_planner.timing import adaptive
from drum_planner.common import ROOT,atomic_json

class Bus:
    def __init__(self):self.queue=[]
    def send(self,frame):
        motor=encoder.FRAME.unpack(frame)[0]&255
        self.queue.append(encoder.FRAME.pack(encoder.EFF|(2<<24)|(motor<<8)|0xfd,8,b'\x80\x00\x80\x00\x80\x00\x00\xe6'))
        return 16
    def recv(self,n):
        if not self.queue:raise BlockingIOError()
        return self.queue.pop(0)
    def close(self):pass

def ready(sockets,*args):
    time.sleep(.0005)  # ordinary nonblocking I/O releases the GIL
    return [s for s in sockets if s.queue],[],[]

def main():
    incident=ROOT/'planner_runs/20261021T163651Z-baseline-c04d74/calculate-6f024196/plan/result'
    winner=json.loads(next((incident/'candidate_pool').glob('*/candidate_011.json')).read_text())
    source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=json.loads((incident/'backend_request.json').read_text()))
    observer=object.__new__(encoder.Observer);observer.sockets={s:Bus() for s in ('left','right')};observer.locks=[];observer.tx_count=0
    stop=threading.Event();samples=[];errors=[]
    def read():
        while not stop.is_set():
            start=time.monotonic()
            try:observer.sample()
            except Exception as exc:errors.append(dict(error=str(exc),queued_frames={s:len(b.queue) for s,b in observer.sockets.items()}));return
            samples.append([start,time.monotonic()]);stop.wait(max(0,.05-(time.monotonic()-start)))
    with patch.object(encoder,'select',SimpleNamespace(select=ready)):
        thread=threading.Thread(target=read);thread.start()
        try:
            law=adaptive.optimize(source,levels=(8,12,20,32,48,64,96,128,192),ftol=1e-10,maxiter=600,log=lambda s:print(s,flush=True))
        finally:stop.set();thread.join()
    report=dict(software_only=True,physical_can_opened=False,errors=errors,sample_batches=len(samples),duration_s=law['duration_s'],maximum_batch_s=max(b-a for a,b in samples))
    atomic_json(Path(__file__).with_name('reproduction.json'),report);print(json.dumps(report,indent=2))
    assert errors and any(v for v in errors[0]['queued_frames'].values()),report
if __name__=='__main__':main()
