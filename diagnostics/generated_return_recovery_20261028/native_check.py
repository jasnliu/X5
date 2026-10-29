"""Saved-camera return validation; no hardware interfaces permitted."""
import json,sys,time,socket
from pathlib import Path
from drum_planner.common import atomic_json
from drum_planner.experiment.runtime import Job,PYTHON
from camera_playback.generated_planning import load
import numpy as np

def audit(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:raise AssertionError('No physical CAN')
sys.addaudithook(audit)
root=Path(__file__).resolve().parent
mission=Path('planner_runs/generated-beat-20261028T165028-904eb7ef/plan-443f6066/mission.json').resolve()
obj,curves=load(mission)
held={s:c.at(c.duration).tolist() for s,c in curves.items()}
# Copy only metadata; never alter the captured data or the real mission/cache.
copy=dict(obj);copy.pop('return_plan',None)
old=copy['return_file'];copy['return_file']=str(root/'intentionally-missing-return.json')
copy['files']=dict(copy['files']);copy['files'][copy['return_file']]=copy['files'].pop(old)
atomic_json(root/'missing-cache-mission.json',copy)
report=dict(hardware_access=False,motor_commands=0,mission=str(mission),cases={})
for name,progress in [('missing-cache-prefix',{s:c.duration for s,c in curves.items()}),
                      ('unknown-progress-replan',dict(left=0.,right=0.))]:
    folder=root/name
    if folder.exists():raise RuntimeError('Preserve prior diagnostics; choose new output')
    folder.mkdir()
    atomic_json(folder/'input.json',dict(mission=str(root/'missing-cache-mission.json'),held=held,progress=progress))
    job=Job([str(PYTHON),'-m','camera_playback.generated_planning','return','--folder',str(folder),'--input',str(folder/'input.json')],folder/'job',timeout=300)
    try:
        while (code:=job.poll()) is None:time.sleep(.05)
        assert code==0,(name,code)
        result=json.loads((folder/'return.json').read_text())
        assert result['status']=='ready' and [s['side'] for s in result['steps']]==['right','left']
        assert all(p['all_valid'] for p in result['proofs'].values())
        from camera_playback.generated_stream import reference
        for step in result['steps']:
            side=step['side'];a,_=reference(curves[side],step,0);b,_=reference(curves[side],step,step['duration'])
            np.testing.assert_allclose(a,held[side],atol=1e-10,rtol=0)
            np.testing.assert_allclose(b,obj['original'][side+'_rad'],atol=1e-10,rtol=0)
        report['cases'][name]=dict(source=result['source'],proofs=result['proofs'],duration_s=sum(s['duration'] for s in result['steps']),elapsed_s=time.monotonic()-job.started)
        atomic_json(root/'native_report.json',report)
    finally:job.close()
report['passed']=True;atomic_json(root/'native_report.json',report)
print(json.dumps(report,indent=2))
