"""Interrupt only our offline CLI; verify disposable worker cleanup."""
from pathlib import Path
import json,signal,subprocess,time,uuid
ROOT=Path(__file__).resolve().parents[2];H=Path(__file__).resolve().parent
before=set((ROOT/'diagnostics').glob('joint-limit-job-*'))
log=H/'cancel_verified_cli.log';output=H/('cancelled_demo-'+uuid.uuid4().hex[:8])
with log.open('w') as f:
    p=subprocess.Popen(['./test_joint_limits.sh','demo','--right','--output',str(output)],
        cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+15
        while 'OFFLINE SOFTWARE TEST' not in log.read_text():
            assert p.poll() is None,'CLI finished before cancellation test'
            assert time.monotonic()<deadline,'worker failed to start'
            time.sleep(.03)
        p.send_signal(signal.SIGINT);p.wait(timeout=15)
        assert p.returncode==130 and not (output/'completion.json').exists()
    finally:
        if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=15)
jobs=set((ROOT/'diagnostics').glob('joint-limit-job-*'))-before
assert len(jobs)==1
job=json.loads((jobs.pop()/'job.json').read_text())
# systemd-run may return zero for an explicitly stopped service. Cancellation,
# terminal service state and absence of completion evidence are authoritative.
assert job['cancelled']
state=subprocess.check_output(['systemctl','--user','show',job['unit']+'.service','-p','ActiveState'],text=True).strip()
assert state not in ('ActiveState=active','ActiveState=activating','ActiveState=deactivating'),state
result=dict(passed=True,parent_exit_code=p.returncode,worker=job,final_service_state=state,hardware_access=False)
(H/'cancel_result.json').write_text(json.dumps(result,indent=2)+'\n')
print('Offline cancellation and worker cleanup passed')
