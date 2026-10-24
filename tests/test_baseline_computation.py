"""Planner/encoder process isolation. No physical CAN or camera access."""
import json,os,socket,subprocess,sys,threading,time
from pathlib import Path
from types import SimpleNamespace
import pytest
from drum_planner.baseline import computation
from drum_planner.common import atomic_json
from safe_zone import encoder

@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def guard(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:raise AssertionError('Physical CAN forbidden in software tests')
        return original(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guard)


def test_native_gil_blocking_compute_cannot_starve_query_deadline(monkeypatch,tmp_path):
    # The real Observer.sample deadline/read loop, but immediate healthy bus
    # replies in memory. The child repeatedly holds ITS GIL for 300ms; no CAN
    # deadline or freshness tolerance is changed to make this test pass.
    class Bus:
        def __init__(self):self.queue=[]
        def send(self,frame):
            motor=encoder.FRAME.unpack(frame)[0]&255
            self.queue.append(encoder.FRAME.pack(encoder.EFF|(2<<24)|(motor<<8)|0xfd,8,b'\x80\x00'*3+b'\x00\xe6'))
            return 16
        def recv(self,n):
            if not self.queue:raise BlockingIOError()
            return self.queue.pop(0)
    def ready(sockets,*a):
        time.sleep(.0005)
        return [s for s in sockets if s.queue],[],[]
    monkeypatch.setattr(encoder,'select',SimpleNamespace(select=ready))
    observer=object.__new__(encoder.Observer);observer.sockets={s:Bus() for s in ('left','right')};observer.tx_count=0
    worker=SimpleNamespace(error=None);stop=threading.Event();samples=[]
    def read():
        try:
            while not stop.is_set():
                start=time.monotonic();observer.sample();samples.append((start,time.monotonic()))
                stop.wait(max(0,.05-(time.monotonic()-start)))
        except Exception as exc:worker.error=str(exc)
    thread=threading.Thread(target=read);thread.start()
    try:
        code=computation._run([sys.executable,'-c',
            'import ctypes; f=ctypes.PyDLL(None).usleep; [f(300000) for _ in range(7)]'],worker)
    finally:stop.set();thread.join(3)
    assert code==0 and not worker.error and not thread.is_alive()
    assert len(samples)>=30 and max(b-a for a,b in samples)<.15
    assert max(b[0]-a[0] for a,b in zip(samples,samples[1:]))<.25


def test_already_failed_monitor_never_starts_computation(monkeypatch):
    monkeypatch.setattr(computation.subprocess,'Popen',lambda *a,**k:pytest.fail('must not launch'))
    with pytest.raises(RuntimeError,match='Missing fresh feedback'):
        computation._run(['unused'],SimpleNamespace(error='Missing fresh feedback'))


def test_process_uses_fresh_interpreter_no_inherited_can_fds_and_streamed_log(monkeypatch):
    seen={}
    class Process:
        pid=999999999
        def __init__(self,command,**kw):seen.update(command=command,**kw)
        def poll(self):return 0
        def wait(self,**kw):return 0
    monkeypatch.setattr(computation.subprocess,'Popen',Process)
    assert computation._run(['program'],SimpleNamespace(error=None))==0
    assert seen['start_new_session'] and seen['close_fds']
    assert seen['env']['OPENBLAS_NUM_THREADS']==seen['env']['OMP_NUM_THREADS']=='1'
    assert 'stdout' not in seen and 'stderr' not in seen


@pytest.mark.parametrize('failure',['feedback','interrupt'])
def test_monitor_error_or_cancel_reaps_compute_and_descendants(tmp_path,monkeypatch,failure):
    worker=SimpleNamespace(error=None);pidfile=tmp_path/'pids';processes=[];real=subprocess.Popen
    def spawn(*a,**k):
        p=real(*a,**k);processes.append(p);return p
    monkeypatch.setattr(computation.subprocess,'Popen',spawn)
    def fault():
        deadline=time.monotonic()+3
        while not pidfile.exists() and time.monotonic()<deadline:time.sleep(.01)
        worker.error='Right arm moved since Calculate'
    if failure=='feedback':
        thread=threading.Thread(target=fault);thread.start()
    else:
        class Fault:
            @property
            def error(self):
                if pidfile.exists():raise KeyboardInterrupt()
                return None
        worker=Fault()
    script=('import subprocess,sys,time,os; from pathlib import Path; '
            'p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"]); '
            f'Path({str(pidfile)!r}).write_text(str(os.getpid())+" "+str(p.pid));time.sleep(60)')
    try:
        with pytest.raises(RuntimeError if failure=='feedback' else KeyboardInterrupt):
            computation._run([sys.executable,'-c',script],worker)
    finally:
        if failure=='feedback':thread.join(4)
    assert processes and processes[0].poll() is not None
    child=int(pidfile.read_text().split()[1]);deadline=time.monotonic()+2
    def alive():
        try:return Path(f'/proc/{child}/stat').read_text().split()[2]!='Z'
        except FileNotFoundError:return False
    while alive() and time.monotonic()<deadline:time.sleep(.01)
    assert not alive()


@pytest.mark.parametrize('plan',[None,'old-plan.json'])
def test_device_free_child_keeps_same_planner_and_exact_curve_check(monkeypatch,tmp_path,plan):
    from drum_planner.baseline import tasks
    from drum_planner.experiment import config
    calls=[]
    monkeypatch.setattr(config,'no_can',lambda:calls.append('no_can'))
    def make(folder,capture,cal,*,moving_side):
        assert moving_side=='right'
        assert folder==tmp_path/'plan' and capture==tmp_path/'capture' and cal=='cal.json'
        calls.append('plan');return tmp_path/'new-plan.json'
    def prepare(folder,p,capture,cal,*,allow_replan):
        assert folder==tmp_path/'checked' and capture==tmp_path/'capture' and not allow_replan
        assert p==(plan or tmp_path/'new-plan.json');calls.append('prepare')
    monkeypatch.setattr(tasks,'make_plan',make);monkeypatch.setattr(tasks,'prepare',prepare)
    if plan:plan=str(tmp_path/plan)
    atomic_json(tmp_path/'backend_request.json',{'moving_side':'right'})
    result=computation.compute(tmp_path,'cal.json',plan=plan)
    assert calls==['no_can']+(['plan'] if plan is None else [])+['prepare']
    assert result['plan']==str(plan or tmp_path/'new-plan.json')


@pytest.mark.parametrize('code,detail',[(0,None),(1,'Original collision failure'),(9,None)])
def test_parent_reads_result_or_original_child_error(tmp_path,monkeypatch,code,detail):
    seen=[]
    def run(command,worker):
        seen.extend(command)
        assert worker.error is None
        if code==0:atomic_json(tmp_path/'computation/result.json',dict(plan='test',prepared='checked'))
        elif detail:atomic_json(tmp_path/'computation/error.json',dict(error=detail))
        return code
    monkeypatch.setattr(computation,'_run',run)
    args=(tmp_path,'cal.json',SimpleNamespace(error=None))
    if code:
        with pytest.raises(RuntimeError,match=detail or 'Calculation process exited'):computation.run(*args,plan='old.json')
    else:assert computation.run(*args,plan='old.json')==dict(plan='test',prepared='checked')
    assert seen[:3]==[sys.executable,'-m','drum_planner.baseline.computation']
    assert '--plan' in seen and seen[-1]==str(Path('old.json').resolve())
    assert not (tmp_path/'result.json').exists()  # Only monitored work.py may publish success.


def test_computation_entrypoint_records_errors_without_publishing_success(tmp_path,monkeypatch):
    (tmp_path/'computation').mkdir()
    monkeypatch.setattr(sys,'argv',['computation','--folder',str(tmp_path),'--calibration','cal.json'])
    def fail(*a,**kw):raise ValueError('Invalid scene')
    monkeypatch.setattr(computation,'compute',fail)
    with pytest.raises(ValueError,match='Invalid scene'):computation.main()
    assert json.loads((tmp_path/'computation/error.json').read_text())==dict(error='Invalid scene',motor_commands=0)
    assert not (tmp_path/'computation/result.json').exists()


def test_failed_child_cannot_leave_a_sigterm_ignoring_descendant(tmp_path):
    ready=tmp_path/'descendant'
    child=('import signal,time,os;from pathlib import Path;signal.signal(signal.SIGTERM,signal.SIG_IGN);'
           f'Path({str(ready)!r}).write_text(str(os.getpid()));time.sleep(60)')
    leader=('import subprocess,sys,time;from pathlib import Path;'
            f'subprocess.Popen([sys.executable,"-c",{child!r}]);p=Path({str(ready)!r});'
            '\nwhile not p.exists():time.sleep(.01)\nraise SystemExit(7)')
    assert computation._run([sys.executable,'-c',leader],SimpleNamespace(error=None))==7
    pid=int(ready.read_text());deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        try:
            if Path(f'/proc/{pid}/stat').read_text().split()[2]=='Z':return
        except FileNotFoundError:return
        time.sleep(.01)
    pytest.fail('Calculation left a running descendant')


def test_child_can_guard_blocks_physical_socket_before_planner_import(tmp_path):
    # Inject a stub tasks module; compute must install its real permanent audit
    # hook before calling the planner. Run in a fresh interpreter, not pytest.
    script=('import sys,types,socket;from drum_planner.baseline.computation import compute;'
            'm=types.ModuleType("drum_planner.baseline.tasks");'
            'm.make_plan=lambda *a,**kw:socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW);'
            'sys.modules[m.__name__]=m;'
            f'compute({str(tmp_path)!r},"cal.json")')
    p=subprocess.run([sys.executable,'-c',script],capture_output=True,text=True,timeout=10)
    assert p.returncode and 'Experiment mode forbids CAN sockets' in p.stderr
