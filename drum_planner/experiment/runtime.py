"""One memory-limited, cancellable, process-tree-contained job at a time."""
import json,os,select,subprocess,time,uuid
from pathlib import Path
from ..common import ROOT,atomic_json,digest

PYTHON=ROOT/'.venv-planner-experiment/bin/python'
BINARY=ROOT/'drum_planner/build/experiment/experiment_oracle'


def build():
    names=('experiment_oracle.cpp','moveit_plan.cpp','CMakeLists.txt')
    signature='\n'.join(digest(ROOT/'drum_planner/cpp'/n) for n in names)
    stamp=BINARY.parent/'signature.txt'
    if BINARY.is_file() and stamp.is_file() and stamp.read_text()==signature:return BINARY
    subprocess.run(['cmake','-S',str(ROOT/'drum_planner/cpp'),'-B',str(BINARY.parent),
                    '-DCMAKE_BUILD_TYPE=Release','-DPython3_EXECUTABLE=/usr/bin/python3'],check=True,timeout=120)
    subprocess.run(['cmake','--build',str(BINARY.parent),'--target','experiment_oracle','-j1'],check=True,timeout=180)
    stamp.write_text(signature);return BINARY


class Job:
    def __init__(self,command,folder,*,timeout=240,allow_can=False):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.unit='x5-experiment-'+uuid.uuid4().hex[:12]
        self.log_path=self.folder/'worker.log';self.log=self.log_path.open('wb');self.offset=0;self.cancelled=False
        self.started=time.monotonic();self.last_limits=0.;self.last_stop=0.;self.limits={};self.reaped=False
        env=[f'--setenv={k}={v}' for k,v in os.environ.items() if k in (
            'PATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','ROS_DISTRO','DISPLAY','XAUTHORITY',
            'XDG_CACHE_HOME','X5_PLANNER_CACHE_DIR')]
        args=['systemd-run','--user','--wait','--pipe','--quiet','--unit='+self.unit,
              '-p','MemoryAccounting=yes','-p','MemoryMax=3G','-p','MemorySwapMax=128M',
              '-p','TasksMax=96','-p','OOMPolicy=stop','-p','KillMode=control-group',
              '-p','TimeoutStopSec=3','-p',f'RuntimeMaxSec={int(timeout)}',
              '-p','RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK'+(' AF_CAN' if allow_can else ''),
              '--setenv=OPENBLAS_NUM_THREADS=1','--setenv=OMP_NUM_THREADS=1',
              '--setenv=PYTHONDONTWRITEBYTECODE=1','--setenv=PYTHONUNBUFFERED=1',
              '--working-directory='+str(ROOT),*env,*map(str,command)]
        try:self.process=subprocess.Popen(args,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:self.log.close();raise

    def poll(self):
        self.tail()
        now=time.monotonic()
        # An immediate Cancel can race systemd-run registering the service.
        # Retry until it exits instead of letting that first "unit not found"
        # silently leave the worker running to completion.
        if self.cancelled and self.process.poll() is None and now-self.last_stop>=.25:
            self.cancel()
        if now-self.last_limits>1.:
            self.last_limits=now
            r=subprocess.run(['systemctl','--user','show',self.unit+'.service',
                '-p','MemoryCurrent','-p','MemoryPeak','-p','MemoryMax','-p','MemorySwapMax','-p','ControlGroup',
                '-p','RestrictAddressFamilies','-p','Result'],capture_output=True,text=True,timeout=3)
            p=dict(line.split('=',1) for line in r.stdout.splitlines() if '=' in line)
            if p.get('MemoryMax')=='3221225472':
                old=int(self.limits.get('observed_peak_bytes',0));current=p.get('MemoryCurrent','0')
                p['observed_peak_bytes']=max(old,int(current) if current.isdigit() else 0)
                self.limits=p;atomic_json(self.folder/'containment.json',dict(unit=self.unit,**p))
        code=self.process.poll()
        if code is not None and not self.reaped:
            self.log.close();self.tail();self.reaped=True
            atomic_json(self.folder/'job.json',dict(exit_code=code,cancelled=self.cancelled,
                elapsed_s=now-self.started,unit=self.unit,containment=self.limits))
        return code

    def tail(self):
        # Stream to terminal, never accumulate unbounded log text in the GUI.
        with self.log_path.open('rb') as f:
            f.seek(self.offset);data=f.read(65536);self.offset=f.tell()
        if data:print(data.decode('utf-8',errors='replace'),end='',flush=True)

    def cancel(self):
        if self.process.poll() is None:
            self.cancelled=True
            self.last_stop=time.monotonic()
            subprocess.run(['systemctl','--user','stop','--no-block',self.unit+'.service'],
                           stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=3)

    def close(self):
        self.cancel()
        try:self.process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            subprocess.run(['systemctl','--user','kill','--signal=KILL',self.unit+'.service'],timeout=3,
                           stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            self.process.kill();self.process.wait(timeout=3)
        self.poll()


class Oracle:
    def __init__(self,request_path,folder):
        self.log_path=(Path(folder)/'oracle.log').resolve()
        self.log=self.log_path.open('wb');self.buffer=b'';self.process=None
        try:
            self.check_start_bounds(request_path)
            self.process=subprocess.Popen([str(build()),str(request_path)],stdin=subprocess.PIPE,
                                          stdout=subprocess.PIPE,stderr=self.log,bufsize=0)
            if self.receive(30).get('ready') is not True:raise RuntimeError('FCL oracle did not initialize')
            self.meta=self.call('metadata');self.lower=self.meta['lower'];self.upper=self.meta['upper']
        except BaseException as exc:
            self.log.write((str(exc)+'\n').encode());self.log.flush()
            self.close();raise

    @staticmethod
    def check_start_bounds(request_path):
        """Explain invalid measured coordinates before an opaque native exit."""
        import math,xml.etree.ElementTree as ET
        from ..common import arm_joints
        req=json.loads(Path(request_path).read_text());side=req.get('moving_side','right')
        root=ET.parse(req['urdf']).getroot()
        for field in ('scene_robot_start','start'):
            if field not in req:continue
            q=req[field]
            if len(q)!=7:raise ValueError(f'{side} {field}: expected seven joint positions')
            for name,value in zip(arm_joints(side),q):
                limit=root.find(f"joint[@name='{name}']/limit")
                lo,hi=float(limit.get('lower')),float(limit.get('upper'))
                if not math.isfinite(value) or value<lo-1e-12 or value>hi+1e-12:
                    raise ValueError(f'{side} {field}: {name} = {math.degrees(value):.6f} degrees, '
                        f'outside model [{math.degrees(lo):.6f}, {math.degrees(hi):.6f}] degrees; '
                        'measured pose NOT clipped. Recalculate with current rest/model.')

    def receive(self,timeout):
        started=time.monotonic();end=started+timeout;last_progress=started
        while True:
            if time.monotonic()-last_progress>=5:
                last_progress=time.monotonic()
                print(f"[worker] FCL/solver operation running: {last_progress-started:.0f}s",flush=True)
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                if line.startswith(b'@x5 '):return json.loads(line[4:])
                self.log.write(line+b'\n')
            remaining=end-time.monotonic()
            if remaining<=0:raise TimeoutError('FCL operation exceeded its time budget')
            ready,_,_=select.select([self.process.stdout],[],[],min(.5,remaining))
            if ready:
                data=os.read(self.process.stdout.fileno(),65536)
                if not data:
                    self.process.wait(timeout=3)
                    if self.buffer:self.log.write(self.buffer);self.buffer=b''
                    self.log.flush()
                    with self.log_path.open('rb') as f:
                        f.seek(max(0,self.log_path.stat().st_size-3000));detail=f.read().decode('utf-8',errors='replace').strip()
                    raise RuntimeError(f'FCL worker exited ({self.process.returncode}): {detail or "No native diagnostic"}\nLog: {self.log_path}')
                self.buffer+=data
                if len(self.buffer)>32*1024*1024:raise RuntimeError('Oversized oracle reply')

    def call(self,op,timeout=90,**kwargs):
        data=(json.dumps(dict(op=op,**kwargs),allow_nan=False,separators=(',',':'))+'\n').encode()
        # Unbuffered pipes may perform partial writes on large state batches.
        view=memoryview(data)
        while view:
            n=os.write(self.process.stdin.fileno(),view);view=view[n:]
        result=self.receive(timeout)
        if result.get('error'):raise RuntimeError(result['error'])
        return result

    def check(self,q,**kwargs):
        return self.call('check',q=q.tolist() if hasattr(q,'tolist') else q,**kwargs)

    def close(self):
        if self.process is None:self.log.close();return
        if self.process.poll() is None:
            self.process.stdin.close()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)
        self.process.stdin.close();self.process.stdout.close();self.log.close()
