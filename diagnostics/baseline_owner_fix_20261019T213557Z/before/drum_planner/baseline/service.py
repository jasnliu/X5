"""Separate motor-owner service. GUI loss means HOLD, not disable or auto-return.

An independent user service survives GUI/editor closure. Local authenticated IPC
can reconnect with `baseline --resume SESSION`. No execution time limit kills a
holding controller. Closing the window leaves the owner holding. Controller
exit and manual torque off are separate explicit commands.
"""
import argparse,json,os,secrets,signal,socket,struct,subprocess,sys,time,uuid
from pathlib import Path
from ..common import ROOT,atomic_json
from .motion import Controller,HOLD_PHASES,RELAX_PHASES,TERMINAL

MAX_MESSAGE=8192
HEARTBEAT_TIMEOUT=1.


def launch(folder,*,prepared=None,simulate_plan=None,rate=1.):
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=True)
    simulated=simulate_plan is not None
    unit='x5-baseline-owner-'+uuid.uuid4().hex[:12]
    runtime=Path(os.environ.get('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}'))/unit
    runtime.mkdir(mode=0o700)
    info=dict(schema='x5-baseline-control-v1',folder=str(folder),socket=str(runtime/'control.sock'),
              token=secrets.token_hex(32),unit=unit,simulated=simulated,rate=rate,
              prepared=None if prepared is None else str(Path(prepared).resolve()),
              plan=str(Path(simulate_plan).resolve()) if simulated else json.loads(Path(prepared).read_text())['plan'])
    atomic_json(folder/'control.json',info);os.chmod(folder/'control.json',0o600)
    env=[f'--setenv={k}={v}' for k,v in os.environ.items() if k in ('PATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','ROS_DISTRO')]
    args=['systemd-run','--user','--quiet','--collect','--unit='+unit,
          '-p','MemoryAccounting=yes','-p','MemoryMax=512M','-p','MemorySwapMax=0',
          '-p','TasksMax=32','-p','TimeoutStopSec=infinity','-p','KillMode=process',
          '-p','RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK'+('' if simulated else ' AF_CAN'),
          '-p','StandardOutput=append:'+str(folder/'controller.log'),'-p','StandardError=append:'+str(folder/'controller.log'),
          '--setenv=OPENBLAS_NUM_THREADS=1','--setenv=OMP_NUM_THREADS=1','--setenv=PYTHONUNBUFFERED=1',
          '--setenv=PYTHONDONTWRITEBYTECODE=1','--working-directory='+str(ROOT),*env,
          '/usr/bin/python3','-m','drum_planner.baseline.service',str(folder)]
    subprocess.run(args,check=True,timeout=8)
    return info


class Client:
    def __init__(self,info):
        self.info=info;self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        self.socket.settimeout(.2);self.socket.connect(info['socket']);self.socket.setblocking(False)
        self.buffer=b'';self.messages=[];self.latest={};self.last_heartbeat=0.;self.last_received=time.monotonic()
        self.send('hello')
    def send(self,op,**kw):
        data=(json.dumps(dict(op=op,token=self.info['token'],**kw),allow_nan=False)+'\n').encode()
        if len(data)>MAX_MESSAGE:raise ValueError('Control message too large')
        # Each request fits comfortably inside the local socket send buffer.
        n=self.socket.send(data)
        if n!=len(data):raise RuntimeError('Incomplete control write; reconnect before any motion')
    def poll(self):
        now=time.monotonic()
        if now-self.last_heartbeat>.2:self.send('heartbeat');self.last_heartbeat=now
        for _ in range(16):
            try:data=self.socket.recv(65536)
            except BlockingIOError:break
            if not data:
                if self.latest.get('phase') in ('CONTROLLER_EXIT','SUPPORTED_EXIT'):return self.latest
                raise RuntimeError('Motor-owner connection lost; do not assume the arm is relaxed')
            self.buffer+=data;self.last_received=now
            if len(self.buffer)>262144:raise RuntimeError('Oversized controller response')
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1);msg=json.loads(line)
                if 'status' in msg:self.latest=msg['status']
                if msg.get('reply') not in (None,'heartbeat','hello'):self.messages.append(msg)
        if now-self.last_received>2:raise RuntimeError('Controller response stale; no confirmed hold')
        return self.latest
    def close(self):self.socket.close()


class Server:
    def __init__(self,info):
        self.info=info;self.folder=Path(info['folder']);self.engine=None;self.bus=None
        self.clients={};self.owner=None;self.heartbeat=time.monotonic();self.last_status=0.;self.quit=False;self.signal_pending=False
        self.job=None;self.pending=None;self.job_generation=None;self.connection_error='';self.start_error=''
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        self.socket.bind(info['socket']);os.chmod(info['socket'],0o600);self.socket.listen(4);self.socket.setblocking(False)
        self.log=(self.folder/'events.jsonl').open('a',buffering=1)
        if info['simulated']:
            from ..experiment.config import no_can
            no_can()
        self.original_handlers={s:signal.signal(s,self.interrupt) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    def interrupt(self,*_):
        # Do not manipulate an arming generator inside a Python signal handler
        # while it is executing a CAN write. Process the request at the next tick.
        self.signal_pending=True
    def event(self,item):
        self.log.write(json.dumps(item)+'\n');print(item,flush=True)
    def status(self):
        value=self.engine.status() if self.engine else dict(phase='REJECTED' if self.start_error else 'READY',error=self.start_error,started=False,disabled=False,safe_to_close=True,can_return=False)
        return dict(value,simulated=self.info['simulated'],session=str(self.folder),
                    recovery_check_running=self.job is not None,connection_error=self.connection_error)
    def reply(self,s,op=None,error=None):
        payload=(json.dumps(dict(status=self.status(),reply=op,error=error),allow_nan=False)+'\n').encode()
        try:
            n=s.send(payload)
            if n!=len(payload):self.drop(s)
        except OSError:self.drop(s)
    def drop(self,s):
        if s is self.owner:
            self.owner=None
            if self.engine and self.engine.started and not self.engine.disabled:
                self.engine.halt('GUI disconnected; holding, reconnect to recover',fault=True)
        self.clients.pop(s,None)
        try:s.close()
        except OSError:pass
    def command(self,msg):
        op=msg['op']
        if op in ('heartbeat','hello'):self.heartbeat=time.monotonic();return
        if op=='start':
            if self.engine is not None:raise ValueError('Session already used; no automatic restart')
            if msg.get('confirmation')!='MOVE RIGHT ARM':raise ValueError('Explicit MOVE RIGHT ARM confirmation required')
            if self.info['simulated']:
                from ..trajectory_view import PlanDisplay
                from ..common import JOINTS
                from .simulation import SimulatedBus
                import xml.etree.ElementTree as ET
                plan=PlanDisplay(self.info['plan']);obj=plan.obj;req=plan.req.copy()
                root=ET.parse(req['urdf']).getroot()
                req['execution_joint_limits']=[[float(root.find(f"joint[@name='{j}']/limit").get(k)) for k in ('lower','upper')] for j in JOINTS]
                self.bus=SimulatedBus(req)
            else:
                from .tasks import check_packet
                from .transport import BaselineMotors
                packet,_=check_packet(self.info['prepared']);obj=packet['object'];req=packet['request']
                self.bus=BaselineMotors(self.folder,req['start'])
            self.engine=Controller(self.bus,obj,req,rate=self.info['rate'],event=self.event)
            self.engine.start();self.heartbeat=time.monotonic()
        elif op=='hold':
            if self.engine:self.engine.halt('Operator EMERGENCY HOLD')
        elif op=='retry_hold':
            if self.engine:self.engine.retry_hold()
        elif op=='relax':
            if self.engine is None:raise ValueError('No active controller; this window has not enabled any motors')
            self.engine.relax_now()
            self.pending=None
            if self.job:self.job.close();self.job=None
        elif op=='reverse':
            if self.engine is None:raise ValueError('No active trajectory')
            request=self.engine.request_return(True)  # authenticated button click, not a second confirmation
            self.job_generation=request.pop('generation')
            if self.info['simulated']:
                from .paths import recovery
                spec=recovery(self.engine.curve,request['held'],request['tau'],request['rate'])
                spec['proof']=dict(all_valid=True,simulation_only=True,not_a_hardware_validation=True)
                self.pending=spec
            else:
                from ..experiment.runtime import Job
                folder=self.folder/('return-'+uuid.uuid4().hex[:8]);folder.mkdir()
                request['prepared']=self.info['prepared'];atomic_json(folder/'request.json',request)
                self.job=Job(['/usr/bin/python3','-m','drum_planner.baseline.tasks','return','--folder',folder,
                    '--input',folder/'request.json'],folder,timeout=180)
        elif op=='supported_exit':
            if self.engine:self.engine.supported_exit()
            self.quit=True
        elif op=='close':
            if self.engine and not self.engine.status()['safe_to_close']:
                self.engine.halt('Close requested while active; held, NOT relaxed')
                raise ValueError('Controller is holding. Use Reverse, Relax right arm now, or Exit controller (no motor change)')
            self.quit=True
        elif op=='inject_fault' and self.info['simulated']:
            if not self.bus:raise ValueError('Start the simulated cycle first')
            kind=msg.get('kind')
            if kind=='contact':self.bus.contact=True
            elif kind=='can':self.bus.fault='CAN failure'
            elif kind=='motor':self.bus.fault='motor overtemperature/stall fault'
            elif kind=='clear':self.bus.contact=False;self.bus.fault=''
            else:raise ValueError('Unknown simulation fault')
        else:raise ValueError('Unknown or unavailable control command')
    def tick(self):
        if self.signal_pending:
            self.signal_pending=False
            if self.engine and (self.engine.started or getattr(self.bus,'enable_attempted',False)) and not self.engine.disabled:
                self.engine.halt('Controller signal / terminal closed',fault=True)
            else:self.quit=True;return
        try:
            connection,_=self.socket.accept();connection.setblocking(False)
            uid=struct.unpack('3i',connection.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))[1]
            if uid!=os.getuid() or len(self.clients)>=4:connection.close()
            else:self.clients[connection]=b''
        except BlockingIOError:pass
        for s in list(self.clients):
            if s not in self.clients:continue
            try:data=s.recv(MAX_MESSAGE)
            except BlockingIOError:continue
            except OSError:self.drop(s);continue
            if not data:self.drop(s);continue
            data=self.clients[s]+data
            if len(data)>MAX_MESSAGE:self.drop(s);continue
            for _ in range(8):
                if b'\n' not in data:break
                line,data=data.split(b'\n',1)
                try:
                    msg=json.loads(line)
                    if not secrets.compare_digest(msg.get('token',''),self.info['token']):raise ValueError('Authentication failed')
                    if s is not self.owner:
                        if msg.get('op')!='hello':raise ValueError('Connect with hello first')
                        if self.owner:self.drop(self.owner)
                        self.owner=s
                    self.command(msg);self.reply(s,msg['op'])
                except Exception as e:
                    if isinstance(locals().get('msg'),dict) and msg.get('op')=='start' and not getattr(self.bus,'enable_attempted',False):self.start_error=str(e)
                    self.reply(s,msg.get('op','invalid') if isinstance(locals().get('msg'),dict) else 'invalid',str(e))
            if s in self.clients:self.clients[s]=data
        now=time.monotonic()
        if self.engine:
            if self.engine.started and not self.engine.disabled and now-self.heartbeat>HEARTBEAT_TIMEOUT and self.engine.phase not in HOLD_PHASES|RELAX_PHASES:
                self.engine.halt('GUI heartbeat lost; no automatic recovery or relaxation',fault=True)
            self.engine.step()
            if self.engine.phase!='CHECKING_RETURN':
                self.pending=None
                if self.job:self.job.close();self.job=None
            if self.job:
                code=self.job.poll()
                if code is not None:
                    folder=self.job.folder;self.job.close();self.job=None
                    try:
                        if code!=0:raise ValueError(json.loads((folder/'error.json').read_text())['error'])
                        self.pending=json.loads((folder/'return.json').read_text())
                    except Exception as e:self.engine.halt('Return check failed: '+str(e),fault=True)
            if self.pending:
                try:
                    # Gather a new quiet hold window after the checker finishes.
                    if self.owner is None or now-self.heartbeat>HEARTBEAT_TIMEOUT:
                        raise ValueError('Operator connection lost during return verification; remain held')
                    if self.engine.now()-self.engine.phase_at>=.55 and self.engine.hold_confirmed:
                        self.engine.accept_return(self.pending,self.job_generation);self.pending=None
                except Exception as e:self.pending=None;self.engine.halt('Return rejected: '+str(e),fault=True)
        if now-self.last_status>.1:
            self.last_status=now
            if self.owner:self.reply(self.owner)
            atomic_json(self.folder/'status.json',self.status())
    def run(self):
        try:
            while not self.quit:
                try:self.tick()
                except BaseException as e:
                    self.connection_error=str(e)
                    if self.engine and self.engine.started and not self.engine.disabled:self.engine.halt('Controller error: '+str(e),fault=True)
                    else:self.quit=True
                time.sleep(.008)
        finally:
            # run never exits a live hold except explicit supported_exit.
            if self.job:self.job.close()
            if self.bus:self.bus.close()  # descriptors only; never a relax
            for s in list(self.clients):self.drop(s)
            self.socket.close();self.log.close()
            Path(self.info['socket']).unlink(missing_ok=True)
            atomic_json(self.folder/'final_status.json',self.status())
            for s,h in self.original_handlers.items():signal.signal(s,h)


def main():
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args()
    info=json.loads((Path(a.folder)/'control.json').read_text())
    print('Independent baseline owner started; NO enable until authenticated explicit UI START + automatic checks',flush=True)
    Server(info).run()
if __name__=='__main__':main()
