"""One serial owner; finite bass K and hi-hat B/O schedule. No force changes.

No firmware flashing or new motor limits. Protocol-v2 encoder telemetry must
verify hi-hat open before the normal accessory release.
"""
import fcntl
import json
import multiprocessing as mp
import os
from pathlib import Path
import queue
import signal
import time

from camera_playback.hihat import HiHatController
from hihat_collection.plan import CLOSE_DEGREES, OPEN_SETTLE_SECONDS
from .plan import PULSE


def worker(port, directory, requests, status, heartbeat):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    stop_signal = False
    def stop(*_):
        nonlocal stop_signal
        stop_signal = True
    signal.signal(signal.SIGTERM,stop)
    directory=Path(directory)
    stream=(directory/'hihat_commands.jsonl').open('w',buffering=1)
    def log(kind,**data):
        stream.write(json.dumps(dict(t=time.monotonic(),kind=kind,**data))+'\n')
    class LoggedController(HiHatController):
        released_ack = None
        def _write(self,command):
            if command not in (b'K',b'B',b'O',b'H',b'S',b'Q',f'A{CLOSE_DEGREES}\n'.encode()):
                raise RuntimeError('Unapproved hi-hat command')
            super()._write(command)
            if command!=b'H':log('command',command=command.decode())
        def _handle_line(self,line):
            log('firmware',line=line)
            if line.startswith('STOP -- all motors released'):
                self.released_ack=time.monotonic()
            super()._handle_line(line)
    controller=LoggedController(port)
    schedule=[];block=None;completed=None;closing_count=0;kick_count=0;error=None;lock=None
    def publish(state):
        value=dict(state=state,block=block,completed=completed,closures=closing_count,
                   error=error,kicks=kick_count,t=time.monotonic())
        while True:
            try:status.get_nowait()
            except queue.Empty:break
        status.put(value)
    try:
        lock=os.open('/tmp/x5-hihat-collection.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if not controller.connect():raise RuntimeError(controller.detail)
        deadline=time.monotonic()+5
        while not controller.ready():
            controller.tick(); publish('starting')
            if controller.state=='error' or time.monotonic()>deadline:
                raise RuntimeError(controller.detail)
            time.sleep(.005)
        # Current protocol requires a verified OPEN, then release, before A90.
        controller.calibration_edge(False)
        deadline=time.monotonic()+2.5
        while True:
            controller.tick(); controller.query_status(); time.sleep(.02); controller.tick(); publish('starting')
            if controller.state=='error': raise RuntimeError(controller.detail)
            t=controller.telemetry
            if t and t['opened']==1 and t['target']==0 and t['released']==1: break
            if time.monotonic()>deadline: raise RuntimeError('Hi-hat open encoder verification failed')
        controller._write(b'S'); time.sleep(.02); controller.tick()
        controller.set_calibration_angle(CLOSE_DEGREES)
        deadline=time.monotonic()+1.
        while controller.angle_ack is None:
            controller.tick(); publish('starting')
            if controller.state=='error' or time.monotonic()>deadline:
                raise RuntimeError('90-degree hi-hat angle acknowledgment missing')
            time.sleep(.005)
        if controller.angle_ack[:2] != (CLOSE_DEGREES,controller.counts_for(CLOSE_DEGREES)):
            raise RuntimeError('Hi-hat angle acknowledgment mismatch')
        log('target_confirmed',degrees=CLOSE_DEGREES,counts=controller.angle_ack[1])
        last_publish=0.
        while not stop_signal:
            now=time.monotonic()
            if now-heartbeat.value>2.:
                raise RuntimeError('Collection parent heartbeat lost')
            controller.tick(now)
            if controller.state=='error':raise RuntimeError(controller.detail)
            try:request=requests.get_nowait()
            except queue.Empty:request=None
            if request:
                action,*args=request
                if action=='stop':break
                if action!='block' or schedule:raise RuntimeError('Overlapping/unknown hi-hat request')
                block,events=args
                if not events or any(c not in ("K","B","O") for _,c in events):
                    raise RuntimeError("Invalid accessory events")
                if events!=sorted(events): raise RuntimeError("Events out of order")
                if events[0][0]<now+.03:raise RuntimeError('Hi-hat schedule was not submitted ahead of time')
                schedule=[(float(t),c.encode()) for t,c in events]
                log('block',block=block,events=events,pulse=PULSE)
            if schedule and now>=schedule[0][0]:
                due,expected=schedule.pop(0)
                if now-due>.08:raise RuntimeError('Hi-hat schedule missed 80 ms deadline')
                if expected==b'K':
                    controller.sequence_active=True
                    controller.send_kick(); kick_count+=1
                else: controller.calibration_edge(expected==b'B')
                actual=expected
                log('beat',block=block,command=actual.decode(),due=due)
                closing_count+=actual==b'B'
                if not schedule:
                    completed=block
                    controller.sequence_active=False
            if now-last_publish>.05:
                publish('scheduled' if schedule else 'idle');last_publish=now
            time.sleep(.002)
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}';log('error',error=error)
    finally:
        shutdown_error=None;ack=False;open_sent=None
        try:
            if controller.fd is not None:
                # Same O return as swing. Continue heartbeat for its normal return.
                controller._write(b'O');open_sent=time.monotonic()
                until=open_sent+OPEN_SETTLE_SECONDS
                verified_open=False
                while time.monotonic()<until:
                    publish('stopping')
                    controller.tick(); controller.query_status();time.sleep(.02)
                    t=controller.telemetry
                    if (controller.telemetry_at and controller.telemetry_at>=open_sent
                            and t['opened']==1 and t['target']==0 and t['released']==1):
                        verified_open=True
                if not verified_open: raise RuntimeError('Hi-hat did not verify open before release')
                controller._write(b'S');released_at=time.monotonic()
                until=released_at+.6
                while time.monotonic()<until:
                    controller._drain()
                    if controller.released_ack and controller.released_ack>=released_at:
                        ack=True;break
                    time.sleep(.005)
                if not ack:raise RuntimeError('Hi-hat release acknowledgement missing')
        except BaseException as exc:
            shutdown_error=str(exc);error=error or shutdown_error
        controller.close()
        result=dict(error=error,shutdown_error=shutdown_error,closures_commanded=closing_count,
                    open_command_at=open_sent,normal_open_return_wait_s=OPEN_SETTLE_SECONDS,
                    release_ack=ack,encoder_position_verified=bool(locals().get("verified_open",False)),
                    open_proof='firmware latched encoder-zero return; mechanism may coast afterward',
                    final_telemetry=controller.telemetry,
                    kicks_commanded=kick_count,bass_pulse=PULSE,finished_at=time.monotonic())
        (directory/'hihat_result.json').write_text(json.dumps(result,indent=2)+'\n')
        publish('stopped')
        stream.close()
        if lock is not None:os.close(lock)


class BassSession:
    def __init__(self,port,directory):
        if os.environ.get('PLAYBACK_OFFLINE_ONLY')=='1':
            raise RuntimeError('Hardware forbidden by offline environment')
        context=mp.get_context('spawn')
        self.requests=context.Queue();self.status_queue=context.Queue()
        self.heartbeat=context.Value('d',time.monotonic())
        self.status=dict(state='starting',error=None,t=time.monotonic())
        self.process=context.Process(target=worker,args=(port,directory,self.requests,self.status_queue,self.heartbeat),name='bass-finite-collection')
        self.process.start();self.stopping=False

    def check(self):
        self.heartbeat.value=time.monotonic()
        while True:
            try:self.status=self.status_queue.get_nowait()
            except queue.Empty:break
        if self.status['error']:raise RuntimeError(self.status['error'])
        if not self.process.is_alive() and not self.stopping:
            raise RuntimeError('Hi-hat worker stopped unexpectedly')
        if time.monotonic()-self.status['t']>1.:
            raise RuntimeError('Hi-hat worker feedback stale')
        return self.status

    def block(self,number,events):
        if self.check()['state']!='idle':raise RuntimeError('Hi-hat not idle')
        self.requests.put(('block',number,events))

    def request_stop(self):
        if not self.stopping:
            self.stopping=True;self.requests.put(('stop',))

    def stop(self):
        self.request_stop()
        deadline=time.monotonic()+4.
        while self.process.is_alive() and time.monotonic()<deadline:
            self.heartbeat.value=time.monotonic();self.process.join(.02)
        if self.process.is_alive():
            # SIGTERM requests the same O->return->S path; never SIGKILL a live driver here.
            self.process.terminate();self.process.join(3.)
        if self.process.is_alive():raise RuntimeError('Hi-hat shutdown did not complete; operator attention required')
