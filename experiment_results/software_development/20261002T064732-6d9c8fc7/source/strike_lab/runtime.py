"""Spawned controller process. Tk/ROS and file writing never clock the strike."""
from dataclasses import asdict
import multiprocessing as mp
import queue
import time
import traceback
from .backends import SimBackend, HardwareBackend
from .campaign import Plan, run_campaign
from .config import Rules, Limits, Plant
from .engine import Engine
from .storage import Store


class StopToken:
    def __init__(self,event,heartbeat):self.event,self.heartbeat=event,heartbeat
    def is_set(self):
        if time.monotonic()-self.heartbeat.value>5:
            raise RuntimeError('Experiment supervisor heartbeat lost')
        return self.event.is_set()
    def wait(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if self.is_set():return True
            self.event.wait(min(.05,max(0,deadline-time.monotonic())))
        return self.is_set()


def worker(options,requests,events,stop_event,pause_event,heartbeat):
    backend=store=engine=None
    stop=StopToken(stop_event,heartbeat)
    def publish(data):
        try:
            if data['phase'] in ('TRIAL COMPLETE','BATCH COMPLETE','FAULT','CLOSED','UNPREPARED'):
                events.put(data,timeout=.2)
            else:events.put_nowait(data)
        except queue.Full:pass  # Full-resolution evidence is separately archived.
    try:
        rules=Rules(**options.get('rules',{}));limits=Limits(**options.get('limits',{}))
        plant=Plant(**options.get('plant',{}))
        hardware=options.get('hardware',False)
        store=Store(options['output'],'hardware' if hardware else 'simulation',rules,limits,
                    dict(plant=None if hardware else asdict(plant),options=options))
        publish(dict(phase='UNPREPARED',session=str(store.path),backend=store.backend))
        while not stop.is_set():
            try:request=requests.get_nowait()
            except queue.Empty:
                if engine:engine.idle()
                else:stop.wait(.02)
                continue
            action=request['action']
            if action=='prepare':
                if engine:continue
                backend=(HardwareBackend(limits) if hardware else SimBackend(plant,options.get('realtime',True)))
                engine=Engine(backend,store,stop,publish,rules,limits)
                publish(dict(phase='READY',positions=list(backend.positions),anchor=engine.anchor,session=str(store.path)))
            elif action=='finish':
                break
            elif action in ('run','campaign'):
                if engine is None:raise ValueError('Prepare and center before running trials')
                if action=='campaign':
                    run_campaign(engine,request['methods'],Plan(**request.get('plan',{})),
                                 request.get('stage','all'),pause_event,request.get('parameters',{}))
                else:
                    for _ in range(request.get('repetitions',1)):
                        for method in request['methods']:
                            while pause_event.is_set():engine.idle()
                            engine.trial(method,request.get('parameters',{}).get(method),'manual')
                            until=backend.now()+request.get('interval',.25)
                            while backend.now()<until:engine.idle()
                report=store.report()
                publish(dict(phase='BATCH COMPLETE',report=str(report),session=str(store.path),
                             positions=list(backend.positions)))
                if options.get('exit_after_batch'):break
            else:raise ValueError('Unknown worker request: '+str(action))
    except InterruptedError as exc:
        publish(dict(phase='STOPPED',message=str(exc)))
    except BaseException as exc:
        publish(dict(phase='FAULT',error=str(exc),traceback=traceback.format_exc()))
    finally:
        if backend:
            try:backend.close()
            except Exception as exc:publish(dict(phase='FAULT',error='Relaxation: '+str(exc)))
        if store:
            try:
                report=store.report();store.close()
                publish(dict(phase='CLOSED',session=str(store.path),report=str(report)))
            except Exception as exc:publish(dict(phase='FAULT',error='Archive: '+str(exc)))


class Session:
    def __init__(self,options):
        ctx=mp.get_context('spawn')
        self.requests=ctx.Queue(maxsize=8);self.events=ctx.Queue(maxsize=128)
        self.stop_event=ctx.Event();self.pause_event=ctx.Event()
        self.heartbeat=ctx.Value('d',time.monotonic())
        self.process=ctx.Process(target=worker,args=(options,self.requests,self.events,
                                  self.stop_event,self.pause_event,self.heartbeat),name='j7-strike-lab')
        self.process.start();self.closed=False

    def request(self,action,**values):
        if not self.process.is_alive():raise RuntimeError('Controller process is not running')
        self.requests.put_nowait(dict(action=action,**values))

    def poll(self):
        self.heartbeat.value=time.monotonic()
        messages=[]
        while True:
            try:messages.append(self.events.get_nowait())
            except queue.Empty:break
        return messages

    def pause(self,value=True):
        self.pause_event.set() if value else self.pause_event.clear()

    def stop(self):self.stop_event.set()

    def close(self):
        self.stop()
        deadline=time.monotonic()+5
        while self.process.is_alive() and time.monotonic()<deadline:
            self.poll();self.process.join(.05)
        if self.process.is_alive():raise RuntimeError('Controller has not shut down; do not start another controller')
        self.closed=True
        for q in (self.requests,self.events):q.cancel_join_thread();q.close()
