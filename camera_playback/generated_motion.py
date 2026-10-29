"""Dual-arm sequence adapter. Exact curves and the existing playback supervisor.

No sockets, no camera, no planning on the control thread. Explicit Start/Return
are the only entries to movement; faults and closure never trigger a return.
"""
from collections import deque
import time
import numpy as np
from .trajectory import PlaybackFollower
from .left_hold import LEFT_GRIPPER_TARGET
from centering.motors import RIGHT_GRIPPER_CLOSED
from drum_planner.baseline.paths import blend, recovery_at


class Motion:
    def __init__(self,bus,curves,*,clock=time.monotonic):
        self.bus,self.curves,self.clock=bus,curves,clock
        self.rest={s:c.at(0) for s,c in curves.items()}
        self.progress={s:0. for s in curves}
        self.targets={s:q.copy() for s,q in self.rest.items()}
        self.followers={s:PlaybackFollower() for s in curves}
        self.stage='READY';self.at=clock();self.previous=None;self.setup=None
        self.error='';self.held=None;self.return_steps=[];self.return_index=0
        self.rest_release_at=None;self.last_hold=0.
        self.pending_worker_hold=False
        self.stream=None;self.stream_side=None;self.stream_held={}
        self.events=[]

    def emit(self,stage):
        self.stage=stage;self.at=self.clock();self.previous=self.at
        self.events.append((self.at,stage));print('GENERATED BEAT: '+stage,flush=True)

    def positions(self):return {s:self.bus.positions_for(s) for s in self.curves}

    def grippers_closed(self):
        return all(abs(self.bus.states[side,8][0]-goal)<=np.radians(1.)
                   for side,goal in (('left',LEFT_GRIPPER_TARGET),('right',RIGHT_GRIPPER_CLOSED)))

    def fresh(self,*,running=True,grippers=True,configuring=()):
        if not self.bus.fresh() or len(self.bus.states)!=16 or any(self.clock()-v[2]>.12 for v in self.bus.states.values()):
            raise RuntimeError('Fresh feedback missing; powered hold cannot be confirmed')
        if running and any(v[1]!=2 for key,v in self.bus.states.items() if key not in configuring):
            raise RuntimeError('An arm/gripper is not running; powered hold cannot be confirmed')
        if running and grippers and not self.grippers_closed():raise RuntimeError('Custom gripper lost its closed position')
        q=self.positions()
        if not all(np.isfinite(v).all() for v in q.values()):raise RuntimeError('Nonfinite feedback')
        return q

    def start(self):
        if self.stage!='READY':raise RuntimeError('New captured mission required before another Start')
        q=self.fresh(running=False)
        if any(v[1]!=0 for v in self.bus.states.values()):raise RuntimeError('Both arms must initially be relaxed')
        if any(np.sum(abs(q[s]-self.rest[s]))>.01 for s in q):raise RuntimeError('Arm moved since capture; capture and calculate again')
        self.setup=self.bus.arm_at_rest();self.emit('ARMING')

    def hold(self,reason='Operator Emergency Hold'):
        if self.setup is not None:self.setup.close();self.setup=None
        self.stop_stream()  # No GUI Hold/return target may race a path writer.
        already=self.stage in ('HOLD','LOADING PREPARED RETURN')
        if not already:self.held=None
        self.error=str(reason);self.emit('HOLD')
        self.pending_worker_hold=True
        if self.bus.active:
            try:
                if self.held is None:self.held=self.fresh(running=False)
                self.held.update(self.stream_held);self.stream_held={}
                for s,q in self.held.items():self.bus.command_arm(s,q,hold=True)
                self.targets={s:q.copy() for s,q in self.held.items()}
            except Exception as exc:self.error+='; HOLD UNCONFIRMED: '+str(exc)
        return self.error

    def return_input(self):
        if self.stage!='HOLD':raise RuntimeError('Hold first before checking return')
        if self.pending_worker_hold:
            result=getattr(self.bus,'sync_worker_holds',lambda values:values)(self.targets)
            if result is None:raise RuntimeError('Waiting for strike workers to confirm their stationary hold')
            self.targets=self.held=result;self.pending_worker_hold=False
        q=self.fresh(running=False)
        if not self.bus.settled(self.targets):raise RuntimeError('Wait for 0.6 seconds of stationary hold feedback, then retry Return')
        self.held=q;self.emit('LOADING PREPARED RETURN')
        return dict(held={s:v.tolist() for s,v in q.items()},progress=self.progress.copy())

    def accept_return(self,result):
        if self.stage!='LOADING PREPARED RETURN':raise RuntimeError('Return cancelled; no queued motion')
        q=self.fresh(running=False)
        if any(np.sum(abs(q[s]-result['held'][s]))>.01 for s in q):raise RuntimeError('Arm moved during return check; hold and retry')
        if result.get('status')!='ready' or result['progress']!=self.progress:raise RuntimeError('Invalid return result')
        self.return_steps=result['steps'];self.return_index=0
        self.return_curves={}
        if not getattr(self.bus,'streaming_enabled',False):
            from drum_planner.baseline.curve import Curve
            self.return_curves={i:Curve(step['plan']) for i,step in enumerate(self.return_steps) if step['kind']=='planned'}
        # Hardware constructs/validates new exact curves in the existing child,
        # not here: two Curve() calls can starve powered MIT heartbeats >1 s.
        self.followers={s:PlaybackFollower() for s in self.curves}
        if not self.return_steps:
            self.targets={s:q.copy() for s,q in self.rest.items()}
            self.emit('SETTLE REST')  # Interrupted setup at the actual rest; no need to enable other drives.
        else:self.emit('RETURN')

    def send(self,side,target):
        self.bus.command_arm(side,target);self.targets[side]=np.asarray(target).copy()

    def _stream_update(self,update):
        side=self.stream_side
        self.targets[side]=update['target'].copy();self.bus.target[side]=update['target'].copy()
        if update['tau'] is not None:self.progress[side]=update['tau']

    def stop_stream(self):
        if self.stream is None:return
        update=self.stream.stop();self._stream_update(update)
        if update['result'] and update['result'].get('held') is not None:
            self.stream_held={self.stream_side:np.asarray(update['result']['held'])}
        self.stream.detach();self.stream=None;self.stream_side=None

    def _stream_step(self,stage):
        side=stage.lower() if stage in ('LEFT','RIGHT') else self.return_steps[self.return_index]['side']
        step=None if stage in ('LEFT','RIGHT') else self.return_steps[self.return_index]
        if self.stream is None:
            self.stream_side=side
            self.stream=self.bus.start_stream(self.curves[side],side,step)
        update=self.stream.poll();self._stream_update(update)
        if update['done']:
            if update['result'].get('held') is not None:
                self.stream_held={side:np.asarray(update['result']['held'])}
            self.stream.detach();self.stream=None;self.stream_side=None
            if not update['result']['success']:
                raise RuntimeError(update['result']['error'])
            self.emit('SETTLE '+stage)

    def step(self):
        now=self.clock();stage=self.stage
        if stage in ('READY','DONE','ARRIVED','WAIT PRIME','RELAXED'):return
        if stage=='ARMING':
            self.fresh(running=False)
            if now-self.at>25:raise RuntimeError('Current-pose setup timeout')
            try:next(self.setup)
            except StopIteration:self.setup=None;self.emit('SETTLE START')
            return
        if stage in ('HOLD','LOADING PREPARED RETURN'):
            if self.held is None and self.bus.active:
                self.held=self.fresh(running=False)
                self.targets={s:q.copy() for s,q in self.held.items()}
            if self.pending_worker_hold and self.held is not None:
                sync=getattr(self.bus,'sync_worker_holds',lambda values:values)
                result=sync(self.targets)
                if result is not None:
                    self.targets=self.held=result;self.pending_worker_hold=False
            # Keep worker heartbeats alive; NEVER rebase a fixed hold to drift.
            if self.held is not None and self.bus.active and now-self.last_hold>=.1:
                self.fresh(running=False)
                for s,q in self.targets.items():self.bus.command_arm(s,q,hold=True)
                self.last_hold=now
            return
        if stage=='RELAXING':
            self.fresh(running=False)
            if all(v[1]==0 and v[2]>=self.rest_release_at for v in self.bus.states.values()):self.emit('DONE')
            elif now-self.rest_release_at>1.:raise RuntimeError('Rest release not confirmed; inspect motor feedback')
            return
        q=self.fresh(running=stage!='SETTLE REST',grippers=stage!='SETTLE START')
        if stage in ('LEFT','RIGHT','RETURN') and getattr(self.bus,'streaming_enabled',False):
            # The isolated writer enforces the 80 ms command deadline, feedback,
            # tracking and envelope checks. Tk does not clock moving targets.
            self.previous=now
            self._stream_step(stage)
            return
        if self.previous is not None and now-self.previous>.08:raise RuntimeError('Control scheduler missed 80 ms; stopped without catch-up')
        self.previous=now
        for s in self.curves:self.followers[s].update(q[s],self.targets[s],now)
        if stage.startswith('SETTLE'):
            if stage=='SETTLE START' and not self.grippers_closed():
                if now-self.at>3.:raise RuntimeError('Custom gripper did not reach its closed target')
                return
            if self.bus.settled(self.targets):
                if stage=='SETTLE START':self.emit('LEFT')
                elif stage=='SETTLE LEFT':self.emit('RIGHT')
                elif stage=='SETTLE RIGHT':self.emit('ARRIVED')
                elif stage=='SETTLE RETURN':
                    self.return_index+=1
                    self.emit('SETTLE REST' if self.return_index==len(self.return_steps) else 'RETURN')
                else:
                    try:self.bus.release_rest()
                    except RuntimeError as exc:
                        if 'Refresh settled rest' not in str(exc):raise
                        return
                    self.rest_release_at=now;self.emit('RELAXING')
            elif now-self.at>8.:raise RuntimeError('Endpoint did not settle; NOT relaxed')
            return
        if stage in ('LEFT','RIGHT'):
            side=stage.lower();curve=self.curves[side];tau=min(curve.duration,now-self.at)
            self.send(side,curve.at(tau));self.progress[side]=tau
            if tau==curve.duration:self.emit('SETTLE '+stage)
        elif stage=='RETURN':
            step=self.return_steps[self.return_index];side=step['side'];elapsed=min(step['duration'],now-self.at)
            if step['kind']=='prefix':
                target,tau,_=recovery_at(self.curves[side],step['spec'],elapsed)
                self.progress[side]=tau
            elif step['kind']=='planned':target=self.return_curves[self.return_index].at(elapsed)
            else:
                target=np.array(step['start'])+(np.array(step['goal'])-step['start'])*blend(elapsed/step['duration'])
            self.send(side,target)
            if elapsed==step['duration']:self.emit('SETTLE RETURN')
