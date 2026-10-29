"""Latched worker-local Hold / checked-return handoff. No mode change or disable."""
import math
from .mit_center_return import MitCenterReturn
from .generated_can import QueuePressure


class PoweredPathHold:
    def __init__(self, sample, now, bias=0., *, kp=40., kd=1.8):
        self.controller=MitCenterReturn(sample.position,sample.position,bias,now,kp=kp,kd=kd)
        self.pressure=QueuePressure()

    def update(self,sample,now,goal,lower,upper):
        if (sample is None or now-sample.at>.020 or sample.state!=2
                or not all(math.isfinite(v) for v in (sample.position,sample.velocity,goal))):
            raise RuntimeError('Powered Hold/return needs fresh running feedback')
        if not lower<=goal<=upper or not lower<=sample.position<=upper:
            raise RuntimeError('Powered Hold/return outside model bounds; not clipped')
        self.controller.goal=goal
        return self.controller.update(sample.position,now,measured_velocity=sample.velocity)

    def send(self,send,sample,now,goal,lower,upper,motor,sign):
        # A refused frame did not move the motor. Preserve the last accepted
        # controller state and recompute from NEW feedback/goal next iteration.
        previous=self.controller.__dict__.copy()
        command=self.update(sample,now,goal,lower,upper)
        try:sent=self.pressure.send(send,command.frame(motor,sign),now)
        except Exception:
            self.controller.__dict__.update(previous);raise
        if not sent:self.controller.__dict__.update(previous)
        return sent


def recover_worker(interface, side, lower, upper, goal, stop, heartbeat, sender,
                   channel_factory=None):
    """Replace a DEAD strike writer for explicit Return, without arming it.

    Only state/mode queries and the existing MIT powered Hold/path controller.
    No enables, disables, mode/gain-parameter changes, strikes or center target.
    Fresh running feedback and an actual MIT mode readback precede any command.
    """
    import pickle
    import signal
    import time
    from dataclasses import replace
    from centering.motors import packet
    from safe_zone.encoder import request_frame
    from .mit_strike import Joint7Channel, Status
    from .snare_beat import PATH_KP, PATH_KD
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    motor, sign = (7, 1.) if side == 'right' else (6, -1.)
    lo, hi = (lower, upper) if sign == 1 else (-upper, -lower)
    channel = None; held = None; feedback_ok = False; status = Status(phase='recovery feedback startup')
    def publish(value):
        try:sender.send(pickle.dumps(value))
        except BlockingIOError:pass
    try:
        channel = (channel_factory or Joint7Channel)(interface, motor=motor, sign=sign)
        began = time.monotonic(); query_at = mode_at = -math.inf
        queries = QueuePressure()
        while not stop.is_set():
            now = time.monotonic()
            if now-heartbeat.value > .5:raise RuntimeError('Recovery GUI heartbeat lost')
            feedback_ok = False
            channel.receive(); feedback_ok = True
            now = time.monotonic(); sample = channel.sample
            # Receive first. MIT replies normally supply this joint already;
            # query only for startup / an aging sample, never an eight-frame burst.
            if now-query_at >= .01 and (sample is None or now-sample.at >= .01):
                if not queries.send(channel.send,request_frame(motor),now):
                    time.sleep(.00025);continue
                query_at = now
            if channel.mode is None and now-mode_at >= .1:
                if not queries.send(channel.send,packet(17,motor,bytes.fromhex('0570000000000000')),now):
                    time.sleep(.00025);continue
                mode_at = now
            if channel.mode is not None and channel.mode != 0:
                raise RuntimeError('Recovery requires existing MIT mode; no mode change attempted')
            valid = (sample is not None and -.001 <= now-sample.at <= .020 and sample.state == 2
                     and all(math.isfinite(x) for x in (sample.position,sample.velocity,sample.torque))
                     and lo <= sample.position <= hi)
            if not valid or channel.mode is None:
                if held is not None or now-began > 3.:
                    raise RuntimeError('Recovery needs fresh running in-bounds encoder feedback')
                time.sleep(.002); continue
            if not math.isfinite(goal.value):
                goal.value = sign*sample.position; held = None
            if held is None:
                held = PoweredPathHold(sample,now,sample.torque,
                    kp=40. if side=='right' else PATH_KP,kd=1.8 if side=='right' else PATH_KD)
            if stop.is_set():break
            if not held.send(channel.send,sample,now,sign*goal.value,lo,hi,motor,sign):
                time.sleep(.00025);continue
            status = replace(status,phase='generated powered hold/return',ready=True,sample=sample,error=None)
            publish(status); time.sleep(.005)
    except Exception as exc:
        publish(replace(status,phase='fault',ready=False,error=str(exc)))
        # Preserve the same fresh measured Hold as ordinary generated faults.
        # Never substitute an old goal or retry a strike after a failed return.
        if feedback_ok and channel is not None and channel.mode == 0 and not stop.is_set():
            try:
                sample=channel.sample; now=time.monotonic()
                if sample is not None and -.001 <= now-sample.at <= .020 and sample.state==2 and lo<=sample.position<=hi:
                    frozen=PoweredPathHold(sample,now,sample.torque,
                        kp=40. if side=='right' else PATH_KP,kd=1.8 if side=='right' else PATH_KD)
                    channel.send(frozen.update(sample,now,sample.position,lo,hi).frame(motor,sign))
            except Exception:pass
    finally:
        if channel is not None:channel.close()
        sender.close()


from .mit_strike import Joint7Session


class RecoverySession(Joint7Session):
    """Existing joined MIT lifecycle, restricted to Hold and checked return."""
    def __init__(self,bus,side,lower,upper):
        import os
        import multiprocessing as mp
        import socket
        import time
        from .mit_strike import Status
        if os.environ.get('STRIKE_LAB_OFFLINE_ONLY')=='1':
            raise RuntimeError('Physical recovery transport forbidden in offline tests')
        if side not in ('right','left'):raise ValueError('Unknown recovery arm')
        self.bus=bus; self.side=side; self.motor=7 if side=='right' else 6
        self.session_attr=f'{side}_joint{self.motor}_session'
        if not bus.active or getattr(bus,self.session_attr,None) is not None:
            raise RuntimeError('Recovery requires powered, exclusively joined strike ownership')
        self.lower,self.upper=float(lower),float(upper); self.closed=False
        ctx=mp.get_context('spawn')
        self.stop_event=ctx.Event(); self.path_hold=ctx.Event(); self.path_hold.set()
        self.path_target=ctx.Value('d',float('nan')); self.goal=self.path_target
        self.heartbeat=ctx.Value('d',time.monotonic()); self.requests=ctx.Queue(maxsize=1)
        self.receiver,sender=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.receiver.setblocking(False); sender.setblocking(False)
        self._status=Status(phase='recovery feedback startup')
        self.process=ctx.Process(target=recover_worker,name=side+'-checked-return-hold',args=(
            bus.sockets[side].getsockname()[0],side,self.lower,self.upper,self.goal,
            self.stop_event,self.heartbeat,sender))
        setattr(bus,self.session_attr,self)
        if side=='left':bus.left_drive.left_joint6_session=self
        try:self.process.start()
        except Exception:
            setattr(bus,self.session_attr,None)
            if side=='left':bus.left_drive.left_joint6_session=None
            self.receiver.close(); self.requests.close(); raise
        finally:sender.close()

    @property
    def status(self):
        import time
        self.heartbeat.value=time.monotonic()
        return super().status

    @property
    def hold_position(self):
        return self.goal.value if math.isfinite(self.goal.value) else None

    def emergency_hold(self):self.goal.value=float('nan')

    def path_goal(self,value):
        if not math.isfinite(value) or not self.lower<=value<=self.upper:
            raise ValueError('Recovery target outside retained joint bounds')
        self.goal.value=float(value)

    def stop(self):
        super().stop()
        if self.side=='left' and self.bus.left_drive.left_joint6_session is self:
            self.bus.left_drive.left_joint6_session=None
