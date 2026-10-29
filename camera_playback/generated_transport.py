"""Normal generated beat adapter on the EXISTING sole PlaybackMotors owner.

No devices at import. No centering; normal release is at verified captured rest.
Explicit Emergency Relax bypasses pose/feedback gates and attempts every disable.
Worker-owned J7/J6 retain MIT mode through Hold and checked prefix recovery.
"""
from collections import deque
import math
import time
import numpy as np
from centering.motors import Motors, CURRENT, GRIPPER_CURRENT, RIGHT_GRIPPER_CLOSED, RIGHT_PLAYBACK_SPEED, parameter, packet
from safe_zone.encoder import joint_to_motor
from .playback_transport import PlaybackMotors
from .left_hold import LEFT_GRIPPER_TARGET, LeftHoldDrive
from .mit_strike import Command
from .generated_can import FeedbackQueries, receive_without_query_burst


class GeneratedLeftDrive(LeftHoldDrive):
    def _send(self,side,frame):
        # Normal planner --hardware --left already uses this .8 rad/s cap.
        # Admit ONLY that exact setup packet here, not a generic bypass of
        # the legacy beat transport's right-only fast-speed restriction.
        if side=='left' and self.active and any(frame==parameter(i,0x7017,RIGHT_PLAYBACK_SPEED) for i in range(1,8)):
            if self.sockets[side].send(frame)!=16:raise RuntimeError('Short left speed-cap write')
            return
        return super()._send(side,frame)


class GeneratedMotors(PlaybackMotors):
    streaming_enabled=True
    @classmethod
    def from_owner(cls,bus,rest):
        result=cls.adopt(bus,rest['right'],left_hold=True)
        view=GeneratedLeftDrive.__new__(GeneratedLeftDrive)
        view.__dict__.update(result.left_drive.__dict__);result.left_drive=view
        result.rest={s:np.asarray(rest[s],float).copy() for s in ('left','right')}
        result.enabled_drives=set();result.release_permitted=False
        result.target={s:q.copy() for s,q in result.rest.items()}
        result.generated_history=deque(maxlen=100)
        result.handoff_goal=None
        result.feedback_failed_at=None
        result.path_session=None
        result.feedback_queries=FeedbackQueries(.020,max_requests=8)
        return result

    def start_stream(self,curve,side,step):
        from .generated_stream import StreamSession
        if getattr(self,'emergency_latched',False) or not self.active:
            raise RuntimeError('Generated writer requires active operator-authorized movement')
        session=StreamSession(self,curve,side,step,geometries=self.stream_geometry,directory=self.stream_directory)
        self.playback_session=session
        # Spawn may briefly delay the GUI. Reuse the existing read-only drain,
        # rather than weakening feedback freshness on the next callback. The
        # delegate cannot advance until the subsequent live session.poll()
        # acknowledges readiness; this drain is still initialization, not motion.
        from .recording_only import refresh_feedback
        try:refresh_feedback(self,.08)
        except Exception:
            session.stop();session.detach();raise
        return session

    def _guard_stream_write(self,side,frame):
        from safe_zone.encoder import FRAME
        cid,_,data=FRAME.unpack(frame)
        session=getattr(self,'path_session',None)
        if (session is not None and side==session.side and (cid>>24)&31==18
                and data[:2]==b'\x16\x70' and cid&255 in range(1,8)):
            raise RuntimeError('Generated worker owns these joint targets; join it before handoff')

    def _send(self,side,frame):
        self._guard_stream_write(side,frame)
        return super()._send(side,frame)

    def arm_at_rest(self):
        if getattr(self,'emergency_latched',False):raise RuntimeError('Capture again before another Start after Emergency Relax')
        if not self.fresh() or len(self.states)!=16 or any(v[1]!=0 for v in self.states.values()):
            raise RuntimeError('Both arms must have fresh disabled feedback before Start')
        for s in self.rest:
            if np.sum(abs(self.positions_for(s)-self.rest[s]))>.01:raise RuntimeError(s+' moved since capture; recalculate')
        self.active=self.left_drive.active=True
        # Existing confirmed configuration holds each arm at its ACTUAL rest;
        # no custom center, opening sweep or different goal is ever dispatched.
        for side,drive,grip in (('left',self.left_drive,LEFT_GRIPPER_TARGET),('right',self,RIGHT_GRIPPER_CLOSED)):
            for motor,q in enumerate([*self.rest[side],grip],1):
                yield from drive._prepare_center_motor_confirmed(motor,float(q),CURRENT[motor-1] if motor<8 else GRIPPER_CURRENT)
                self.enabled_drives.add((side,motor))
            for motor in range(1,8):
                drive.send_control(parameter(motor,0x7017,RIGHT_PLAYBACK_SPEED));yield

    def positions_for(self,side):return np.array([self.states[side,i][0] for i in range(1,8)])

    def command_arm(self,side,target,*,hold=False):
        q=np.asarray(target,float)
        if getattr(self,'path_session',None) is not None:
            raise RuntimeError('Stop the generated writer before sending GUI arm targets')
        if getattr(self,'emergency_latched',False) or not self.active or q.shape!=(7,) or not np.isfinite(q).all():raise RuntimeError('Invalid generated arm command')
        drive=self if side=='right' else self.left_drive
        special=7 if side=='right' else 6
        session=getattr(self,f'{side}_joint{special}_session',None)
        for i,value in enumerate(q,1):
            # During interrupted arming never enable previously disabled motors.
            if self.states.get((side,i),(0,0,0))[1]!=2:continue
            if i==special and session is not None:
                if hold:
                    if not session.path_hold.is_set():session.emergency_hold()
                else:session.path_goal(float(value))
            else:drive.send_control(parameter(i,0x7016,joint_to_motor(side,i,float(value))))
        self.target[side]=q.copy()

    def sync_worker_holds(self,targets):
        result={s:q.copy() for s,q in targets.items()}
        for side,motor in (('right',7),('left',6)):
            worker=getattr(self,f'{side}_joint{motor}_session',None)
            if worker is not None:
                if worker.hold_position is None:return None
                if worker.status.error:raise RuntimeError(side+' strike worker fault; powered hold unconfirmed')
                result[side][motor-1]=worker.hold_position
        return result

    def recover_return_workers(self):
        """Explicit Return only: replace exited MIT owners, never re-arm/strike.

        A still-live faulted writer must finish exiting before any replacement.
        Healthy held owners are reused. Freshness and drive faults stay enforced.
        """
        from .generated_worker_hold import RecoverySession
        if not self.fresh():return False
        ready=True
        for side,motor in (('right',7),('left',6)):
            attr=f'{side}_joint{motor}_session'; worker=getattr(self,attr,None)
            if worker is None:continue
            status=worker.status
            if status.error or not worker.process.is_alive():
                if worker.process.is_alive():return False
                attempted=getattr(self,'return_recovery_attempted',set())
                if side in attempted:
                    raise RuntimeError(f'{side} recovery Hold could not start: {status.error}; keep Hold and retry Return after correcting feedback')
                if self.states[side,motor][1]!=2:
                    raise RuntimeError(f'{side} J{motor} is not powered; no automatic enable during Return')
                worker.stop()  # Already exited; releases IPC ownership, NOT torque.
                geometry=self.stream_geometry[side]
                worker=RecoverySession(self,side,geometry.lower[motor-1],geometry.upper[motor-1])
                self.return_recovery_attempted=attempted|{side}
                print(f'RETURN: replaced exited {side} J{motor} strike writer with powered Hold/return only; no enable or mode change.',flush=True)
                ready=False
            elif worker.hold_position is None:ready=False
        return ready

    def set_positions(self,joints):self.command_arm('right',joints)
    def set_left_positions(self,joints):self.command_arm('left',joints)

    def snare_center_evidence(self):
        # Compatibility name: at the GENERATED LEFT GOAL, not LEFT_CENTER.
        goal=self.handoff_goal
        if goal is None or not self.settled({'left':goal}):raise RuntimeError('Left generated goal has not settled for snare handoff')
        return dict(at=time.monotonic(),generated_goal=True)

    def guard_left_disable(self,frame):
        from safe_zone.encoder import FRAME
        self._guard_stream_write('left',frame)
        cid,_,_=FRAME.unpack(frame)
        if (cid>>24)&31==4 and self.states.get(('left',cid&255),(0,0,0))[1]!=0 and not self.release_permitted:
            raise RuntimeError('No off-rest left disable through GUI transport')

    def poll(self):
        # Parent checks fresh states/drive faults/temperatures and feeds workers;
        # its fixed-center monitors are deliberately not installed in this mode.
        self._inside_poll=True
        try:receive_without_query_burst(self,super().poll)
        except Exception:
            self.feedback_failed_at=time.monotonic()
            # Queries must continue after a receive/freshness fault so Return
            # can recover. Keep the ORIGINAL receive error if TX also fails.
            try:self.feedback_queries.poll(self)
            except Exception:pass
            raise
        finally:self._inside_poll=False
        if (self.feedback_failed_at is not None and len(self.states)==16
                and all(v[2]>self.feedback_failed_at for v in self.states.values())):
            self.feedback_failed_at=None
        # Recent per-joint replies suppress duplicate queries automatically.
        # Do not wait 100 ms for an all-or-nothing worker handoff: that leaves
        # less than one GUI callback before the unchanged 120 ms guard.
        # TX failures propagate as TX errors, not a false lost-RX latch.
        self.feedback_queries.poll(self)
        if self.fresh():
            now=time.monotonic();self.generated_history.append((now,{s:self.positions_for(s) for s in self.rest}))
            while self.generated_history and now-self.generated_history[0][0]>.65:self.generated_history.popleft()

    def fresh(self):
        return super().fresh() and (getattr(self,'_inside_poll',False) or getattr(self,'feedback_failed_at',None) is None)

    def settled(self,targets):
        h=self.generated_history;now=time.monotonic()
        if not self.fresh() or len(h)<2 or now-h[-1][0]>.1 or h[-1][0]-h[0][0]<.6:return False
        for side,goal in targets.items():
            values=np.array([v[side] for _,v in h])
            if np.max(abs(values-goal))>math.radians(.2) or np.max(np.ptp(values,axis=0))>math.radians(.12):return False
        return True

    def release_rest(self):
        if not self.settled(self.rest):raise RuntimeError('Captured rest not stationary; NOT relaxed')
        for side,motor in (('right',7),('left',6)):
            session=getattr(self,f'{side}_joint{motor}_session',None)
            if session is not None:session.stop()
        # Joining can age feedback: caller keeps polling and retries; never use
        # a stale rest proof to release the arm after worker shutdown.
        if not self.settled(self.rest):raise RuntimeError('Refresh settled rest after worker join')
        self.release_permitted=True
        try:
            for side,drive in (('right',self),('left',self.left_drive)):
                for i in range(1,9):drive.send_control(packet(4,i))
        finally:self.release_permitted=False
        self.active=self.left_drive.active=False

    def relax(self):
        self.release_rest()

    def emergency_release(self):
        """Explicit operator torque-off, at ANY pose, even with stale feedback.

        Reuse the established bounded emergency writer shutdown. No planner,
        stationary check, zone, joint bound or other peripheral can veto this.
        Attempt all drives even if a writer/individual CAN write reports failure.
        """
        from .emergency import stop_writer
        self.emergency_latched=True
        failures=[];sessions=[]
        for sock in self.sockets.values():sock.emergency_latched=True
        path_session=getattr(self,'path_session',None)
        if path_session is not None:sessions.append(path_session)
        for side,motor in (('right',7),('left',6)):
            session=getattr(self,f'{side}_joint{motor}_session',None)
            if session is not None and all(session is not old for old in sessions):sessions.append(session)
        for session in sessions:
            try:
                event=getattr(session,'stop_event',None)
                (session.cancel if event is None else event).set()
            except Exception as exc:failures.append('writer stop request: '+str(exc))
        for session in sessions:
            try:stop_writer(session)
            except Exception as exc:failures.append(str(exc))
        if path_session is not None:
            try:
                if not path_session.process.is_alive():path_session.detach()
            except Exception as exc:failures.append('path writer handoff: '+str(exc))
        # Direct disable-only packets deliberately bypass command ownership and
        # the normal return-before-relax guards. Never send enable/mode/targets.
        delivered=set();last_errors={}
        for _ in range(3):
            for side in ('right','left'):
                for motor in range(1,9):
                    try:
                        if self.sockets[side].send(packet(4,motor))!=16:raise RuntimeError('Short CAN disable write')
                        delivered.add((side,motor))
                    except Exception as exc:last_errors[side,motor]=str(exc)
        for key in set((s,i) for s in ('right','left') for i in range(1,9))-delivered:
            failures.append(f'{key[0]} motor {key[1]} disable: '+last_errors.get(key,'not sent'))
        self.active=self.left_drive.active=bool(failures)
        if failures:raise RuntimeError('Disable unconfirmed; use physical power cutoff if needed: '+'; '.join(failures))

    def close(self):
        session=getattr(self,'path_session',None)
        if session is not None:session.stop();session.detach()
        return super().close()
