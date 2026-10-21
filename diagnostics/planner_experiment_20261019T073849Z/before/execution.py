"""Explicitly armed, supervised STATIC-SCENE execution of newly measured plans.

Not invoked by preview/planning. Commissioning evidence and conservative route
rechecking are mandatory. New plans use detected obstacles, not a visibility veto.
No demo, legacy preview-only route, auto-center, safe-zone,
motor-zero change, gain tuning or open-gripper command is accepted.
"""
from collections import deque
import copy
import json
from pathlib import Path
import select
import signal
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import PPoly

from .common import DEFAULT_CALIBRATION, DEFAULT_RECORDING, atomic_json, digest, endpoint
from . import planning

TRACK_L1=.01                 # total joint error budget, radians (not per joint)
LEFT_MAX=np.radians(.35)
REST_TOL=np.radians(.2)
SCENE_AGE=30.


class Curve:
    def __init__(self,plan):
        timing=plan['timing'];self.scale=float(timing['time_scale'])
        self.duration=float(timing['duration_s'])
        self.curves=[PPoly(np.asarray(c),np.asarray(x),extrapolate=False) for c,x in
            zip(timing['coefficients'],timing['polynomial_breaks_unscaled_s'])]
        if (len(self.curves)!=7 or not np.isfinite(self.scale) or self.scale<=0 or
                not np.isfinite(self.duration) or not 0<self.duration<=120):
            raise ValueError('Invalid exact polynomial trajectory')
        for point in plan['outward']:
            if not np.allclose(self.at(point['time_s']),point['positions_rad'],atol=1e-9,rtol=0):
                raise ValueError('Polynomial differs from collision-checked samples')
    def at(self,t):
        if not np.isfinite(t) or not 0<=t<=self.duration:raise ValueError('Trajectory time out of range')
        q=np.array([p(t/self.scale) for p in self.curves])
        if not np.isfinite(q).all():raise ValueError('Invalid trajectory target')
        return q


def commissioning(path,plan,req,cal_path):
    """Human physical checks are explicit, hash-bound evidence, never inferred."""
    review=json.loads(Path(path).read_text())
    if review.get('schema')!='x5-drum-planner-commissioning-v1':raise ValueError('Commissioning review required')
    hashes=dict(calibration_sha256=digest(cal_path),urdf_sha256=digest(req['urdf']),
                srdf_sha256=digest(req['srdf']),recording_sha256=digest(DEFAULT_RECORDING))
    if any(review.get(k)!=v for k,v in hashes.items()):raise ValueError('Commissioning review does not match current geometry/calibration/goal')
    for field in ('depth_alignment_verified','right_tool_mount_verified','left_attachment_clearance_reviewed',
                  'supported_rest_verified','static_scene_access_controlled','emergency_stop_available'):
        if review.get(field) is not True:raise ValueError('Physical commissioning incomplete: '+field)
    if not isinstance(review.get('verification_notes'),str) or len(review['verification_notes'].strip())<20:
        raise ValueError('Document physical measurements/support/left attachment review; do not merely toggle flags')
    support=np.asarray(review.get('supported_right_rad'),dtype=float)
    if support.shape!=(7,) or not np.isfinite(support).all() or np.max(abs(support-req['start']))>REST_TOL:
        raise ValueError('Current start does not match the physically verified supported rest')
    return review


def prepare(path,review_path,cal_path=DEFAULT_CALIBRATION):
    path=Path(path).resolve();obj=json.loads(path.read_text())
    req=json.loads((path.parent/'backend_request.json').read_text())
    if (obj.get('status')!='planned' or obj.get('synthetic') is not False or
            obj.get('collision_policy')!=planning.COLLISION_POLICY or
            req.get('collision_policy')!=planning.COLLISION_POLICY or
            obj.get('strict_unknown_space') is not False or req.get('occupied_only_preview') is not False or
            req.get('execute') is not False or not obj.get('actual_current_robot_state_verified') or
            not obj.get('reverse_checked_on_same_static_scene') or obj.get('hold_at_goal_s')!=2.):
        raise ValueError('Execution requires a NEW measured detected-obstacle real-scene plan; demos/legacy previews forbidden')
    if obj.get('calibration_sha256')!=digest(cal_path) or obj.get('recording_sha256')!=digest(DEFAULT_RECORDING):
        raise ValueError('Plan calibration or recording changed')
    if digest(req['urdf'])!=obj['model']['calibrated_urdf_sha256'] or digest(req['srdf'])!=obj['model']['srdf_sha256']:
        raise ValueError('Collision model changed')
    if digest(obj['provenance']['state'])!=obj['provenance']['state_sha256']:
        raise ValueError('Measured start file changed')
    state=json.loads(Path(obj['provenance']['state']).read_text())
    if not state.get('measured') or state['right_rad']!=req['start'] or state['left_rad']!=req['left']:
        raise ValueError('Plan does not use the measured start')
    world=planning.scene(obj['provenance']['scene'],cal_path)
    if world['source_scene_sha256']!=req['scene']['source_scene_sha256']:
        raise ValueError('Planning scene changed')
    review=commissioning(review_path,obj,req,cal_path)
    curve=Curve(obj)
    if (not np.allclose(curve.at(0),req['start'],atol=1e-10,rtol=0) or
            not np.allclose(curve.at(curve.duration),endpoint(),atol=1e-10,rtol=0)):
        raise ValueError('Incorrect start or record3 endpoint')
    check=copy.deepcopy(req);check['scene']=world
    check['scene']['obstacle_padding_m']=max(.05,world['obstacle_padding_m'])
    check['minimum_self_clearance_m']=.04
    # Include outward + reversed prefix: same geometry, exact polynomial, no
    # unchecked shortcuts or separate centering transition.
    times=np.r_[np.arange(0,curve.duration,.002),curve.duration]
    check['check_states']=[curve.at(t).tolist() for t in times];check['check_path']=True
    result=planning.backend(check,path.parent,'execution_clearance')
    checks=result.get('checks',[])
    if (result.get('status')!='checked' or result.get('collision_policy')!=planning.COLLISION_POLICY or
            result.get('strict_unknown_space') is not False or len(checks)!=len(times) or
            not all(p.get('valid') is True for p in checks)):
        failures=[p for p in checks if not p.get('valid')]
        raise ValueError('Execution detected-obstacle/self-clearance check failed: '+str(failures[:3] or result))
    now=time.monotonic();stamp=world['frames'][-1]['received_monotonic']
    if not 0<=now-stamp<=SCENE_AGE:raise ValueError('Scene too old for arming; acquire and plan again')
    root=ET.parse(req['urdf']).getroot()
    from .common import JOINTS
    req['execution_joint_limits']=[[float(root.find(f"joint[@name='{name}']/limit").get(key))
                                   for key in ('lower','upper')] for name in JOINTS]
    atomic_json(path.parent/'execution_commissioning.json',dict(review=review,source_sha256=digest(review_path),
                plan_sha256=digest(path),scene_sha256=world['source_scene_sha256'],
                collision_policy=planning.COLLISION_POLICY,unobserved_space_blocks_motion=False))
    return obj,req,curve,review,stamp


class Cancelled(Exception):pass


class Runner:
    """Motor ownership is not released on a fault without operator support.

    CAN loss/power loss/stalls cannot guarantee holding torque. Never command
    relax in those cases; retain ownership and ask for physical intervention.
    """
    def __init__(self,bus,obj,req,curve,folder,scene_stamp):
        self.bus=bus;self.obj=obj;self.req=req;self.curve=curve;self.folder=Path(folder)
        self.scene_stamp=scene_stamp;self.cancel=False;self.recovering=False;self.started=False
        self.tau=0.;self.anchor=curve.at(0);self.events=[];self.phase='PREFLIGHT';self.disabled=False
        self.operator_supported_exit=False

    def event(self,phase,**kw):
        self.phase=phase;item=dict(monotonic=time.monotonic(),phase=phase,**kw);self.events.append(item)
        with (self.folder/'execution_events.jsonl').open('a') as log:log.write(json.dumps(item)+'\n')
        print(phase,kw or '',flush=True)

    def stop(self,*args):self.cancel=True

    def feedback(self,enabled=True):
        self.bus.poll()
        if not self.bus.fresh():raise RuntimeError('Missing fresh feedback; no relax')
        now=time.monotonic()
        if any(now-v[2]>.12 for v in self.bus.states.values()):raise RuntimeError('Stale feedback; no relax')
        if any(self.bus.states['left',i][1]!=0 for i in range(1,9)):
            raise RuntimeError('Left arm must remain disabled')
        left=np.array([self.bus.states['left',i][0] for i in range(1,8)])
        if np.max(abs(left-self.req['left']))>LEFT_MAX:raise RuntimeError('Static left arm moved; no checked return')
        opening=np.clip(.044*self.bus.states['left',8][0]/1.0472,0,.044)
        if abs(opening-self.req['left_opening_m'])>.001:raise RuntimeError('Static left gripper changed')
        if enabled and any(self.bus.states['right',i][1]!=2 for i in range(1,9)):
            raise RuntimeError('A right motor stopped; operator required')
        if not enabled and any(v[1]!=0 for v in self.bus.states.values()):
            raise RuntimeError('Preflight requires all motors already disabled')
        from centering.motors import RIGHT_GRIPPER_CLOSED
        if abs(self.bus.states['right',8][0]-RIGHT_GRIPPER_CLOSED)>np.radians(1.):
            raise RuntimeError('Gripper is not near custom CLOSED; do not force rubber-band mechanism')
        q=np.array([self.bus.states['right',i][0] for i in range(1,8)])
        bounds=np.asarray(self.req['execution_joint_limits'])
        if not np.isfinite(q).all() or np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):
            raise RuntimeError('Measured right joints outside URDF limits; no clipping or recovery guess')
        return q

    def command(self,q):
        actual=self.feedback()
        if np.sum(abs(actual-self.anchor))>TRACK_L1:
            raise RuntimeError('Tracking deviation/stall exceeds checked clearance budget; no relax')
        if np.sum(abs(np.asarray(q)-self.anchor))>.025:
            raise RuntimeError('Target discontinuity rejected')
        self.bus.set_positions(q);self.anchor=np.asarray(q).copy()

    def settle(self,goal,seconds=.6):
        history=deque();began=time.monotonic()
        while True:
            self.command(goal);now=time.monotonic()
            q=np.array([self.bus.states['right',i][0] for i in range(1,8)])
            history.append((now,q))
            while history and now-history[0][0]>seconds+.025:history.popleft()
            values=np.array([p for _,p in history])
            if now-history[0][0]>=seconds and np.max(abs(values-goal))<=REST_TOL and np.max(np.ptp(values,axis=0))<=np.radians(.12):
                return
            if now-began>8:raise RuntimeError('Endpoint did not settle; no relax')
            time.sleep(.01)

    def stream(self,duration,parameter,*,returning=False):
        began=time.monotonic();previous=began
        while True:
            now=time.monotonic()
            if now-previous>.08:raise RuntimeError('Scheduler missed 80 ms; retain last hold target')
            if self.cancel and not returning:raise Cancelled('Controlled return requested')
            elapsed=min(duration,now-began);tau=float(parameter(elapsed))
            self.command(self.curve.at(tau));self.tau=tau;previous=now
            if elapsed>=duration:break
            time.sleep(max(0,.01-(time.monotonic()-now)))

    def return_to_start(self):
        self.event('RETURN_TO_SAVED_START');self.recovering=True
        # Stop at last commanded on-path point, then retrace that exact prefix.
        # Quintic scalar time warp starts/ends at rest, with no geometry changes.
        self.settle(self.anchor)
        tau=self.tau
        if tau>0:
            duration=max(3.,3.*tau)
            def parameter(t):
                s=t/duration;blend=10*s**3-15*s**4+6*s**5
                return float(np.clip(tau*(1-blend),0,tau))
            self.stream(duration,parameter,returning=True)
        self.settle(self.curve.at(0))
        self.event('VERIFIED_SUPPORTED_START')
        # Support was physically commissioned for precisely this start pose.
        # This is not an inference of contact from stationary joint readings.
        self.feedback();self.bus.release_at_verified_rest(self.req['start']);self.disabled=True
        self.event('RELAXED_AT_SAVED_START')

    def fault_wait(self,error):
        try:self.event('FAULT_NO_RELAX_OPERATOR_REQUIRED',error=str(error))
        except Exception:pass  # logging failure must not release torque ownership
        try:
            print('No disable/relax sent. Do not assume torque survives a motor/CAN/power fault. '
                  'Physically support the arm, use the hardware stop if needed, then type SUPPORTED EXIT. '
                  'Ownership stays locked until acknowledgement; Ctrl-C does NOT release it.',flush=True)
        except Exception:pass
        while True:
            # Do not invent a path or repeatedly drive a stalled motor. Firmware
            # retains its last CSP setpoint; no further motion targets are sent.
            try:self.bus.poll()
            except Exception:pass
            try:
                if sys.stdin.isatty() and select.select([sys.stdin],[],[],.1)[0]:
                    if sys.stdin.readline().strip()=='SUPPORTED EXIT':
                        self.operator_supported_exit=True;break
                else:time.sleep(.1)
            except Exception:time.sleep(.1)  # broken terminal never authorizes releasing ownership
        try:self.event('OPERATOR_SUPPORTED_EXIT_NO_RELAX')
        except Exception:pass

    def run(self):
        old={sig:signal.signal(sig,self.stop) for sig in (signal.SIGINT,signal.SIGTERM)}
        error=None
        try:
            self.event('WAITING_FOR_DISABLED_FEEDBACK');began=time.monotonic()
            while not self.bus.fresh():
                self.bus.poll()
                if time.monotonic()-began>2:raise RuntimeError('No fresh initial encoders')
                time.sleep(.005)
            if np.sum(abs(self.feedback(enabled=False)-self.req['start']))>TRACK_L1:
                raise RuntimeError('Arm moved since capture; replan before enabling')
            if not 0<=time.monotonic()-self.scene_stamp<=SCENE_AGE:
                raise RuntimeError('Scene expired before arming')
            if self.cancel:raise Cancelled('Cancelled before arming')
            self.event('ARMING_AT_CURRENT_POSE');self.started=True
            self.bus.arm_at_start(self.req['start'])
            self.settle(self.curve.at(0))
            self.event('OUTWARD')
            self.stream(self.curve.duration,lambda t:t)
            self.settle(self.curve.at(self.curve.duration))
            self.event('HOLD_AT_GOAL_2_SECONDS')
            self.stream(2.,lambda t:self.curve.duration)
            self.return_to_start()
        except BaseException as exc:
            error=str(exc) or type(exc).__name__
            if not getattr(self.bus,'enable_attempted',self.started):self.started=False
            if self.started and not self.disabled:
                # Only a user cancellation is automatically recoverable. A
                # tracking/stall/CAN/static-scene error invalidates the tube.
                if isinstance(exc,Cancelled):
                    try:self.return_to_start()
                    except BaseException as recovery:self.fault_wait(recovery)
                else:self.fault_wait(exc)
            if not self.started:raise
        finally:
            for sig,handler in old.items():signal.signal(sig,handler)
            atomic_json(self.folder/'execution_report.json',dict(error=error,completed=self.disabled,
                relaxed_at_verified_start=self.disabled,events=self.events,
                static_scene_only=True,operator_supported_exit=self.operator_supported_exit))


def execute(path,review_path,cal_path=DEFAULT_CALIBRATION):
    if not sys.stdin.isatty():raise ValueError('Execution requires an attended interactive terminal')
    obj,req,curve,review,stamp=prepare(path,review_path,cal_path)
    print('PHYSICAL RIGHT-ARM MOTION: current pose → record3 → hold 2 s → saved supported start → relax.\n'
          'Collision checking uses detected obstacles; unseen obstacles are not accounted for.\n'
          'Left stays disabled. Keep the static scene clear; have the hardware stop available.\n'
          'Type MOVE RIGHT ARM to authorize this freshly checked cycle:',flush=True)
    if input().strip()!='MOVE RIGHT ARM':raise ValueError('Motion not authorized')
    if not 0<=time.monotonic()-stamp<=SCENE_AGE:raise ValueError('Scene expired during confirmation; recapture')
    from .motor_transport import PlannerMotors
    folder=Path(path).resolve().parent
    bus=PlannerMotors(folder,req['start'])
    try:
        runner=Runner(bus,obj,req,curve,folder,stamp);runner.run()
        return runner.disabled
    finally:bus.close()  # descriptors ONLY; never disables or relaxes
