"""Experiment-only ordered endpoints, independently checked legs, exact playback.

No replanning from rest between checkpoints, no spline across stopped joins,
and no reinterpretation of the saved camera scene at a simulated arm pose.
"""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from ..common import ROOT, atomic_json, arm_side, arm_joints, digest, read_state
from ..trajectory_view import PlanDisplay
from . import destinations

MAX_CHECKPOINTS = 3
HOLD_SECONDS = 2.
SCHEMA = 'x5-experiment-checkpoints-v2'
RETURN_POLICY = 'independently planned final checkpoint to original start'


def signature(bindings):
    return hashlib.sha256(json.dumps([b['destination_sha256'] for b in bindings],
                                     separators=(',',':')).encode()).hexdigest()


def freeze(paths, side, session):
    if not 1 <= len(paths) <= MAX_CHECKPOINTS:
        raise ValueError('Select one to three checkpoints, including the first destination')
    paths=[Path(p).resolve() for p in paths]
    if any(a==b for a,b in zip(paths,paths[1:])):
        raise ValueError('Consecutive checkpoints must be different destinations')
    bindings=[destinations.freeze(p,side,session) for p in paths]
    # Keep the original one-destination binding/result/cache contract intact.
    result=dict(bindings[-1])
    if len(bindings)>1:
        result.update(checkpoints=bindings,checkpoint_sha256=signature(bindings))
    return result


def derived_state(original, side, start, source_sha):
    state={k:copy.deepcopy(original[k]) for k in (
        'schema','coordinate_convention','right_rad','left_rad',
        'right_gripper_closed_motor_joint_rad','left_gripper_opening_m')}
    state[side+'_rad']=list(start)
    state.update(preview_only=True,source='Simulated checkpoint; NOT a fresh measurement',
                 snapshot_state_sha256=source_sha,motor_commands=0)
    return state


def plan_sequence(context, folder, method, params, seconds, plan_leg):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    side=arm_side(context.get('moving_side','right'));bindings=context['checkpoints']
    if not 2 <= len(bindings) <= MAX_CHECKPOINTS:
        raise ValueError('Checkpoint sequence requires two or three destinations')
    if context.get('checkpoint_sha256')!=signature(bindings):
        raise ValueError('Checkpoint selection identity changed')
    # Validate every frozen endpoint before starting potentially expensive work.
    endpoints=[destinations.goal(dict(b,moving_side=side))[1] for b in bindings]
    original=read_state(context['state']);start=original[side+'_rad'];legs=[];paths=[]
    # An endpoint-only snapshot, not a captured destination or a recording route.
    # Keeping it private to this result leaves the user's destination lists alone.
    return_goal=folder/'original_start_destination.json'
    atomic_json(return_goal,dict(schema=f'openarmx-{side}-motion-recording-v1',
        arm=side,joint_order=arm_joints(side),joint_position_unit='rad',
        model_sha256=digest(ROOT/'model/openarmx.urdf'),
        source='Original saved start; endpoint only; no live capture',
        snapshot_state_sha256=context['state_sha256'],
        samples=[dict(time_s=0.,positions_rad=original[side+'_rad'])]))
    return_binding=dict(goal_recording=str(return_goal.resolve()),
                        destination_source=str(return_goal.resolve()),destination_sha256=digest(return_goal))
    targets=list(zip(bindings,endpoints))+[(return_binding,original[side+'_rad'])]
    for i,(binding,endpoint) in enumerate(targets):
        returning=i==len(bindings)
        target=folder/('return_to_start' if returning else f'checkpoint_{i+1}')
        target.mkdir(parents=True,exist_ok=True)
        local={k:copy.deepcopy(v) for k,v in context.items()
               if k not in ('checkpoints','checkpoint_sha256')}
        local.update(binding)
        if i:
            state_path=target/'simulated_start.json'
            atomic_json(state_path,derived_state(original,side,start,context['state_sha256']))
            local.update(state=str(state_path.resolve()),state_sha256=digest(state_path),
                         scene_robot_start=original[side+'_rad'])
        atomic_json(target/'context.json',local)
        print(f'[checkpoints {method}] LEG {i+1}/{len(targets)}: '
              f'{"saved rest" if i==0 else Path(bindings[i-1]["destination_source"]).stem} → '
              f'{"original saved rest (new return path)" if returning else Path(binding["destination_source"]).stem}; '
              f'budget {seconds:g}s; frozen scene unchanged',flush=True)
        leg=plan_leg(local,target,method,params,seconds);legs.append(leg);paths.append(target/'plan.json')
        if leg['status']!='planned':
            # Never quietly skip a blocked checkpoint or present a partial route
            # as the whole sequence. Normal UI retry/method selection still works.
            result=copy.deepcopy(legs[0])
            for key in ('outward','cycle','timing','source_timing','path_waypoints','original_path_waypoints',
                        'duration_outward_s','final_curve_check','sampled_interpolated_checks'):
                result.pop(key,None)
            result.update(status='no_feasible_route',recording_sha256=bindings[-1]['destination_sha256'],
                          checkpoint_sha256=context['checkpoint_sha256'],
                          details=dict(reason=('Return from final checkpoint to original start has no feasible route'
                                               if returning else f'Checkpoint {i+1}/{len(bindings)} has no feasible route'),
                                       failed_checkpoint=None if returning else i+1,
                                       failed_leg='return_to_start' if returning else f'checkpoint_{i+1}',
                                       leg_details=leg.get('details',{})))
            result['experiment']['report']={'candidates':[],'selected':None}
            result['experiment'].pop('retiming',None);result['experiment'].pop('cycle_test',None)
            break
        if not np.array_equal(leg['outward'][0]['positions_rad'],start):
            raise ValueError('Checkpoint leg did not start at the previous endpoint')
        if not np.array_equal(leg['outward'][-1]['positions_rad'],endpoint):
            raise ValueError('Checkpoint leg missed its exact selected endpoint')
        start=endpoint
    else:
        result=combine(legs[:-1],bindings,paths[:-1],context,legs[-1],paths[-1])
    first_request=json.loads((paths[0].parent/'backend_request.json').read_text())
    first_request['goal']=endpoints[-1]
    atomic_json(folder/'backend_request.json',first_request)
    atomic_json(folder/'plan.json',result)
    return result


def combine(legs, bindings, paths, context, return_plan, return_path):
    from ..timing.core import Evaluator
    from ..workflow import simulate
    outward=[];segments=[];offset=0.
    for i,(leg,binding,path) in enumerate(zip(legs,bindings,paths)):
        evaluator=Evaluator(leg['timing'])
        duration=evaluator.duration
        q,v,a,_=evaluator(np.array([0.,duration]))
        if np.max(abs(v))>1e-8 or np.max(abs(a))>1e-7:
            raise ValueError('Checkpoints require rest-to-rest legs')
        if outward and not np.array_equal(outward[-1]['positions_rad'],leg['outward'][0]['positions_rad']):
            raise ValueError('Discontinuous checkpoint join')
        for j,point in enumerate(leg['outward']):
            if i and j==0:continue  # preceding hold endpoint owns this timestamp
            shifted=dict(point,time_s=offset+point['time_s'])
            # Two distinct local samples can round to the same absolute float
            # after adding a leg offset. These are display samples only; the
            # exact analytic law below is unchanged (including its endpoints).
            if outward and shifted['time_s']==outward[-1]['time_s']:outward[-1]=shifted
            else:outward.append(shifted)
        segments.append(dict(index=i+1,name=Path(binding['destination_source']).stem,
            destination_source=binding['destination_source'],destination_sha256=binding['destination_sha256'],
            start_time_s=offset,arrival_time_s=offset+duration,hold_s=HOLD_SECONDS,
            start=leg['outward'][0]['positions_rad'],goal=leg['outward'][-1]['positions_rad'],
            timing=leg['timing'],plan=str(path.resolve()),plan_sha256=digest(path),
            collision_zone_check=leg['final_curve_check']))
        offset+=duration
        if i<len(legs)-1:
            offset+=HOLD_SECONDS
            outward.append(dict(time_s=offset,positions_rad=leg['outward'][-1]['positions_rad'],
                                velocities_rad_s=[0.]*7,accelerations_rad_s2=[0.]*7))
    return_evaluator=Evaluator(return_plan['timing'])
    q,v,a,_=return_evaluator(np.array([0.,return_evaluator.duration]))
    if (not np.array_equal(return_plan['outward'][0]['positions_rad'],outward[-1]['positions_rad']) or
        not np.array_equal(return_plan['outward'][-1]['positions_rad'],outward[0]['positions_rad']) or
        np.max(abs(v))>1e-8 or np.max(abs(a))>1e-7):
        raise ValueError('Return must connect the final checkpoint to original rest, rest-to-rest')
    return_start=offset+HOLD_SECONDS
    return_leg=dict(start_time_s=return_start,arrival_time_s=return_start+return_evaluator.duration,
        start=return_plan['outward'][0]['positions_rad'],goal=return_plan['outward'][-1]['positions_rad'],
        timing=return_plan['timing'],plan=str(return_path.resolve()),plan_sha256=digest(return_path),
        collision_zone_check=return_plan['final_curve_check'])
    full_cycle=[dict(p,phase='outward') for p in outward]
    full_cycle.append(dict(time_s=return_start,positions_rad=return_leg['start'],
                           velocities_rad_s=[0.]*7,accelerations_rad_s2=[0.]*7,phase='hold_complete'))
    for point in return_plan['outward'][1:]:
        shifted=dict(point,time_s=return_start+point['time_s'],phase='return')
        # A sub-ULP sample at a join is not an additional moment in time.
        if shifted['time_s']==full_cycle[-1]['time_s']:
            if full_cycle[-1]['phase']!='hold_complete':full_cycle[-1]=shifted
        else:full_cycle.append(shifted)
    all_legs=legs+[return_plan]
    result=copy.deepcopy(legs[0])
    for key in ('timing','source_timing','path_waypoints','original_path_waypoints',
                'reverse_checked_on_same_static_scene'):
        result.pop(key,None)
    result.update(recording_sha256=bindings[-1]['destination_sha256'],
        checkpoint_sha256=context['checkpoint_sha256'],outward=outward,cycle=full_cycle,
        duration_outward_s=offset,hold_at_goal_s=HOLD_SECONDS,
        checkpoint_sequence=dict(schema=SCHEMA,segments=segments,hold_at_each_checkpoint_s=HOLD_SECONDS,
                                 return_policy=RETURN_POLICY,return_leg=return_leg,
                                 movement_outward_s=sum(s['timing']['duration_s'] for s in segments)),
        final_curve_check=dict(valid=True,legs=[leg['final_curve_check'] for leg in all_legs]),
        sampled_interpolated_checks=sum(leg['sampled_interpolated_checks'] for leg in all_legs))
    result['experiment'].update(report={'candidates':[],'selected':None},
        goal_source='Ordered checkpoint endpoints only; each leg starts at the preceding endpoint',
        destination_sha256=bindings[-1]['destination_sha256'],
        elapsed_s=sum(leg['experiment'].get('elapsed_s',0.) for leg in all_legs),
        checkpoint_count=len(legs),budget_per_leg_s=legs[0]['experiment'].get('budget_s'),
        retiming={'label':'Per-leg adaptive timing; 2 s at every checkpoint'},
        checkpoint_reports=[leg['experiment'] for leg in legs],return_report=return_plan['experiment'])
    result['experiment']['cycle_test']=simulate(result)
    print(f'[checkpoints] READY: {len(legs)} destinations; {offset:.3f}s to final checkpoint '
          f'(includes intermediate holds) → 2s hold → newly planned {return_evaluator.duration:.3f}s '
          'return to original start; SIMULATION ONLY',flush=True)
    return result


class ExperimentPlanDisplay(PlanDisplay):
    """Evaluate each checked analytic leg; never linearly approximate its curve."""
    def __init__(self,path):
        super().__init__(path)
        self.segments=[]
        sequence=self.obj.get('checkpoint_sequence') if self.playable else None
        if sequence:
            from ..timing.core import Evaluator
            if sequence.get('schema')!=SCHEMA:raise ValueError('Unknown checkpoint preview schema')
            previous=self.req['start'];offset=0.
            for segment in sequence['segments']:
                evaluator=Evaluator(segment['timing'])
                if (not np.array_equal(segment['start'],previous) or
                    abs(segment['start_time_s']-offset)>1e-9 or
                    abs(segment['arrival_time_s']-offset-evaluator.duration)>1e-9 or
                    segment['hold_s']!=HOLD_SECONDS):
                    raise ValueError('Inconsistent checkpoint timing or join')
                q=evaluator(np.array([0.,evaluator.duration]))[0]
                if not np.allclose(q,[segment['start'],segment['goal']],atol=1e-9,rtol=0):
                    raise ValueError('Checkpoint analytic endpoints changed')
                self.segments.append((segment,evaluator));previous=segment['goal']
                offset=segment['arrival_time_s']+HOLD_SECONDS
            return_leg=sequence['return_leg'];return_evaluator=Evaluator(return_leg['timing'])
            if (sequence.get('return_policy')!=RETURN_POLICY or
                not np.array_equal(return_leg['start'],previous) or
                not np.array_equal(return_leg['goal'],self.req['start']) or
                abs(return_leg['start_time_s']-offset)>1e-9 or
                abs(return_leg['arrival_time_s']-offset-return_evaluator.duration)>1e-9 or
                not np.allclose(return_evaluator(np.array([0.,return_evaluator.duration]))[0],
                                [previous,self.req['start']],atol=1e-9,rtol=0)):
                raise ValueError('Inconsistent independently planned return')
            self.return_leg=(return_leg,return_evaluator)
            if (not 2<=len(self.segments)<=MAX_CHECKPOINTS or
                not np.array_equal(previous,self.req['goal']) or
                abs(offset-HOLD_SECONDS-self.obj['duration_outward_s'])>1e-9 or
                abs(self.duration-return_leg['arrival_time_s'])>1e-9):
                raise ValueError('Inconsistent checkpoint sequence duration/endpoints')

    def locate(self,t):
        return_leg,evaluator=self.return_leg
        if t>=return_leg['start_time_s']:
            return return_leg,evaluator,t-return_leg['start_time_s'],True
        for segment,evaluator in self.segments:
            if t<=segment['arrival_time_s']+HOLD_SECONDS:
                local=(evaluator.duration if t>=segment['arrival_time_s']
                       else t-segment['start_time_s'])
                return segment,evaluator,local,False
        raise ValueError('Checkpoint time outside cycle')

    def positions(self,t):
        if not self.segments:return super().positions(t)
        if t<=0 or t>=self.duration:return np.array(self.req['start'])
        segment,evaluator,local,_=self.locate(t)
        if local>=evaluator.duration:return np.array(segment['goal'])
        if local<=0:return np.array(segment['start'])
        return evaluator(local)[0]

    def phase(self,t):
        if not self.segments:return super().phase(t)
        if t>=self.duration:return 'RETURN COMPLETE'
        segment,evaluator,local,returning=self.locate(max(0.,t))
        if returning:return 'RETURN TO ORIGINAL START'
        action='HOLD (2 seconds)' if local>=evaluator.duration else 'OUTWARD'
        return f'{action} • checkpoint {segment["index"]}/{len(self.segments)}: {segment["name"]}'
