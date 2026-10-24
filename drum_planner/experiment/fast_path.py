"""Rank cheap workspace proposals first; certify only a selected played curve.

The production workspace generator is deliberately not modified. Same endpoint
fits, continuous joint bounds, geometry/clearance score and hard scene rules.
Ranking is not a collision certificate: callers MUST validate the timed winner.
"""
import time
import numpy as np
from scipy.spatial import cKDTree
from .. import planning, style
from ..timing import core
from . import gesture, workspace_curve as workspace
from .fluidity import measure,reversals


def candidates(oracle, req, params, seconds, log, *, prior_weight=2.,rank=True):
    began=time.monotonic();deadline=began+seconds;cfg=workspace.OPTIONS
    attempts=[];rows=[]
    report=dict(method='workspace_curve',policy='fast-workspace-v1',pool_reused=False,
        result_cache_used=False,selected=None,candidates=[],searches=attempts,
        joint_limit_check=oracle.meta['joint_limit_check'],options=cfg)
    endpoints=oracle.check([req['start'],req['goal']])
    if len(endpoints['valid'])!=2 or not all(endpoints['valid']):
        return [],dict(report,reason='nominal_endpoint_invalid',details=endpoints)
    ends=np.array(gesture.fk(oracle,np.array([req['start'],req['goal']]))['wrist_xyz_m'])
    offsets=workspace.arc_offsets(*ends,count=cfg['proposal_count'])
    # Bounded staged search: moderate broad curves first. Stronger route
    # priors remain an on-demand second stage, not destination-specific tuning.
    proposals=[(offset,prior_weight) for offset in offsets]
    times=dict(fit_s=0.,geometry_s=0.,style_s=0.)
    for i,(offset,weight) in enumerate(proposals,1):
        if time.monotonic()>=deadline:break
        index=i if prior_weight==2. else i+len(offsets)
        t=time.monotonic()
        solver=workspace.CurveFit(oracle,req,offset,dict(cfg,prior_weight=weight),deadline)
        q,detail=solver.solve();detail.update(index=index,accepted=False,prior_weight=weight)
        attempts.append(detail);times['fit_s']+=time.monotonic()-t;t=time.monotonic()
        if q is None or detail['fit_max_m']>cfg['max_fit_m']:
            detail['reason']='cartesian_curve_fit_too_far';continue
        q[0]=req['start'];q[-1]=req['goal']
        try:points,fine,timing=planning.smooth_samples(q,False)
        except ValueError as exc:detail['reason']=str(exc);continue
        path=core.original(dict(timing=timing))
        valid=True
        for j in range(7):
            from scipy.interpolate import PPoly
            joint=PPoly(path.c[:,:,j],path.x,extrapolate=False)
            roots=joint.derivative().roots(extrapolate=False)
            values=joint(np.r_[path.x,roots[np.isfinite(roots)]])
            if values.min()<oracle.lower[j]-1e-12 or values.max()>oracle.upper[j]+1e-12:
                valid=False;break
        if not valid:detail['reason']='continuous_joint_bounds';continue
        samples=style.sample_path(fine)
        features=gesture.fk(oracle,samples);metrics=workspace.path_metrics(samples,features)
        s=np.linspace(0,1,1001)
        target=ends[0]+(ends[1]-ends[0])*s[:,None]+workspace.bezier_basis(s)@np.array(detail['control_offsets_m'])
        error=float(cKDTree(target).query(np.array(features['wrist_xyz_m']))[0].max())
        detail['played_curve_fit_max_m']=error
        if error>cfg['max_fit_m']:detail['reason']='played_curve_fit_too_far';continue
        times['geometry_s']+=time.monotonic()-t;t=time.monotonic()
        # This screen can reject an obviously blocked proposal cheaply, but
        # NEVER substitutes for the final timed dense curve validation.
        screen=oracle.check(samples[::25],stop=True)
        if len(screen['valid'])!=33 or not all(screen['valid']):
            detail['reason']='coarse_collision_or_zone';continue
        times['style_s']+=time.monotonic()-t
        rows.append(dict(index=index,path=q.tolist(),outward=points,timing=timing,_samples=samples,
            duration_s=timing['duration_s'],workspace=metrics,gesture=metrics['gesture'],
            workspace_fit=detail,source='workspace_curve',stop_at_corners=False))
    # Native padding bands rebuild collision meshes. Batch all survivors so
    # each band is built once, retaining the original 201-sample soft metric.
    # When several shapes compete, include the stronger-prior family before
    # ranking. A sole surviving moderate shape needs neither ranking nor that
    # extra family unless its final validation fails. This is task-independent.
    if not rank:
        return rows,dict(report,pool_elapsed_s=time.monotonic()-began,profile=times)
    if len(rows)>1 and prior_weight==2. and time.monotonic()<deadline:
        extra,stage=candidates(oracle,req,params,deadline-time.monotonic(),log,prior_weight=5.,rank=False)
        rows.extend(extra);attempts.extend(stage['searches']);report['expanded_alternatives']=True
        for key,value in stage['profile'].items():times[key]+=value
    t=time.monotonic()
    if len(rows)==1:
        row=rows[0]
        row['metrics']=dict(reversals=reversals(row.pop('_samples'),np.radians(2.)),
            clearance_scoring='not needed: only one screened candidate; nominal collision checks still required')
        row['score']=workspace.score(row['workspace'],0.,params)
        row['workspace_fit'].update(score=row['score'],rankable=True)
    elif rows:
        features=oracle.call('style',paths=[r['_samples'].tolist() for r in rows])['rows']
        if len(features)!=len(rows):raise ValueError('Incomplete workspace scoring reply')
        for row,feature in zip(rows,features):
            row['metrics']=measure(row.pop('_samples'),feature,
                dict(reversal_threshold_deg=2.,joint_spacing_rad=.05,spatial_spacing_m=.05))
            value=workspace.score(row['workspace'],row['metrics']['baseline']['components']['obstacle_proximity'],params)
            row['score']=value;row['workspace_fit'].update(score=value,rankable=True)
    times['style_s']+=time.monotonic()-t
    rows.sort(key=lambda r:(round(r['score'],8),r['index']))
    report.update(pool_elapsed_s=time.monotonic()-began,profile=times,
        candidates=[{k:r[k] for k in ('index','score','duration_s','workspace','metrics','gesture','source')} for r in rows])
    if not rows:report['reason']='no_workspace_proposal_within_budget'
    log(f'FAST PATH: ranked {len(rows)}/{len(attempts)} broad curves in {time.monotonic()-began:.3f}s; '
        'geometry + clearance, final timed collision check still required')
    return rows,report
