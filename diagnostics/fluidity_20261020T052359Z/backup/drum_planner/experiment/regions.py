"""Sampled FCL C-space IRIS regions + genuine Drake GCS shortest path.

The samples come from FULL robot collision queries. Neither a TCP workspace box
nor a demonstration is substituted for configuration space. A finite point
obstacle approximation does NOT certify a whole region; final FCL rechecking is
mandatory and failures are reported, never silently replaced by an RRT result.
"""
import time
import numpy as np
from .algorithms import resample,dense_check
from ..common import atomic_json


def generate(oracle,seeds,req,params,deadline,log,folder):
    from pydrake.geometry.optimization import Iris,IrisOptions,HPolyhedron,Point,GraphOfConvexSetsOptions
    from pydrake.planning import GcsTrajectoryOptimization
    from pydrake.common import RandomGenerator
    from pydrake.solvers import ClarabelSolver,SolverOptions
    generator=RandomGenerator(params['seed'])
    lower=np.asarray(oracle.lower);upper=np.asarray(oracle.upper);regions=[];stats=[];domains=[];witnesses=[];region_centers=[]
    centers=[];sequences=[];original_indices=[]
    for path in seeds:
        length=np.linalg.norm(np.diff(path,axis=0),axis=1).sum()
        sequence=[]
        for q in resample(path,min(24,max(9,int(np.ceil(length/.20))+1))):
            index=next((i for i,p in enumerate(centers) if np.linalg.norm(q-p)<1e-10),len(centers))
            if index==len(centers):centers.append(q)
            sequence.append(index)
        sequences.append(sequence)
    centers=np.asarray(centers)
    options=IrisOptions();options.iteration_limit=3;options.require_sample_point_is_contained=True
    options.termination_threshold=.05;options.random_seed=params['seed']
    def inflate(obstacles,center,domain):
        # IRIS on point witnesses is well-conditioned compared with micron-sized
        # ellipsoids. Conservatively pull each resulting face 3% toward the free
        # seed to avoid optimizing exactly onto sampled collision boundaries.
        region=Iris(obstacles,center,domain,options)
        A=region.A();b=region.b();slack=np.maximum(0.,b-A@center)
        return HPolyhedron(A,b-.03*slack)
    for k,center in enumerate(centers):
        if time.monotonic()>deadline-5:break
        # Local domains avoid unnecessary high-dimensional obstacle clouds and
        # make memory scale with <=48 regions, not camera pixels x samples.
        lo=np.maximum(lower,center-.55);hi=np.minimum(upper,center+.55)
        domain=HPolyhedron.MakeBox(lo,hi);region=domain;obstacles=[];last=center
        for iteration in range(3):
            if time.monotonic()>deadline-5:break
            sample=[]
            for _ in range(96):
                last=region.UniformSample(generator,last,mixing_steps=8);sample.append(last)
            q=np.array(sample);valid=np.asarray(oracle.check(q)['valid'])
            bad=q[~valid]
            if len(bad):
                # Move collision witnesses toward the seed by bisection to find
                # more informative separating planes than far random samples.
                a=np.tile(center,(len(bad),1));b=bad.copy()
                for _ in range(6):
                    mid=(a+b)/2;good=np.array(oracle.check(mid)['valid']);a[good]=mid[good];b[~good]=mid[~good]
                obstacles.extend(Point(p) for p in b)
                try:
                    region=inflate(obstacles,center,domain)
                except RuntimeError:
                    log(f'IRIS numerical failure in region {k+1}: region omitted, not treated as free')
                    region=None;break
                last=center
            else:break
        if region is None:continue
        original_indices.append(k)
        regions.append(region);domains.append(domain);witnesses.append(obstacles);region_centers.append(center);stats.append(dict(center=center.tolist(),collision_witnesses=len(obstacles),
            A=region.A().tolist(),b=region.b().tolist(),certificate=False))
        log(f'IRIS region {k+1}/{len(centers)}: {len(obstacles)} full-body collision witnesses, {region.A().shape[0]} faces')
    atomic_json(folder/'regions.json',dict(method='Drake IRIS on iteratively sampled C-space collision neighborhoods',regions=stats,
        guarantee='sampled approximation only; every returned curve requires dense FCL validation'))
    if not regions:return None,dict(reason='region_budget_exhausted',region_count=0)
    detail={}
    for refinement in range(24):
        if time.monotonic()>deadline-3:
            return None,dict(detail,reason='GCS refinement budget exhausted')
        log(f'GCS: {len(regions)} regions, convex relaxation + rounding, refinement {refinement}; '+
            f'endpoint cells {sum(r.PointInSet(np.asarray(req["start"]),1e-8) for r in regions)}/'+
            f'{sum(r.PointInSet(np.asarray(req["goal"]),1e-8) for r in regions)}')
        gcs=GcsTrajectoryOptimization(7)
        # Preserve seed-chain connectivity and add local cross-route choices.
        # An all-pairs graph makes the conic formulation grow quadratically and
        # needlessly duplicates overlapping neighborhoods. This finite sparse
        # graph is an explicit search approximation, not a global-optimality claim.
        mapping={old:new for new,old in enumerate(original_indices)};pairs=set()
        for sequence in sequences:
            for a,b in zip(sequence[:-1],sequence[1:]):
                if a in mapping and b in mapping and a!=b:
                    pairs.add(tuple(sorted((mapping[a],mapping[b]))))
        for i,center in enumerate(region_centers):
            nearest=np.argsort([np.linalg.norm(center-q) for q in region_centers])
            for j in nearest[1:4]:pairs.add(tuple(sorted((i,int(j)))))
        edges=[]
        for i,j in sorted(pairs):
            if regions[i].IntersectsWith(regions[j]):edges.extend([(i,j),(j,i)])
        log(f'GCS sparse graph: {len(regions)} regions / {len(edges)} directed edges')
        space=gcs.AddRegions(regions,edges_between_regions=edges,order=3,h_min=.1,h_max=20,name='sampled_free_regions')
        start=gcs.AddRegions([Point(np.asarray(req['start']))],order=0,h_min=0,h_max=0,name='start')
        goal=gcs.AddRegions([Point(np.asarray(req['goal']))],order=0,h_min=0,h_max=0,name='goal')
        gcs.AddEdges(start,space);gcs.AddEdges(space,goal)
        gcs.AddTimeCost(.1);gcs.AddPathLengthCost(1.);gcs.AddPathContinuityConstraints(1)
        gcs.AddVelocityBounds(-np.full(7,.6),np.full(7,.6))
        solve_options=GraphOfConvexSetsOptions();solve_options.convex_relaxation=True;solve_options.preprocessing=True
        solve_options.max_rounded_paths=4;solve_options.rounding_seed=params['seed'];solve_options.solver=ClarabelSolver()
        solver_options=SolverOptions();solver_options.SetOption(ClarabelSolver.id(),'max_iter',150)
        solve_options.solver_options=solver_options
        trajectory,result=gcs.SolvePath(start,goal,solve_options)
        detail=dict(region_count=len(regions),directed_edges=len(edges),sparse_graph=True,solver=str(result.get_solver_id().name()),
                    solver_status=str(result.get_solution_result()),success=bool(result.is_success()),refinements=refinement,
                    region_certification='sampled, not continuous',algorithm='Drake IRIS + GcsTrajectoryOptimization')
        if not result.is_success():return None,detail
        t=np.linspace(trajectory.start_time(),trajectory.end_time(),101)
        q=np.array([trajectory.value(v).ravel() for v in t]);q[0]=req['start'];q[-1]=req['goal']
        samples=resample(q,801)
        ok,proof=dense_check(oracle,samples);detail['gcs_geometric_collision_check']=proof
        if ok:return q,detail
        valid=np.array(oracle.check(samples)['valid']);bad=samples[~valid]
        if not len(bad):
            # Edge failure between sparse samples: ask a denser grid for actual
            # witnesses rather than accepting an unchecked narrow passage.
            samples=resample(q,3201);valid=np.array(oracle.check(samples)['valid']);bad=samples[~valid]
        if not len(bad):return None,dict(detail,reason='subsample_collision_rejected')
        bad=bad[np.linspace(0,len(bad)-1,min(32,len(bad))).astype(int)]
        log(f'GCS refinement: {len(bad)} new whole-robot collision witnesses; rebuilding affected IRIS regions')
        for k,region in enumerate(regions):
            included=[p for p in bad if region.PointInSet(p,1e-6)]
            if not included:continue
            # Keep all accumulated witnesses for this local domain; no unsafe
            # path is played while regions are being refined.
            witnesses[k].extend(Point(p) for p in included)
            try:regions[k]=inflate(witnesses[k],region_centers[k],domains[k])
            except RuntimeError:
                return None,dict(detail,reason='IRIS numerical failure during refinement; no path accepted')
        atomic_json(folder/f'region_refinement_{refinement}.json',dict(collisions=bad.tolist(),
            regions=[dict(A=r.A().tolist(),b=r.b().tolist(),witnesses=len(w)) for r,w in zip(regions,witnesses)]))
    return None,dict(detail,reason='sampled_regions_missed_collision_final_FCL_rejected')
