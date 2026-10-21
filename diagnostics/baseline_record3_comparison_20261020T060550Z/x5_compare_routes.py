import json,sys,time,copy
from pathlib import Path
import numpy as np
from scipy.interpolate import PPoly
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner.experiment.config import no_can,settings
no_can()
from drum_planner.experiment.runtime import Oracle
from drum_planner.experiment.algorithms import dense_check
from drum_planner.experiment import fluidity
from drum_planner import common,style,planning
from camera_playback.trajectory import _locally_smooth_joints,_clamped_cubic_coefficients
D=Path(Path('/tmp/x5_comparison_task').read_text().strip())
P=ROOT/'planner_runs/20261020T011905Z-baseline-ed3647/calculate-d0efbf1e/plan/result/plan.json'
p=json.loads(P.read_text());req=json.loads(P.with_name('backend_request.json').read_text());record=json.loads(common.DEFAULT_RECORDING.read_text())
assert p['baseline_hardware_session'] and not p.get('experiment_only') and p['status']=='planned'
assert common.digest(req['urdf'])==p['model']['calibrated_urdf_sha256']
assert common.digest(common.DEFAULT_RECORDING)==p['recording_sha256']
planning.scene(p['provenance']['scene'])
t=np.array([s['time_s'] for s in record['samples']]);raw=np.array([s['positions_rad'] for s in record['samples']])
fit=_locally_smooth_joints(t,raw);coeff=_clamped_cubic_coefficients(t,fit)
record_curves=[PPoly(coeff[:,:,j][:,::-1].T,t) for j in range(7)]
timing=p['timing'];planned_curves=[PPoly(c,x) for c,x in zip(timing['coefficients'],timing['polynomial_breaks_unscaled_s'])]
planned_times=np.r_[np.arange(0,p['duration_outward_s'],.002),p['duration_outward_s']]
planned=np.column_stack([c(planned_times/timing['time_scale']) for c in planned_curves]);planned[0]=req['start'];planned[-1]=req['goal']
record_times=np.linspace(0,t[-1],4001);smooth=np.column_stack([c(record_times) for c in record_curves]);smooth[0]=raw[0];smooth[-1]=raw[-1]
paths={'baseline':planned,'record3_raw':raw,'record3_playback_fit':smooth}
qpaths={name:style.sample_path(q) for name,q in paths.items()}
common.atomic_json(D/'backend_request.json',req);oracle=Oracle(D/'backend_request.json',D)
results={};fks={}
try:
 for name,q in qpaths.items():
  print('Computing',name,flush=True)
  f=oracle.call('style',paths=[q.tolist()])['rows'][0];fks[name]=f
  measured=fluidity.measure(q,f,settings())
  diff=np.abs(np.diff(q,axis=0));total=diff.sum(axis=0);net=np.abs(q[-1]-q[0]);excess=total-net
  # Dominant joint progress after removing quantization noise. PCA describes
  # geometric coordination only, not proof of a physiological synergy.
  centered=q-q.mean(axis=0);s=np.linalg.svd(centered,compute_uv=False);explained=s*s/(s@s)
  d=np.array(f['distal_clearance_estimate_m']);tip=np.array(f['tip_xyz_m']);tipc=tip[::4]
  transit=np.linalg.norm(tipc-tipc[-1],axis=1)>.10
  clearance={}
  for part,mask in [('all',np.ones(len(d),dtype=bool)),('outside_final_10cm',transit)]:
   a=d[mask]
   clearance[part]=dict(n=int(len(a)),min_m=float(a.min()),p10_m=float(np.quantile(a,.1)),median_m=float(np.median(a)),mean_m=float(a.mean()),
      fraction_below_5cm=float(np.mean(a<.05)),fraction_below_7cm=float(np.mean(a<.07)),fraction_above_9cm=float(np.mean(a>=.09)))
  validity=oracle.check(q,edges=True)
  reasons={}
  for i,good in enumerate(validity['valid']):
   if not good:
    row=oracle.check(q[i:i+1]);reason=row.get('details',{}).get('reason','unknown')
    reasons[reason]=reasons.get(reason,0)+1
  # End-effector route lengths, arc complexity and coordinate excursions.
  workspace={}
  for link in ('wrist','tip'):
   x=np.array(f[link+'_xyz_m']);workspace[link]=dict(start=x[0].tolist(),goal=x[-1].tolist(),
    min_xyz=x.min(axis=0).tolist(),max_xyz=x.max(axis=0).tolist(),travel_m=float(np.linalg.norm(np.diff(x,axis=0),axis=1).sum()),
    straight_m=float(np.linalg.norm(x[-1]-x[0])),broad_turning_deg=style.broad_turning(x))
  results[name]=dict(metrics=measured,per_joint_total_deg=np.degrees(total).tolist(),per_joint_net_deg=np.degrees(net).tolist(),
   per_joint_excess_deg=np.degrees(excess).tolist(),joint_total_deg=float(np.degrees(total.sum())),
   joint_excess_deg=float(np.degrees(excess.sum())),joint_pca_fraction=explained.tolist(),
   reversal_thresholds={str(deg):fluidity.reversals(q,np.radians(deg)) for deg in (1,2,3,5)},
   workspace=workspace,clearance=clearance,nominal_checked_states=len(q),nominal_invalid_states=int(np.count_nonzero(np.logical_not(validity['valid']))),
   invalid_reasons=reasons,last_check_details=validity.get('details'))
 for name,curves,scale,duration in [('baseline',planned_curves,timing['time_scale'],p['duration_outward_s']),('record3_playback_fit',record_curves,1.,float(t[-1]))]:
  peaks=np.array([[planning.extrema(c,k)/scale**k for c in curves] for k in (1,2,3)])
  factor=max(peaks[0].max()/.6,np.sqrt(peaks[1].max()/1.2),np.cbrt(peaks[2].max()/6.))
  results[name]['timing']=dict(duration_s=duration,peaks_rad=peaks.tolist(),uniform_scale_to_baseline_limits=float(factor),
         duration_with_identical_derivative_limits_s=float(duration*factor),
         note='Uniform scaling of this exact fitted curve, not globally time-optimal retiming. Recording cubic may have acceleration jumps into hold.')
  if name=='record3_playback_fit':results[name]['timing']['start_beat_velocity_only_duration_s']=float(duration*max(1.,peaks[0].max()/.72))
 scales=p['optimization']['normalization']
 print('normalization',scales,flush=True)
 for name in results:results[name]['baseline_score_using_actual_plan_normalizers']=style.score(results[name]['metrics']['baseline'],scales)
 metadata=dict(baseline_plan=str(P),recording=str(common.DEFAULT_RECORDING),scene=p['provenance']['scene'],
   calibration_sha256=p['calibration_sha256'],recording_sha256=common.digest(common.DEFAULT_RECORDING),plan_sha256=common.digest(P),
   model_sha256=common.digest(req['urdf']),same_endpoint=bool(np.allclose(req['goal'],raw[-1],atol=1e-12)),
   start_difference_deg=np.degrees(np.array(req['start'])-raw[0]).tolist(),
   start_joints_deg=dict(baseline=np.degrees(req['start']).tolist(),record3=np.degrees(raw[0]).tolist()),
   recorded_raw_duration_s=float(t[-1]),same_left_pose_and_frozen_self_filtered_scene=True,
   recording_fit_max_joint_deviation_deg=float(np.degrees(abs(fit-raw)).max()),
   no_motion=True,no_can=True,goal_hold_and_reverse_excluded=True,
   original_goal_interiors_used_only_for_analysis_not_planning=True)
 common.atomic_json(D/'report.json',dict(metadata=metadata,results=results));common.atomic_json(D/'features.json',fks)
 np.savez_compressed(D/'paths.npz',**qpaths)
 print(json.dumps(dict(metadata=metadata,results=results),indent=2),flush=True)
finally:oracle.close()
