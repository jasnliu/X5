"""Common metrics on the ACTUAL retimed curves; saved inputs, no devices."""
import csv,json
from pathlib import Path
import numpy as np
from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.experiment import config,gesture,fluidity
from drum_planner.experiment.runtime import Oracle
from drum_planner.timing.core import Evaluator,LIMITS
from drum_planner import style

DOC=Path(__file__).parent

def main():
    config.no_can();rows=[];traces={};cases=json.loads((DOC/'cases.json').read_text())
    for case in cases:
        session=Path(case['session']);results=json.loads((session/'final_results.json').read_text())
        assert set(results)==set(config.METHODS)
        reference=Path(results['single_arc']['plan']).parent
        out=session/'final_metric_audit';out.mkdir(exist_ok=True)
        oracle=Oracle(reference/'backend_request.json',out)
        try:
            for method in config.METHODS:
                path=Path(results[method]['plan']);plan=json.loads(path.read_text())
                req=json.loads((path.parent/'backend_request.json').read_text())
                assert not plan['motion_authorized'] and not plan['hardware_execution_available']
                assert plan['experiment_only'] and not plan['can_access'] and plan['motor_commands']==0
                assert plan['experiment']['destination_sha256']==case['destination_sha256']
                row=dict(destination=case['id'],side=case['side'],method=method,status=plan['status'],plan=str(path))
                if plan['status']!='planned':
                    row['reason']=plan.get('details',{}).get('reason');rows.append(row);continue
                curve=Evaluator(plan['timing']);grid=np.linspace(0,curve.duration,max(801,int(np.ceil(curve.duration/.002))+1))
                q,v,a,j=curve(grid)
                assert np.allclose(q[0],req['start'],atol=1e-10) and np.allclose(q[-1],req['goal'],atol=1e-10)
                assert np.all(np.array([abs(v).max(),abs(a).max(),abs(j).max()])<=LIMITS+1e-7)
                # Timing dispatch already performs dense continuous checks.
                # Repeat geometry checks independently on the rendered law.
                from drum_planner.experiment.algorithms import dense_check
                valid,proof=dense_check(oracle,q)
                assert valid,(case['id'],method,proof)
                samples=style.sample_path(q);f=oracle.call('style',paths=[samples.tolist()])['rows'][0]
                g=gesture.geometry(samples,f);m=fluidity.measure(samples,f,dict(reversal_threshold_deg=2.,joint_spacing_rad=.05,spatial_spacing_m=.05))
                clearance=np.asarray(f['distal_clearance_estimate_m'])
                report=plan['experiment']['report']
                selected=next(r for r in report['candidates'] if r['index']==report['selected'])
                row.update(duration_s=curve.duration,generation_s=report['pool_elapsed_s'],
                    retiming_s=plan['experiment']['retiming']['elapsed_s'],
                    wrist_turn_deg=g['wrist']['broad_turning_deg'],tip_turn_deg=g['tip']['broad_turning_deg'],
                    wrist_length_m=g['wrist']['length_m'],tip_length_m=g['tip']['length_m'],
                    wrist_curvature_variation=m['spatial']['wrist']['curvature_variation'],
                    tip_curvature_variation=m['spatial']['tip']['curvature_variation'],
                    joint_reversals=g['reversal_count'],joint_extra_deg=g['joint_excess_deg'],
                    proximity_cost=m['baseline']['components']['obstacle_proximity'],
                    clearance_min_m=float(clearance.min()),clearance_p10_m=float(np.quantile(clearance,.1)),
                    clearance_median_m=float(np.median(clearance)),
                    source=selected['source'],fallback=report.get('normalization',{}).get('fallback_used',False),
                    independent_dense_check=proof,smoothing_applied=bool(plan['experiment']['retiming'].get('smoothing_applied')))
                rows.append(row)
                traces.setdefault(case['id'],{})[method]={key:np.asarray(f[key]).tolist() for key in ('wrist_xyz_m','tip_xyz_m')}
                print('AUDITED',case['id'],method,'wrist',round(row['wrist_turn_deg'],1),'tip',round(row['tip_turn_deg'],1),'seconds',round(curve.duration,3),flush=True)
        finally:oracle.close()
    atomic_json(DOC/'comparison.json',dict(cases=cases,rows=rows,all_methods_checked=True,
                no_can=True,camera_opened=False,physical_motion=False,
                clearance_note='Distal world-only 5 mm padding-band estimate saturated at 100 mm, not exact clearance',
                metrics_note='Time-independent geometry from actual analytic playback law, sampled at <=2 ms; common metric definitions for all methods'))
    atomic_json(DOC/'traces.json',traces)
    scalar=[k for k in rows[0] if k!='independent_dense_check']
    with (DOC/'comparison.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=scalar,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(12,12))
    for i,case in enumerate(cases):
        ax=fig.add_subplot(2,2,i+1,projection='3d')
        for method,color in [('single_arc','tab:blue'),('workspace_curve','tab:orange')]:
            xyz=np.array(traces[case['id']][method]['wrist_xyz_m']);tip=np.array(traces[case['id']][method]['tip_xyz_m'])
            ax.plot(*xyz.T,color=color,label=method+' gripper base')
            ax.plot(*tip.T,color=color,linestyle='--',alpha=.65,label=method+' stick tip')
            ax.scatter(*xyz[[0,-1]].T,c=color,s=15)
        ax.set_title(case['id']);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Z (m)')
        limits=np.array([ax.get_xlim3d(),ax.get_ylim3d(),ax.get_zlim3d()]);mid=limits.mean(axis=1);radius=np.ptp(limits,axis=1).max()/2
        ax.set_xlim(mid[0]-radius,mid[0]+radius);ax.set_ylim(mid[1]-radius,mid[1]+radius);ax.set_zlim(mid[2]-radius,mid[2]+radius);ax.set_box_aspect([1,1,1])
        ax.legend(fontsize=7)
    fig.suptitle('Same start, destination, scene and constraints — solid: gripper base; dashed: stick tip')
    fig.tight_layout();fig.savefig(DOC/'paths.png',dpi=130);plt.close(fig)
    print('COMMON METRIC AUDIT COMPLETE',flush=True)

if __name__=='__main__':main()
