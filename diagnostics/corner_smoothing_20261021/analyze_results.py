"""Audit production results and compare smoothness, using no devices.
Run with LEFT_RESULTS_JSON RIGHT_RESULTS_JSON; emits compact retained laws and
proofs beside this script. Recording samples are only read after planning.
"""
import json,sys
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.experiment.config import no_can
from drum_planner.experiment.runtime import Oracle
from drum_planner.timing import core,rounding,benchmark,motion_quality

HERE=Path(__file__).resolve().parent


def workspace_metrics(oracle,t,q):
    fk=oracle.call('fk',q=q.tolist());result={}
    for point in ('wrist','tip'):
        x=np.asarray(fk[point+'_xyz_m']);v=np.gradient(x,t,axis=0,edge_order=2)
        m=motion_quality.measure(t,x,v)
        m['peak_component_speed_m_s']=m.pop('peak_joint_speed_rad_s')
        m['cruise_speed_m_s']=m.pop('cruise_joint_norm_rad_s');m['path_length_m']=m.pop('path_length_rad')
        m['definition']=m['definition'].replace('joint-speed norm','Cartesian speed norm')
        m['endpoint_exclusion']=m['endpoint_exclusion'].replace('joint arc','workspace arc')
        result[point]=m
    return result


def run(left,right):
    no_can();rows={};curves={};resources=[]
    for side,results in [('left',left),('right',right)]:
        for method,spec in json.loads(Path(results).read_text()).items():
            case=side+'_'+method;planfile=Path(spec['plan']);obj=json.loads(planfile.read_text())
            assert spec['status']==obj['status']=='planned'
            old=json.loads((HERE/'inputs'/f'{case}.json').read_text())
            source=dict(old);source['request']=benchmark.benchmark_request(source)
            before=old['v1_law'];after=obj['timing'];meta=obj['experiment']['retiming']
            assert obj['source_timing']==old['timing'],case
            assert obj['motor_commands']==0 and not obj['hardware_execution_available']
            if side=='right':
                assert obj['path_waypoints']==old['path_waypoints'],case
                for key in ['kind','duration_s','path','density','curve']:
                    if key in before:assert after[key]==before[key],(case,key)
                assert meta['geometry_preserved'] and after['method']=='adaptive'
            else:
                assert obj['original_path_waypoints']==old['path_waypoints']
                assert meta['smoothing_applied'] and after['method']=='adaptive_c3_v2'
                source=rounding.as_source(core.decode(after['path']),source)
                source['model']=old['model']
                assert core.identity(source)==after['source_path_sha256']
                assert meta['after']['internal_stops']==meta['after']['deep_slowdowns']==0
            target=HERE/'selected'/case;target.mkdir(parents=True,exist_ok=True)
            atomic_json(target/'law.json',after);atomic_json(target/'source.json',source)
            # Independently recheck actual exported coefficients, not just the
            # cached verification produced by the planning worker.
            proof=benchmark.validate(after,source,ROOT/'planner_runs/corner_export_audit'/case)
            atomic_json(target/'verification.json',proof)
            row=dict(duration_before_s=before['duration_s'],duration_after_s=after['duration_s'],
                before=motion_quality.law_metrics(before),after=motion_quality.law_metrics(after),
                v1_right_bit_identical=side=='right',meta=meta,
                law=str((target/'law.json').relative_to(ROOT)),source=str((target/'source.json').relative_to(ROOT)),
                actual_exported_curve_revalidated=True,original_plan_sha256=digest(planfile))
            if side=='left':
                work=ROOT/'planner_runs/corner_workspace_audit'/case;work.mkdir(parents=True,exist_ok=True)
                atomic_json(work/'request.json',source['request']);oracle=Oracle(work/'request.json',work)
                try:
                    for name,law in [('before',before),('after',after)]:
                        t=np.linspace(0,law['duration_s'],5001);q,v,*_=core.evaluate(law,t)
                        row['workspace_'+name]=workspace_metrics(oracle,t,q)
                        curves[case+'_'+name]=(t/ t[-1],np.linalg.norm(v,axis=1))
                    if method=='single_arc':
                        record=json.loads((ROOT/'left_recordings/record1.json').read_text())
                        samples=record['samples'];tr=np.array([s['time_s'] for s in samples]);qr=np.array([s['positions_rad'] for s in samples])
                        grid=np.linspace(tr[0],tr[-1],round((tr[-1]-tr[0])/.005)+1);dt=grid[1]-grid[0]
                        qr=np.column_stack([np.interp(grid,tr,qr[:,j]) for j in range(7)])
                        window=int(round(.2/dt))|1;qf=savgol_filter(qr,window,3,axis=0)
                        vf=savgol_filter(qr,window,3,deriv=1,delta=dt,axis=0)
                        reference=dict(joint=motion_quality.recording_metrics(record),workspace=workspace_metrics(oracle,grid,qf),
                            different_start=True,generation_input=False,encoder_filter_window_s=.2)
                        curves['record1']=((grid-grid[0])/(grid[-1]-grid[0]),np.linalg.norm(vf,axis=1))
                finally:oracle.close()
            rows[case]=row
            job=planfile.parent/'job.json'
            if job.exists():resources.append(json.loads(job.read_text()))
    protected=json.loads((HERE/'preserved_sources.json').read_text())
    assert all(digest(ROOT/f)==sha for f,sha in protected.items())
    report=dict(rows=rows,record1=reference,protected_sources_unchanged=True,
        software_only=True,can_opened=False,camera_opened=False,resources=resources)
    atomic_json(HERE/'integration_audit.json',report)
    import matplotlib;matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    for ax,method in zip(axes,['single_arc','clearance']):
        for name,label,color in [('before','V1: corner stops','#9c4c2b'),('after','V2: bounded C3 blends','#1975b4')]:
            t,v=curves['left_'+method+'_'+name];ax.plot(t,v,label=label,color=color,lw=2)
        t,v=curves['record1'];ax.plot(t,v,label='record1 reference (different start; filtered)',color='#767676',ls='--',alpha=.7)
        ax.set(title='Left '+method.replace('_',' ').title(),ylabel='Joint-speed norm (rad/s)',xlabel='Fraction of outward motion')
        ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=9)
    fig.suptitle('Smoothing removes internal stops without raising the joint motion limits')
    fig.savefig(HERE/'fluidity_comparison.png',dpi=140);plt.close(fig)
    for name,row in rows.items():
        print(name,round(row['duration_before_s'],4),'->',round(row['duration_after_s'],4),'stops',row['before']['internal_stops'],'->',row['after']['internal_stops'])
        if name.startswith('left'):print('  tip:',row['workspace_before']['tip']['internal_stops'],'->',row['workspace_after']['tip']['internal_stops'],'stops;',row['workspace_after']['tip']['deep_slowdowns'],'deep slowdowns')
    print('ALL EXPORTED CURVES AND RIGHT-ARM IDENTITY CHECKS PASSED')


if __name__=='__main__':run(*sys.argv[1:])
