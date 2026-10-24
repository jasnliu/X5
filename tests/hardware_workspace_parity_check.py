"""Saved-input parity: identical Workspace geometry/selection in both UIs.

Experiments now use the explicitly approved <=1% GPU timing policy; hardware
retains the CPU reference. Do not require bit-identical timing coefficients.

Uses local checked candidate/timing pools produced by the native UI test, never
hardware acquisition or physical execution authority. No camera/CAN access.
"""
import argparse,json
from pathlib import Path
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION,digest
from drum_planner.right_zone import metadata
from drum_planner.experiment import worker,config


def main():
    p=argparse.ArgumentParser();p.add_argument('--ui',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();config.no_can()
    a.output.mkdir(parents=True,exist_ok=False);rows=[]
    for session in a.ui:
        verification=json.loads((session/'verification.json').read_text())
        for case in verification['rows']:
            root=Path(case['plan']);mission=json.loads(root.read_text())
            if 'checkpoint_sequence' in mission:
                sequence=mission['checkpoint_sequence']
                sources=[Path(s['plan']) for s in sequence['segments']+[sequence['return_leg']]]
            else:sources=[root]
            for i,path in enumerate(sources):
                original=json.loads(path.read_text());req=json.loads((path.parent/'backend_request.json').read_text())
                side=req.get('moving_side','right');contextfile=path.parent/'context.json'
                if contextfile.exists():context=json.loads(contextfile.read_text())
                else:
                    context=dict(original['provenance'],moving_side=side,calibration=str(DEFAULT_CALIBRATION),
                                 pool_root=str(root.parent/'candidate_pool'),**mission['destination_bindings'][0])
                    context['scene_sha256']=digest(context['scene'])
                spec=req[side+'_zone'];context[side+'_zone']=metadata(spec);context['zone_path']=spec['zone_file']
                envelope=original.get('route_planner',original.get('experiment'))
                out=a.output/f'{side}-{case["checkpoints"]}-leg-{i+1}'
                experiment=worker.plan(context,out,'workspace_curve',envelope['settings'],envelope['budget_s'])
                assert experiment['status']=='planned' and experiment['experiment_only']
                assert not experiment['actual_current_robot_state_verified']
                fields=['path_waypoints','source_timing']
                assert all(experiment[k]==original[k] for k in fields),(side,i,'path drift')
                from drum_planner.timing import core
                import numpy as np
                actual=experiment['timing'];reference=original['timing']
                if actual['kind']=='time_density':
                    actual_path,reference_path=core.decode(actual['path']),core.decode(reference['path'])
                    assert np.array_equal(actual_path.x,reference_path.x) and np.array_equal(actual_path.c,reference_path.c)
                assert actual['duration_s']<=1.01*reference['duration_s']
                assert experiment['outward'][0]['positions_rad']==original['outward'][0]['positions_rad']
                assert experiment['outward'][-1]['positions_rad']==original['outward'][-1]['positions_rad']
                report=experiment['experiment']['report']
                # Experiment worker source participates in pool identity. GPU
                # routing legitimately invalidates historical pools once; exact
                # geometry and selected index above/below still must match.
                same_pool_key=report['pool_key']==envelope['report']['pool_key']
                if same_pool_key:assert report['pool_reused']
                assert report['selected']==envelope['report']['selected']
                rows.append(dict(side=side,checkpoints=case['checkpoints'],leg=i+1,
                    source=str(path),experiment=str(out/'plan.json'),bit_identical_fields=fields,
                    same_selection=True,same_pool_key=same_pool_key,
                    pool_reused=report['pool_reused'],duration_s=actual['duration_s'],
                    cpu_reference_duration_s=reference['duration_s'],
                    duration_ratio=actual['duration_s']/reference['duration_s'],
                    timing_backend=experiment['experiment']['retiming']['actual_backend']))
    atomic_json(a.output/'verification.json',dict(passed=True,cases=rows,camera_access=False,motor_commands=0))
    print('EXPERIMENT / HARDWARE WORKSPACE PARITY PASS:',len(rows),'legs',flush=True)


if __name__=='__main__':main()
