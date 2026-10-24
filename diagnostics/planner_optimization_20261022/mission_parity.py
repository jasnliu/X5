"""Full experiment checkpoint composition vs the tested hardware simulation.

Uses its saved inputs and candidate pools, but independently verifies timing
against the scene. All outputs retain experiment-only flags; no hardware access.
"""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from verify import numerical
from drum_planner.common import atomic_json,digest,DEFAULT_CALIBRATION
from drum_planner.experiment import worker,config
from drum_planner.right_zone import metadata


def main():
    p=argparse.ArgumentParser();p.add_argument('--ui',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();config.no_can()
    a.output.mkdir(parents=True,exist_ok=False);rows=[]
    for ui in a.ui:
        verification=json.loads((ui/'verification.json').read_text())
        record=next(r for r in verification['rows'] if r['checkpoints']==3)
        path=Path(record['plan']);old=json.loads(path.read_text())
        req=json.loads((path.parent/'backend_request.json').read_text());side=req.get('moving_side','right')
        context=dict(old['provenance'],moving_side=side,calibration=str(DEFAULT_CALIBRATION),
            pool_root=str(path.parent/'candidate_pool'),saved_not_live=True,can_access=False,
            checkpoints=old['destination_bindings'],checkpoint_sha256=old['checkpoint_sha256'])
        context.update(old['destination_bindings'][-1]);context['scene_sha256']=digest(context['scene'])
        context['zone_path']=req[side+'_zone']['zone_file'];context[side+'_zone']=metadata(req[side+'_zone'])
        new=worker.plan(context,a.output/side,'workspace_curve',config.settings(),120.)
        assert new['status']=='planned' and new['experiment_only'] and not new['actual_current_robot_state_verified']
        for key in ('outward','cycle','duration_outward_s','sampled_interpolated_checks','final_curve_check'):
            assert new[key]==old[key],(side,key)
        def legs(obj):return obj['checkpoint_sequence']['segments']+[obj['checkpoint_sequence']['return_leg']]
        for before,after in zip(legs(old),legs(new)):
            assert numerical(before['timing'])==numerical(after['timing'])
            planned=json.loads(Path(after['plan']).read_text())
            assert planned['experiment']['report']['pool_reused']
            assert planned['experiment']['retiming']['math_cache']['cached']
        rows.append(dict(side=side,exact_cycle=True,exact_timing=True,same_candidates=True,
                         timing_workers=new['experiment']['timing_workers'],elapsed_s=new['experiment']['elapsed_s']))
    atomic_json(a.output/'verification.json',dict(rows=rows,camera_access=False,motor_commands=0))
    print('FULL EXPERIMENT / HARDWARE MISSION PARITY PASS',flush=True)


if __name__=='__main__':main()
