"""Opt-in OFFLINE validation of a real saved green-target solution; no devices.

Reopens an unmodified completed run and independently initializes native collision
and FK checks. Never shifts the target or substitutes a hypothetical success task.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from cymbal_destination import output,solver,aim,goal_limits
from cymbal_destination.config import MAX_INWARD_YAW_DEG
from drum_planner.common import atomic_json,endpoint,digest
from drum_planner.experiment.config import no_can
from drum_planner.experiment.runtime import Oracle


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True);a=p.parse_args();no_can()
    source=a.run.resolve();r,req,state,world,target,green,normal=output.verified(source)
    assert r['status']=='ready'
    q=endpoint(source/'destination.json');np.testing.assert_array_equal(q,r['goal_rad'])
    saved=json.loads((source/'destination.json').read_text())
    assert saved['sample_count']==1 and len(saved['samples'])==1 and saved['duration_s']==0
    assert not saved['motion_authorized'] and not list(source.glob('*trajectory*'))
    np.testing.assert_array_equal(saved['target_world_m'],green)
    np.testing.assert_allclose(green,np.asarray(target['target']['world_m'])+.0762*normal,atol=1e-12)
    kin=solver.Kinematics(req['urdf'],'right');g=solver.geometry(kin,q,green,normal,aim.reference(target,normal))
    assert solver.acceptable(g) and np.all(q>=kin.lower) and np.all(q<=kin.upper)
    reserve=goal_limits.verify(q,saved['goal_joint_reserve'],(kin.lower,kin.upper))
    assert np.all(q-kin.lower>=np.radians(goal_limits.policy()['lower_deg'])-1e-10)
    assert np.all(kin.upper-q>=np.radians(goal_limits.policy()['upper_deg'])-1e-10)
    # Explicit point-to-plane shaft relation, independent of the label/formula.
    f=kin.fk(q,jac=True);shaft_back=f['tip']-.2794*f['axis']
    shaft_height=float((shaft_back-f['tip'])@normal);assert shaft_height>=-1e-9
    # Independent directed angle; abs(dot) would incorrectly accept pointing away.
    ray=np.asarray(target['flat_disc']['center_world_m'])-target['target']['world_m']
    ray-=normal*(ray@normal);ray/=np.linalg.norm(ray)
    projected=f['axis']-normal*(f['axis']@normal);projected/=np.linalg.norm(projected)
    cosine=float(projected@ray);independent_yaw=float(np.degrees(np.arccos(np.clip(cosine,-1,1))))
    assert cosine>0 and independent_yaw<=MAX_INWARD_YAW_DEG+1e-6
    assert abs(abs(g['inward_yaw_deg'])-independent_yaw)<1e-8
    a.report.parent.mkdir(parents=True,exist_ok=True)
    native=Oracle(source/'backend_request.json',a.report.parent)
    try:
        check=native.check(np.array([q]));assert check['valid']==[True]
        fk=native.call('fk',q=[q.tolist()]);error=float(np.linalg.norm(np.array(fk['tip_xyz_m'][0])-green))
        assert error<1e-5
    finally:native.close()
    assert r['approach']['valid'] and r['approach']['trajectory_discarded']
    proof=dict(passed=True,hypothetical_test_only=False,actual_dt1_target_success=True,
        target_shift_m=0.,hardware_access=False,motor_commands=0,source_run=str(source),
        elapsed_s=r['elapsed_s'],geometry=g,goal_joint_reserve=reserve,approach=r['approach'],
        shaft_height_above_tip_tangent_m=shaft_height,independent_absolute_inward_yaw_deg=independent_yaw,
        independent_native_tip_error_m=error,independent_native_collision_zone=check,
        target_sha256=digest(source/'acquisition/camera/target.json'),source_scene_sha256=world['source_scene_sha256'])
    atomic_json(a.report,proof);print(json.dumps(proof,indent=2))


if __name__=='__main__':main()
