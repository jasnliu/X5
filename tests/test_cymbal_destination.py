"""Device-free tests for the standalone static right-arm destination program."""
import copy,json,subprocess,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,JOINTS,atomic_json,calibration,digest,endpoint
from drum_planner.model import generate
from cymbal_destination import inputs,solver,output,cli,acquisition,aim,goal_limits
from cymbal_destination.config import VERSION,ANGLE_CONVENTION,AIM_CONVENTION,MAX_INWARD_YAW_DEG


def toward(kin,q,n):
    # Existing elevation/J7 tests use a synthetic inward radial reference aligned
    # with their stick, so they continue isolating those independent constraints.
    d=kin.fk(q)['axis'];u=d-n*(d@n);return u/np.linalg.norm(u)


@pytest.fixture(scope='module')
def kin(tmp_path_factory):
    folder=tmp_path_factory.mktemp('cymbal_model');generate(folder,DEFAULT_CALIBRATION)
    k=solver.Kinematics(folder/'robot.urdf','right');k.test_path=folder/'robot.urdf';return k


@pytest.fixture
def task(kin):
    q=np.array([-.4,.4,.2,1.,.006,-.022,1.3]);f=kin.fk(q,jac=True)
    n=f['Jtip'][:,6];n=n/np.linalg.norm(n)
    assert n[2]>0
    return q,f['tip'],n


@pytest.mark.parametrize('mode',[0.,2.,21.,None])
@pytest.mark.parametrize('q7_offset',[-.2,.2])
def test_analytic_jacobian(kin,task,mode,q7_offset):
    q,p,n=task;q=q.copy();q[6]+=q7_offset;t=solver.Task(kin,p,n,mode,toward(kin,q,n));delta=np.eye(7)*1e-6
    numeric=np.column_stack([(t.fun(q+d)-t.fun(q-d))/2e-6 for d in delta])
    np.testing.assert_allclose(t.jac(q),numeric,atol=1e-7,rtol=1e-6)


@pytest.mark.parametrize('degrees,accepted',[(0,True),(3,True),(-3,False),(9,True),(21,True),(45,True),(80,True),(-21,False)])
def test_signed_tilt_examples(kin,task,degrees,accepted):
    q,_,n=task;q=q.copy();q[6]-=np.radians(degrees);p=kin.fk(q)['tip']
    proof=solver.geometry(kin,q,p,n,toward(kin,q,n))
    assert proof['stick_tilt_deg']==pytest.approx(degrees,abs=1e-10)
    assert solver.acceptable(proof)==accepted


def test_actual_j7_axis_not_assumed_joint5(kin,task):
    q,p,n=task;proof=solver.geometry(kin,q,p,n,toward(kin,q,n));f=kin.fk(q,jac=True)
    np.testing.assert_allclose(proof['j7_axis_world'],f['Jrotation'][:,6])
    assert np.linalg.norm(f['Jrotation'][:,4]-f['Jrotation'][:,6])>.5
    assert proof['negative_j7_toward_disc_cosine']==pytest.approx(1.)


@pytest.mark.parametrize('yaw',[45.,-45.,155.,-155.,180.])
def test_outside_inward_aim_rejected(kin,task,yaw):
    q,p,n=task;proof=solver.geometry(kin,q,p,n,toward(kin,q,n))
    # Regression: the old final gate entirely ignored sideways/outward aiming.
    proof.update(inward_yaw_deg=yaw,inward_cosine=float(np.cos(np.radians(yaw))),
                 stick_plane_projection_norm=1.,aim_convention='disc_plane_grip_to_tip_inward_radial_v1')
    assert not solver.acceptable(proof)


def test_r1_reference_shaft_is_above_parallel(kin):
    # User's accepted R1 example; original camera disc normal, no target shift.
    q=endpoint(ROOT/'destinations/right/destinationR1.json')
    n=np.array([-.37502364,.42914250,.82170189]);n/=np.linalg.norm(n)
    f=kin.fk(q);proof=solver.geometry(kin,q,f['tip'],n,toward(kin,q,n))
    # This is an ANGLE reference, not a claim R1 hits green or has exact J7 roll.
    assert proof['stick_tilt_deg']==pytest.approx(21.06279,abs=.001)
    assert np.all(q>=kin.lower) and np.all(q<=kin.upper)


def test_shaft_above_disc_tangent_at_tip_not_grip_to_tip_elevation(kin,task):
    q,_,n=task;q=q.copy();q[6]-=np.radians(21.)
    f=kin.fk(q);gripper=f['tip']-.2794*f['axis']
    assert (gripper-f['tip'])@n>0  # Actual shaft lies ABOVE tangent through tip.
    proof=solver.geometry(kin,q,f['tip'],n,toward(kin,q,n))
    assert proof['stick_tilt_deg']==pytest.approx(21.)
    assert solver.acceptable(proof)  # No arbitrary 8-degree rejection.


@pytest.mark.parametrize('change',[dict(tip_error_m=.001),dict(j7_plane_error_deg=1.),
                                  dict(negative_j7_toward_disc_cosine=-1.),dict(stick_tilt_deg=float('nan')),
                                  dict(stick_tilt_deg=-.001),dict(stick_tilt_deg=91.),
                                  dict(angle_convention='grip_to_tip'),dict(tip_error_m=float('inf'))])
def test_invalid_endpoint_rejected(kin,task,change):
    q,p,n=task;proof=solver.geometry(kin,q,p,n,toward(kin,q,n));proof.update(change)
    assert not solver.acceptable(proof)


def test_actual_tool_radius_matches_j7_numeric_motion(kin,task):
    q,p,n=task;step=1e-6;d=np.eye(7)[6]*step
    finite=(kin.fk(q+d)['tip']-kin.fk(q-d)['tip'])/(2*step)
    np.testing.assert_allclose(finite,kin.fk(q,jac=True)['Jtip'][:,6],atol=1e-9)
    assert np.linalg.norm(finite)==pytest.approx(.5245,abs=1e-8)


def test_solver_finds_feasible_exact_postures_without_native_devices(kin,task,monkeypatch):
    q,p,n=task
    class Oracle:
        def check(self,poses):return {'valid':[True]*len(poses)}
    monkeypatch.setattr(solver,'IK_TILT_MODES',(0.,None))
    monkeypatch.setattr(solver,'seeds',lambda *_:[q+.03,q-.03,(kin.lower+kin.upper)/2])
    rows,report=solver.candidates(kin,Oracle(),q,p,n,toward(kin,q,n),lambda _:None)
    assert rows and report['endpoint_valid_candidates']==len(rows)
    for row in rows:
        assert solver.acceptable(solver.geometry(kin,row['q'],p,n,toward(kin,q,n)))
        assert np.all(row['q']>=kin.lower) and np.all(row['q']<=kin.upper)
    assert rows[0]['geometry']['stick_tilt_deg']==pytest.approx(0.,abs=1e-5)


def test_seeds_do_not_modify_start(kin,task):
    q,_,_=task;before=q.copy();seeds=solver.seeds(kin,q)
    np.testing.assert_array_equal(before,q)
    assert len(seeds)==34 and all(np.all(s>=kin.lower) and np.all(s<=kin.upper) for s in seeds)


def test_current_snapshot_corrected_angle_diagnostic_not_a_destination(kin):
    from cymbal_destination.diagnostic import explain
    p=np.array([.5194547187448498,-.3579550960874901,.4815788525734128])
    n=np.array([-.3752368445515765,.4267983048316658,.8228247185663495])
    q=np.array([-.4139266617,.3544151596,.230679961,.8188665533,.006077545,-.0223664154,1.4])
    r=explain(kin,q,p,n,toward(kin,q,n),[q])
    assert r['diagnostic_only'] and r['not_a_destination'] and not r['global_impossibility_proven']
    best=r['best_exact_tip_found']
    assert best['geometry']['stick_tilt_deg']==pytest.approx(6.15577,abs=.01)
    assert best['geometry']['tip_error_m']<1e-6
    assert any(x['joint']=='J7' and x['side']=='upper' for x in best['active_joint_bounds'])
    assert not r['collision_validated'] and 'below parallel' not in r['summary']


def test_continuous_search_keeps_above_eight_degrees(kin,task,monkeypatch):
    # Narrow synthetic test bounds around a 21-degree valid posture force the
    # free-angle branch. Real models/limits are not modified by this fixture.
    q,_,n=task;q=q.copy();q[6]-=np.radians(21.);p=kin.fk(q)['tip']
    k=copy.copy(kin);k.lower=q-(.02+np.radians(3.));k.upper=q+(.02+np.radians(3.))
    class Oracle:
        def check(self,poses):return {'valid':[True]*len(poses)}
    monkeypatch.setattr(solver,'seeds',lambda *_:[q+.005,q-.005])
    rows,r=solver.candidates(k,Oracle(),q,p,n,toward(kin,q,n),lambda _:None)
    assert rows and r['continuous_elevation_search'] and r['maximum_elevation_cap_deg'] is None
    assert all(row['geometry']['stick_tilt_deg']>8. for row in rows)
    assert all(row['requested_tilt_deg'] is None for row in rows)
    assert all(solver.acceptable(row['geometry']) for row in rows)


def test_orientation_is_frame_rotation_invariant(kin,task):
    from scipy.spatial.transform import Rotation
    q,p,n=task;q=q.copy();q[6]-=.3
    f=kin.fk(q,jac=True);p=f['tip'];R=Rotation.from_rotvec([.5,-.7,.8]).as_matrix()
    class Rotated:
        def fk(self,*args,**kwargs):
            out=dict(f)
            for key in ('axis','tip','Jrotation','Jtip'):out[key]=R@f[key]
            return out
    a=solver.geometry(kin,q,p,n,toward(kin,q,n));b=solver.geometry(Rotated(),q,R@p,R@n,R@toward(kin,q,n))
    for key in ('stick_tilt_deg','j7_plane_error_deg','negative_j7_toward_disc_cosine'):
        assert a[key]==pytest.approx(b[key],abs=1e-10)
    np.testing.assert_array_equal(a['stick_direction'],f['axis'])
    np.testing.assert_array_equal(a['shaft_direction_tip_to_grip'],-f['axis'])


def test_route_quality_dominates_edge_preference_but_parallel_still_preferred():
    proof=lambda tilt,yaw:dict(stick_tilt_deg=tilt,inward_yaw_deg=yaw)
    # Equal routes favor either edge symmetrically; 10% shorter wins even at 0°.
    assert solver.preference_score(1.,proof(7.,MAX_INWARD_YAW_DEG))==solver.preference_score(1.,proof(7.,-MAX_INWARD_YAW_DEG))
    assert solver.preference_score(1.,proof(7.,MAX_INWARD_YAW_DEG))<solver.preference_score(1.,proof(7.,0.))
    assert solver.preference_score(.9,proof(7.,0.))<solver.preference_score(1.,proof(7.,MAX_INWARD_YAW_DEG))
    assert solver.preference_score(1.,proof(7.,MAX_INWARD_YAW_DEG))<solver.preference_score(1.,proof(21.,MAX_INWARD_YAW_DEG))


@pytest.mark.parametrize('status',['ready','no_feasible_destination','failed'])
def test_obsolete_angle_result_rejected_before_loading_payload(tmp_path,status):
    atomic_json(tmp_path/'result.json',dict(version='cymbal-prime-destination-v1',status=status))
    with pytest.raises(ValueError,match='destination constraints are obsolete'):output.verified(tmp_path)


def test_export_rejects_obsolete_convention(kin,task,tmp_path):
    q,p,n=task;d=output.destination(q,solver.geometry(kin,q,p,n,toward(kin,q,n)),p,n,DEFAULT_CALIBRATION,'scene',joint_bounds=(kin.lower,kin.upper))
    d['version']='cymbal-prime-destination-v1';atomic_json(tmp_path/'source.json',d)
    with pytest.raises(ValueError,match='destination constraints are obsolete'):
        output.export(tmp_path/'source.json',tmp_path/'export.json',digest(tmp_path/'source.json'))
    assert not (tmp_path/'export.json').exists()


def target_object():
    n=np.array([0.,.6,.8]);red=np.array([.5,-.3,.4]);green=red+.0762*n
    return dict(schema='x5-cymbal-target-v1',valid=True,calibration_sha256=digest(DEFAULT_CALIBRATION),
        target=dict(world_m=red.tolist()),above_target=dict(world_m=green.tolist(),direction='estimated_disc_upward_normal'),
        flat_disc=dict(normal_world=n.tolist(),center_world_m=red.tolist(),radius_m=.2))


def test_exact_dt1_target_and_normal(tmp_path):
    path=tmp_path/'target.json';atomic_json(path,target_object());obj,p,n=inputs.target(path,DEFAULT_CALIBRATION)
    np.testing.assert_allclose(p,np.array(obj['target']['world_m'])+.0762*n,atol=1e-12)


@pytest.mark.parametrize('kind',['changed_point','vertical_offset','down_normal','nan','invalid','calibration','bad_disc'])
def test_target_rejections(tmp_path,kind):
    obj=target_object()
    if kind=='changed_point':obj['above_target']['world_m'][0]+=.001
    if kind=='vertical_offset':obj['above_target']['direction']='world_vertical'
    if kind=='down_normal':obj['flat_disc']['normal_world'][2]*=-1
    if kind=='nan':obj['above_target']['world_m'][0]=float('nan')
    if kind=='invalid':obj['valid']=False
    if kind=='calibration':obj['calibration_sha256']='wrong'
    if kind=='bad_disc':obj['flat_disc']['radius_m']=-1
    path=tmp_path/'target.json';path.write_text(json.dumps(obj))
    with pytest.raises(ValueError):inputs.target(path,DEFAULT_CALIBRATION)


@pytest.fixture
def pair():
    cal=calibration(DEFAULT_CALIBRATION);meta=copy.deepcopy(cal['camera'])
    meta.update(protocol=1,alignment='native_depth',frame_sync=True,encoding='mjpeg')
    meta['depth'].update(width=1280,height=800,format_enum=8,fps=10)
    meta['color'].update(width=1280,height=720,format_enum=5,fps=10)
    return np.ones((800,1280),np.uint16)*600,np.zeros((720,1280,3),np.uint8),dict(scale_m=.001,depth_timestamp_us=100000,color_timestamp_us=120000),meta,cal


def test_pair_geometry(pair):assert inputs.validate_pair(*pair)==.001


@pytest.mark.parametrize('kind',['skew','serial','firmware','crop','K','extrinsics','depth_format','scale','sync'])
def test_pair_rejects_mismatch(pair,kind):
    depth,color,h,m,cal=pair
    if kind=='skew':h['color_timestamp_us']+=30000
    if kind in ('serial','firmware'):m[kind]='changed'
    if kind=='crop':m['depth']['width']=640
    if kind=='K':m['depth']['K'][0][0]+=1.
    if kind=='extrinsics':m['T_color_depth_m'][0][3]+=.01
    if kind=='depth_format':m['depth']['format_enum']=5
    if kind=='scale':h['scale_m']=float('nan')
    if kind=='sync':m['frame_sync']=False
    with pytest.raises(ValueError):inputs.validate_pair(depth,color,h,m,cal)


def test_destination_one_sample_compatible_and_export_no_overwrite(kin,task,tmp_path):
    q,p,n=task;proof=solver.geometry(kin,q,p,n,toward(kin,q,n))
    obj=output.destination(q,proof,p,n,DEFAULT_CALIBRATION,'test_scene',joint_bounds=(kin.lower,kin.upper))
    source=tmp_path/'one.json';atomic_json(source,obj)
    assert len(obj['samples'])==1 and obj['duration_s']==0 and obj['capture_kind']=='destination'
    assert 'timing' not in obj and 'trajectory' not in obj and not obj['motion_authorized']
    np.testing.assert_array_equal(endpoint(source),q)
    target=tmp_path/'export.json';output.export(source,target,digest(source))
    assert target.read_bytes()==source.read_bytes()
    with pytest.raises(FileExistsError):output.export(source,target,digest(source))
    before=target.read_bytes();old=digest(source);source.write_text('{}')
    with pytest.raises(ValueError,match='changed'):output.export(source,tmp_path/'bad.json',old)
    assert target.read_bytes()==before and not (tmp_path/'bad.json').exists()


@pytest.mark.parametrize('args',[['--replay','anything'],['--open','x','--state','y'],['--seconds','nan'],
    ['--seconds','-1'],['--execute'],['--left']])
def test_invalid_cli_never_acquires(monkeypatch,args):
    monkeypatch.setattr(cli,'Job',lambda *a,**kw:pytest.fail('Must not access a device'))
    with pytest.raises(SystemExit):cli.main(args)


def test_existing_output_never_overwritten(monkeypatch,tmp_path):
    monkeypatch.setattr(cli,'Job',lambda *a,**kw:pytest.fail('Must not access a device'))
    assert cli.main(['--output',str(tmp_path),'--headless'])==1


def test_offline_worker_cannot_open_can():
    code='from drum_planner.experiment.config import no_can; no_can(); import socket; socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)'
    p=subprocess.run([sys.executable,'-c',code],cwd=ROOT,text=True,capture_output=True)
    assert p.returncode!=0 and 'CAN' in p.stderr


def test_acquisition_failed_child_cleanup_without_hardware(monkeypatch,tmp_path):
    from drum_planner import read_only
    pose=dict(schema='x5-planner-start-state-v1',coordinate_convention='encoder_to_joint',right_rad=[0.]*7,left_rad=[0.]*7,
        right_gripper_closed_motor_joint_rad=float(np.radians(7)),left_gripper_opening_m=0.,calibration_sha256=digest(DEFAULT_CALIBRATION))
    state=tmp_path/'state.json';atomic_json(state,pose)
    class Child:
        returncode=1
        def poll(self):return 1
    monkeypatch.setattr(acquisition.subprocess,'Popen',lambda *a,**kw:Child())
    monkeypatch.setattr(read_only,'Telemetry',lambda *a:pytest.fail('Saved replay cannot query encoders'))
    folder=tmp_path/'acq'
    with pytest.raises(RuntimeError,match='child failed'):
        acquisition.run(folder,DEFAULT_CALIBRATION,replay='missing.npz',state_path=state)
    report=json.loads((folder/'acquisition_report.json').read_text())
    assert not report['passed'] and report['motor_commands']==0 and report['telemetry'] is None


def test_out_of_range_start_never_clipped(kin,task):
    q,p,n=task;q=q.copy();q[6]=1.41
    with pytest.raises(ValueError,match='not clipped'):
        solver.solve(None,dict(urdf=str(kin.test_path),start=q.tolist()),p,n,toward(kin,q,n))


@pytest.fixture
def frozen_files(tmp_path):
    folder=tmp_path/'acq';camera=folder/'camera';camera.mkdir(parents=True)
    state=dict(schema='x5-planner-start-state-v1',coordinate_convention='encoder_to_joint',right_rad=[0.]*7,left_rad=[0.]*7,
        right_gripper_closed_motor_joint_rad=float(np.radians(7)),left_gripper_opening_m=0.,calibration_sha256=digest(DEFAULT_CALIBRATION))
    atomic_json(folder/'measured_start.json',state)
    for name in inputs.PAYLOADS:(camera/name).write_bytes(b'unit test payload, NOT camera data')
    atomic_json(camera/'target.json',target_object())
    scene=dict(schema='x5-cymbal-single-pair-scene-v1',synthetic=False,calibration_sha256=digest(DEFAULT_CALIBRATION),
        measured_state_sha256=digest(folder/'measured_start.json'),files_sha256={n:digest(camera/n) for n in inputs.PAYLOADS},
        points_file='points_base.f32',free_depth_file='depth_m.bin')
    atomic_json(camera/'scene.json',scene)
    return folder,scene


def test_frozen_payloads_load(frozen_files):
    folder,_=frozen_files;state,world,obj,p,n=inputs.load(folder,DEFAULT_CALIBRATION)
    assert Path(world['points_file']).is_absolute()
    assert world['source_scene_sha256']==digest(folder/'camera/scene.json')


@pytest.mark.parametrize('file',['target.json','display.npz','rgb.png','points_base.f32','depth_m.bin','pair.npz','state'])
def test_changed_snapshot_rejected(frozen_files,file):
    folder,_=frozen_files
    path=folder/'measured_start.json' if file=='state' else folder/'camera'/file
    path.write_bytes(path.read_bytes()+b'changed')
    with pytest.raises(ValueError,match='changed'):inputs.load(folder,DEFAULT_CALIBRATION)


@pytest.mark.parametrize('change',['incomplete','outside','other_file'])
def test_scene_references_bound_to_verified_payloads(frozen_files,change):
    folder,scene=frozen_files
    if change=='incomplete':scene['files_sha256']={}
    if change=='outside':scene['files_sha256']['../outside']='bad'
    if change=='other_file':scene['points_file']='other.bin'
    atomic_json(folder/'camera/scene.json',scene)
    with pytest.raises(ValueError):inputs.load(folder,DEFAULT_CALIBRATION)


def test_headless_open_uses_same_geometry_and_snapshot_validation(frozen_files,kin,task):
    folder,scene=frozen_files;run=folder.parent;folder.rename(run/'acquisition');folder=run/'acquisition'
    q,p,n=task;target=target_object();target['above_target']['world_m']=p.tolist()
    target['target']['world_m']=(p-.0762*n).tolist();target['flat_disc']['normal_world']=n.tolist()
    target['flat_disc']['center_world_m']=(p-.0762*n+.1*toward(kin,q,n)).tolist()
    atomic_json(folder/'camera/target.json',target)
    scene['files_sha256']['target.json']=digest(folder/'camera/target.json');atomic_json(folder/'camera/scene.json',scene)
    state,world,*_=inputs.load(folder,DEFAULT_CALIBRATION)
    req=dict(urdf=str(kin.test_path),start=state['right_rad'],left=state['left_rad'],scene=world)
    atomic_json(run/'backend_request.json',req)
    proof=solver.geometry(kin,q,p,n,toward(kin,q,n))
    d=output.destination(q,proof,p,n,DEFAULT_CALIBRATION,world['source_scene_sha256'],joint_bounds=(kin.lower,kin.upper));atomic_json(run/'destination.json',d)
    r=dict(schema='x5-cymbal-destination-result-v1',version=VERSION,angle_convention=ANGLE_CONVENTION,geometry=proof,
        joint_position_policy=d['joint_position_policy'],
        aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,
        goal_joint_reserve_policy=goal_limits.policy(),status='ready',endpoint_only=True,motion_authorized=False,
        hardware_execution_available=False,calibration=str(DEFAULT_CALIBRATION),request_sha256=digest(run/'backend_request.json'),
        model=dict(calibrated_urdf_sha256=digest(kin.test_path)),destination_sha256=digest(run/'destination.json'),goal_rad=q.tolist())
    atomic_json(run/'result.json',r)
    assert output.verified(run)[0]['status']=='ready'
    saved=copy.deepcopy(r);r['geometry']['stick_tilt_deg']=-21.;atomic_json(run/'result.json',r)
    with pytest.raises(ValueError,match='geometry labels'):output.verified(run)
    r=saved;atomic_json(run/'result.json',r)
    # Matching files/hashes are insufficient if the alleged posture misses the actual green dot.
    q[0]+=.02;r['goal_rad']=q.tolist();d['samples'][0]['positions_rad']=q.tolist()
    atomic_json(run/'destination.json',d);r['destination_sha256']=digest(run/'destination.json');atomic_json(run/'result.json',r)
    with pytest.raises(ValueError,match='target constraints'):output.verified(run)


@pytest.mark.parametrize('yaw,accepted',[(0,True),(20,True),(-20,True),(25,True),(-25,True),
    (20.01,True),(-20.01,True),
    (25.01,False),(-25.01,False),(45,False),(-45,False),(155,False),(-155,False),(180,False)])
@pytest.mark.parametrize('elevation',[0.,21.,60.])
def test_directed_disc_plane_aim_examples(yaw,accepted,elevation):
    # Pure geometry fixtures: the two signed examples are symmetric, but the
    # reversed line is NOT allowed. Elevation must not distort top-down yaw.
    from scipy.spatial.transform import Rotation
    R=Rotation.from_rotvec([.4,-.7,.2]).as_matrix();n=R@np.array([0.,0.,1.]);u=R@np.array([1.,0.,0.]);v=np.cross(n,u)
    a,t=np.radians([yaw,elevation]);d=np.cos(t)*(np.cos(a)*u+np.sin(a)*v)-np.sin(t)*n
    axes=np.zeros((3,7));axes[:,6]=np.cross(n,d);axes[:,6]/=np.linalg.norm(axes[:,6])
    jac=np.zeros((3,7));jac[:,6]=n
    k=SimpleNamespace(fk=lambda *args,**kw:dict(axis=d,tip=np.zeros(3),Jrotation=axes,Jtip=jac))
    proof=solver.geometry(k,np.zeros(7),np.zeros(3),n,u)
    assert abs(proof['inward_yaw_deg'])==pytest.approx(abs(yaw),abs=1e-10)
    if abs(yaw)<180:assert proof['inward_yaw_deg']==pytest.approx(yaw,abs=1e-10)
    assert proof['stick_tilt_deg']==pytest.approx(elevation,abs=1e-10)
    assert solver.acceptable(proof)==accepted


def test_radial_reference_projects_both_red_points_without_moving_them():
    n=np.array([0.,.6,.8]);u=np.array([1.,0.,0.]);red=np.array([.4,-.3,.5]);center=red+.12*u+.03*n
    target=dict(target=dict(world_m=red.tolist()),flat_disc=dict(center_world_m=center.tolist()))
    before=copy.deepcopy(target);np.testing.assert_allclose(aim.reference(target,n),u,atol=1e-12)
    assert target==before
    from scipy.spatial.transform import Rotation
    R=Rotation.from_rotvec([.4,-.3,.8]).as_matrix();offset=np.array([2.,-3.,1.])
    rotated=dict(target=dict(world_m=(R@red+offset).tolist()),flat_disc=dict(center_world_m=(R@center+offset).tolist()))
    np.testing.assert_allclose(aim.reference(rotated,R@n),R@u,atol=1e-12)


@pytest.mark.parametrize('normal_offset',[0.,.0762,-.1])
def test_no_arbitrary_reference_when_projected_target_is_disc_center(normal_offset):
    n=np.array([0.,.6,.8]);red=np.array([.4,-.3,.5]);center=red+normal_offset*n
    target=dict(target=dict(world_m=red.tolist()),flat_disc=dict(center_world_m=center.tolist()))
    with pytest.raises(ValueError,match='Inward aim is undefined'):aim.reference(target,n)


def test_vertical_stick_has_no_top_down_direction(kin,task):
    q,p,n=task;proof=solver.geometry(kin,q,p,n,toward(kin,q,n))
    proof.update(aim.measure(-n,n,toward(kin,q,n)))
    assert not proof['aim_defined'] and not solver.acceptable(proof)


@pytest.mark.parametrize('yaw',[0.,15.,-15.,45.,-45.,150.,-150.])
def test_aim_inequality_and_penalty_jacobians(kin,task,yaw):
    # Avoid +/-25 and +/-155: a cone hinge has no two-sided derivative there.
    from scipy.spatial.transform import Rotation
    q,p,n=task;u=Rotation.from_rotvec(n*np.radians(-yaw)).apply(toward(kin,q,n))
    t=solver.Task(kin,p,n,0.,u);delta=np.eye(7)*1e-6
    numeric=np.column_stack([(t.fun(q+d)-t.fun(q-d))/2e-6 for d in delta])
    np.testing.assert_allclose(t.jac(q),numeric,atol=1e-7,rtol=1e-6)
    numeric=np.column_stack([(t.aim(q+d)-t.aim(q-d))/2e-6 for d in delta])
    np.testing.assert_allclose(t.aim_jac(q),numeric,atol=1e-7,rtol=1e-6)
    assert bool(np.all(t.aim(q)>=0))==(abs(yaw)<MAX_INWARD_YAW_DEG)


def test_final_gate_requires_explicit_inward_proof(kin,task):
    q,p,n=task;proof=solver.geometry(kin,q,p,n,toward(kin,q,n));del proof['aim_convention']
    assert not solver.acceptable(proof)


def test_inward_aim_range_is_twenty_five_degrees():
    assert MAX_INWARD_YAW_DEG==25.


@pytest.mark.parametrize('version',['cymbal-prime-destination-v1','cymbal-prime-destination-v2','cymbal-prime-destination-v3','cymbal-prime-destination-v4'])
def test_pre_inward_results_rejected_even_with_current_other_metadata(tmp_path,version):
    atomic_json(tmp_path/'result.json',dict(version=version,angle_convention=ANGLE_CONVENTION,
        aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,status='ready'))
    with pytest.raises(ValueError,match='destination constraints are obsolete'):output.verified(tmp_path)


def test_previous_20_degree_range_rejected_even_with_current_version(tmp_path):
    atomic_json(tmp_path/'result.json',dict(version=VERSION,angle_convention=ANGLE_CONVENTION,
        aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=20.,status='ready'))
    with pytest.raises(ValueError,match='destination constraints are obsolete'):output.verified(tmp_path)
