"""Device-free parity with legacy Zone/Model and real native hard-zone checks."""
import copy,json,socket
from itertools import product
from pathlib import Path
import numpy as np
import pytest
from safe_zone.geometry import Model,Zone,RIGHT_TCP,MEMBERSHIP_BUFFER_M
from drum_planner import common,planning,model,right_zone
from drum_planner.experiment.config import latest_state
from drum_planner.experiment.runtime import Oracle


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('No motor/CAN access in zone tests')
        return real(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',guarded)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No camera in zone unit tests'))


@pytest.fixture(scope='module')
def reference():
    m=Model(right_zone.REFERENCE_MODEL)
    return m,Zone.load(right_zone.ZONE,m.digest,RIGHT_TCP),right_zone.constraint()


@pytest.fixture(scope='module')
def generated(tmp_path_factory):
    folder=tmp_path_factory.mktemp('zone3_robot');model.generate(folder)
    return folder


@pytest.fixture
def req(generated,reference):
    state=common.read_state(latest_state())
    return dict(schema='x5-moveit-request-v1',execute=False,urdf=str(generated/'robot.urdf'),
        srdf=str(generated/'robot.srdf'),start=state['right_rad'],left=state['left_rad'],
        left_opening_m=state['left_gripper_opening_m'],goal=common.endpoint().tolist(),
        scene=dict(synthetic=True,obstacle_padding_m=.02,boxes=[]),
        collision_policy=planning.COLLISION_POLICY,planning_seconds=3.,seed=19,
        occupied_only_preview=False,right_zone=copy.deepcopy(reference[2]))


def point(model,q):return model.transforms(dict(zip(common.JOINTS,q)))[RIGHT_TCP][:3,3]


def box_zone(tmp_path,lo,hi):
    m=Model(right_zone.REFERENCE_MODEL);z=Zone(m.digest,RIGHT_TCP)
    z.points=[list(v) for v in product(*zip(lo,hi))];z.save(tmp_path/'zone.json')
    return right_zone.constraint(tmp_path/'zone.json')


def oracle(req,folder):
    common.atomic_json(folder/'backend_request.json',req)
    return Oracle(folder/'backend_request.json',folder)


def test_new_request_always_carries_original_model_right_point_zone(tmp_path,reference):
    state=common.read_state(latest_state());r,_=planning.request(tmp_path,state,dict(synthetic=True,obstacle_padding_m=.02,boxes=[]))
    s=r['right_zone'];assert s==reference[2]
    assert s['buffer_m']==MEMBERSHIP_BUFFER_M==.005
    assert s['tcp']==RIGHT_TCP and s['frame']=='world'
    assert s['point_only'] and not s['left_arm_constrained'] and not s['stick_constrained']
    assert s['reference_urdf']==str(right_zone.REFERENCE_MODEL)
    assert s['reference_urdf']!=r['urdf']
    assert right_zone.metadata(s)['constraint_sha256']==s['constraint_sha256']
    assert 'planes' not in right_zone.metadata(s) and 'kinematic_chain' not in right_zone.metadata(s)


def test_imported_planes_are_not_trusted_and_bad_zone_fails(tmp_path,reference):
    obj=json.loads(right_zone.ZONE.read_text());obj['planes']=[[0,0,1,-500]]
    p=tmp_path/'zone.json';p.write_text(json.dumps(obj))
    a=right_zone.constraint(p);assert np.allclose(a['planes'],reference[2]['planes'])
    obj['tcp']='openarmx_left_hand_tcp';p.write_text(json.dumps(obj))
    with pytest.raises(ValueError,match='Incompatible'):right_zone.constraint(p)
    obj=json.loads(right_zone.ZONE.read_text());obj['points']=[[0,0,0],[1,0,0],[0,1,0]];p.write_text(json.dumps(obj))
    with pytest.raises(ValueError,match='3-D'):right_zone.constraint(p)


def test_cpp_file_backend_matches_legacy_fk_and_membership_400_poses(req,tmp_path,reference):
    m,z,_=reference;q=np.random.default_rng(19).uniform(-1,1,(400,7));q[:,3]=abs(q[:,3])
    q=np.vstack([req['start'],req['goal'],q]);r=copy.deepcopy(req);r['right_zone_states']=q.tolist()
    out=planning.backend(r,tmp_path,'zone_parity');assert out['status']=='zone_checked' and not out['hard_collision_checked']
    assert out['right_zone_constraint']==right_zone.metadata(req['right_zone'])
    inside=[]
    for joints,row in zip(q,out['zone_checks'],strict=True):
        p=point(m,joints);expected=z.contains(p,MEMBERSHIP_BUFFER_M)
        assert np.allclose(row['point_xyz_m'],p,atol=2e-14,rtol=0)
        assert row['valid']==expected;inside.append(expected)
    assert inside[0] and inside[1] and any(inside) and not all(inside)


@pytest.mark.parametrize('distance,valid',[(.0049999,True),(.005,True),(.005+5e-10,True),(.005+2e-9,False),(.0050001,False)])
def test_exact_legacy_membership_tolerance(req,tmp_path,reference,distance,valid):
    p=point(reference[0],req['start']);lo=p-.1;hi=p+.1;hi[0]=p[0]-distance
    req['right_zone']=box_zone(tmp_path,lo,hi);req['right_zone_states']=[req['start']]
    row=planning.backend(req,tmp_path,'boundary')['zone_checks'][0]
    assert row['valid']==valid
    assert row['max_signed_plane_distance_m']==pytest.approx(distance,abs=2e-14)


def test_left_arm_and_collision_gripper_stick_do_not_define_zone_point(req,tmp_path):
    req['right_zone_states']=[req['start'],req['goal']]
    a=planning.backend(req,tmp_path,'before')['zone_checks']
    req['left']=[2]*7  # Zone-only diagnostic is deliberately independent of left validity.
    b=planning.backend(req,tmp_path,'left_changed')['zone_checks'];assert a==b
    req['left']=[0]*7
    # Alter only the planning URDF's hand TCP offset. Zone membership must still
    # use the ORIGINAL recorded FK chain, never this modified/corrected model.
    import xml.etree.ElementTree as ET
    root=ET.parse(req['urdf'])
    j=next(j for j in root.getroot().findall('joint') if j.find('child').get('link')==RIGHT_TCP)
    j.find('origin').set('xyz','1 1 1');path=tmp_path/'changed_robot.urdf';root.write(path)
    req['urdf']=str(path)
    c=planning.backend(req,tmp_path,'tool_changed')['zone_checks'];assert a==c


def test_zone_rejects_mid_edge_even_when_endpoint_points_are_inside(req,tmp_path,reference):
    m=reference[0];a=np.asarray(req['start']);b=a.copy();a=a.copy();a[0]-=.55;b[0]+=.55
    pa,pb,pm=[point(m,q) for q in (a,b,(a+b)/2)]
    req['right_zone']=box_zone(tmp_path,np.minimum(pa,pb)-.001,np.maximum(pa,pb)+.001)
    z=Zone.load(tmp_path/'zone.json',m.digest,RIGHT_TCP)
    assert z.contains(pa,.005) and z.contains(pb,.005) and not z.contains(pm,.005)
    o=oracle(req,tmp_path)
    try:
        rows=o.call('zone',q=[a.tolist(),b.tolist(),((a+b)/2).tolist()])['rows']
        assert [r['valid'] for r in rows]==[True,True,False]
        result=o.check([a.tolist(),b.tolist()],edges=True)
        assert not all(result['valid']) and result['details']['reason']=='outside_right_zone3',result
    finally:o.close()


def test_outside_point_is_hard_invalid_in_file_and_persistent_backend(req,tmp_path,reference):
    m,z,_=reference
    qs=np.random.default_rng(41).uniform(-.8,.8,(200,7));qs[:,3]=abs(qs[:,3])
    q=next(q for q in qs if not z.contains(point(m,q),.005))
    req['check_states']=[q.tolist()]
    out=planning.backend(req,tmp_path,'outside');assert out['checks'][0]['reason']=='outside_right_zone3'
    o=oracle(req,tmp_path)
    try:
        r=o.check([q.tolist()]);assert r['valid']==[False] and r['details']['reason']=='outside_right_zone3'
        assert o.meta['right_zone_constraint']==right_zone.metadata(req['right_zone'])
    finally:o.close()


def test_scenarios_do_not_move_or_pad_the_zone_with_stick_camera_errors(req,tmp_path):
    o=oracle(req,tmp_path)
    try:
        expected=o.call('zone',q=[req['start'],req['goal']])['rows']
        o.check([req['start']],extra_padding=.03,tool_padding=.08,world_transform=[[1,0,0,.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        assert o.call('zone',q=[req['start'],req['goal']])['rows']==expected
    finally:o.close()
