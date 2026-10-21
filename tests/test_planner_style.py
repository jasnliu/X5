"""Pure route-cost math plus real MoveIt/FCL scoring, without live devices."""
import copy
import socket

import numpy as np
import pytest

from drum_planner import common, depth, model, planning, style
from safe_zone.geometry import Model


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('No CAN in style tests')
        return original(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',guarded)
    def denied(*args,**kwargs):raise AssertionError('No camera in style tests')
    monkeypatch.setattr(depth,'DepthCamera',denied)


def straight_features(clearance=.1):
    x=np.column_stack([np.linspace(0.,1.,801),np.zeros((801,2))])
    return dict(joint_bounds_valid=True,wrist_xyz_m=x,tip_xyz_m=x,
                distal_clearance_estimate_m=np.full(201,clearance))


def test_joint_arc_sampling_ignores_pauses_and_preserves_endpoints():
    q=np.array([[0.]*7,[.5]*7,[1.]*7])
    actual=style.sample_path(np.repeat(q,[5,100,3],axis=0))
    assert actual.shape==(801,7)
    assert np.allclose(actual,style.sample_path(q),atol=1e-15)
    assert np.array_equal(actual[0],q[0]) and np.array_equal(actual[-1],q[-1])
    assert np.allclose(np.diff(actual,axis=0),1./800)


@pytest.mark.parametrize('q',[[],[[0.]*6,[1.]*6],[[0.]*7],[[0.]*7,[0.]*7],
                             [[0.]*7,[float('nan')]*7],[[0.]*7,[float('inf')]*7]])
def test_invalid_or_stationary_paths_cannot_be_ranked(q):
    with pytest.raises(ValueError):style.sample_path(q)


def test_broad_bending_and_route_economy_distinguish_a_detour():
    q=style.sample_path([[0.]*7,[1.]*7]);direct=straight_features()
    a=style.metrics(q,direct)
    assert a['components']==pytest.approx(dict(workspace_economy=1.,broad_bending=0.,
                                               obstacle_proximity=0.,joint_economy=1.))
    angle=np.linspace(np.pi,0,801)
    bent=np.column_stack([.5+.5*np.cos(angle),.5*np.sin(angle),np.zeros(801)])
    b=style.metrics(q,dict(direct,wrist_xyz_m=bent,tip_xyz_m=bent))
    assert b['components']['workspace_economy']==pytest.approx(np.pi/2,rel=1e-5)
    assert 330<b['components']['broad_bending']<360
    scales=style.normalizers(b)
    assert style.score(a,scales)['total']<style.score(b,scales)['total']
    assert style.broad_turning(np.zeros((801,3)))==0.


def test_five_cm_bending_does_not_rank_fine_noise_as_macro_weaving():
    x=np.linspace(0.,1.,801)
    fine=np.column_stack([x,.001*np.sin(100*np.pi*x),np.zeros(801)])
    assert style.broad_turning(fine,.05)<.1*style.broad_turning(fine,.005)


def test_clearance_hinge_and_joint_backtracking_cost():
    q=style.sample_path([[0.]*7,[2.]*7,[1.]*7])
    measured=style.metrics(q,straight_features(.05))
    assert measured['components']['obstacle_proximity']==pytest.approx(.25)
    assert measured['components']['joint_economy']==pytest.approx(3.,abs=.006)
    assert measured['clearance_is_soft_estimate_not_safety_certificate']
    assert measured['minimum_distal_padding_estimate_m']==.05


def test_reviewed_weights_and_common_reference_normalization():
    ref=dict(components=dict(workspace_economy=2.,broad_bending=500.,obstacle_proximity=.2,joint_economy=2.))
    scales=style.normalizers(ref);cost=style.score(ref,scales)
    assert cost['weighted_components']==dict(workspace_economy=40.,broad_bending=30.,
                                            obstacle_proximity=20.,joint_economy=10.)
    assert cost['total']==100.
    cheaper=dict(components={k:v/2 for k,v in ref['components'].items()})
    assert style.score(cheaper,scales)['total']==50.
    zero=dict(components={k:0. for k in style.WEIGHTS})
    assert style.normalizers(zero)==style.NORMALIZATION_FLOORS
    assert style.score(zero,style.normalizers(zero))['total']==0.


@pytest.mark.parametrize('bad',['bounds','clearance_short','clearance_nan','clearance_negative',
                                'clearance_large','wrist_short','tip_nan','joint_short'])
def test_incomplete_features_fail_closed(bad):
    q=style.sample_path([[0.]*7,[1.]*7]);f=straight_features()
    if bad=='bounds':f['joint_bounds_valid']=False
    if bad=='clearance_short':f['distal_clearance_estimate_m']=[]
    if bad=='clearance_nan':f['distal_clearance_estimate_m'][0]=np.nan
    if bad=='clearance_negative':f['distal_clearance_estimate_m'][0]=-.01
    if bad=='clearance_large':f['distal_clearance_estimate_m'][0]=.101
    if bad=='wrist_short':f['wrist_xyz_m']=f['wrist_xyz_m'][:-1]
    if bad=='tip_nan':f['tip_xyz_m'][0,0]=np.nan
    if bad=='joint_short':q=q[:-1]
    with pytest.raises(ValueError):style.metrics(q,f)


@pytest.fixture(scope='module')
def generated(tmp_path_factory):
    folder=tmp_path_factory.mktemp('style_model');info=model.generate(folder)
    start=common.read_state(common.ROOT/'drum_planner/config/example_start.json')
    req=dict(schema='x5-moveit-request-v1',execute=False,urdf=str(folder/'robot.urdf'),
             srdf=str(folder/'robot.srdf'),start=start['right_rad'],left=start['left_rad'],
             left_opening_m=start['left_gripper_opening_m'],goal=common.endpoint().tolist(),
             scene=dict(synthetic=True,obstacle_padding_m=.02,boxes=[]),
             collision_policy=planning.COLLISION_POLICY,occupied_only_preview=False)
    return req,info


def test_backend_fk_uses_calibrated_wrist_and_actual_full_stick(generated,tmp_path):
    req,info=generated;q=style.sample_path([req['start'],req['goal']])
    scored=planning.backend(dict(req,style_paths=[q.tolist()]),tmp_path,'fk')
    assert scored['status']=='scored' and not scored['hard_collision_checked']
    assert not scored['motion_authorized'] and 'outward' not in scored
    f=scored['style_features'][0]
    assert f['joint_bounds_valid'] and f['distal_clearance_estimate_m']==[.1]*201
    robot=Model(req['urdf'])
    for i in [0,139,400,800]:
        transforms=robot.transforms(dict(zip(common.JOINTS,q[i])))
        assert np.allclose(f['wrist_xyz_m'][i],transforms['openarmx_right_hand'][:3,3],atol=1e-12)
        tip=transforms['right_drumstick']@np.array([0,0,info['stick_length_from_mount_m']/2,1])
        assert np.allclose(f['tip_xyz_m'][i],tip[:3],atol=1e-12)
    assert style.metrics(q,f)['components']['obstacle_proximity']==0.


def test_soft_scoring_never_exempts_real_stick_collision_or_changes_padding(generated,tmp_path):
    req,_=generated;req=copy.deepcopy(req);q=req['start']
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,q)))['right_drumstick']
    req['scene']['boxes']=[dict(id='known_stick_collision',xyz=(T[:3,3]+T[:3,2]*.14).tolist(),size=[.025]*3)]
    before=planning.backend(dict(req,check_states=[q]),tmp_path,'before')
    report=planning.backend(dict(req,style_paths=[[q]*801]),tmp_path,'cost')
    after=planning.backend(dict(req,check_states=[q]),tmp_path,'after')
    assert before['checks']==after['checks']
    assert not after['checks'][0]['valid']
    assert any('right_drumstick' in pair for pair in after['checks'][0]['contacts'])
    assert report['status']=='scored' and report['hard_collision_checked'] is False
    assert report['style_features'][0]['distal_clearance_estimate_m']==[0.]*201
    assert req['scene']['obstacle_padding_m']==.02


def test_soft_padding_bands_detect_nearby_stick_without_nominal_contact(generated,tmp_path):
    req,_=generated;req=copy.deepcopy(req);q=req['start']
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,q)))['right_drumstick']
    # Tiny box 5 cm sideways from the tip-side shaft, away from the wrist.
    xyz=T@np.array([.05,0.,.14,1.])
    req['scene']['boxes']=[dict(id='near_stick',xyz=xyz[:3].tolist(),size=[.002]*3)]
    result=planning.backend(dict(req,style_paths=[[q]*801]),tmp_path,'nearby')
    assert result['status']=='scored'
    distance=np.array(result['style_features'][0]['distal_clearance_estimate_m'])
    assert np.all((distance>.025)&(distance<.06)),distance
    assert np.allclose(distance,distance[0])


@pytest.mark.parametrize('bad',['empty','short','mixed','too_many','wrong_joints','nonfinite','bounds'])
def test_backend_rejects_malformed_cost_inputs(generated,tmp_path,bad):
    req,_=generated;req=copy.deepcopy(req);q=req['start'];req['style_paths']=[[q]*801]
    if bad=='empty':req['style_paths']=[]
    if bad=='short':req['style_paths']=[[q]*800]
    if bad=='mixed':req['check_states']=[q]
    if bad=='too_many':req['style_paths']=[[q]*801]*65
    if bad=='wrong_joints':req['style_paths']=[[[0.]*6]*801]
    if bad=='nonfinite':req['style_paths']=[[[None]*7]*801]
    if bad=='bounds':req['style_paths']=[[[100.]*7]*801]
    report=planning.backend(req,tmp_path,bad)
    if bad=='bounds':
        assert report['status']=='scored' and not report['style_features'][0]['joint_bounds_valid']
    else:assert report['status']=='error'
    assert not report['motion_authorized'] and 'outward' not in report
