"""Traditional-grip CAD draft: no devices, no motor commands, no fitted goals."""
import ast
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial import cKDTree

from drum_planner import common, model
from drum_planner.experiment import algorithms, config
from drum_planner.visual_mesh import origin_transform
from safe_zone.geometry import Model


@pytest.fixture(scope='module')
def draft(tmp_path_factory):
    folder=tmp_path_factory.mktemp('traditional-grip')
    manifest=model.generate(folder,moving_side='left')
    return folder,ET.parse(folder/'robot.urdf').getroot(),manifest


def test_halves_form_one_matching_bore_and_front_face(draft):
    _,root,manifest=draft
    fronts=[];rear=[];cross_sections=[]
    for suffix,file,front_y,rear_y in (
            ('left','gripper2left.stl',95.75,-30.75),
            ('right','gripper2right.stl',-95.75,30.75)):
        name=f'openarmx_left_{suffix}_finger'
        joint=next(j for j in root.findall('joint') if j.find('child').get('link')==name)
        assert joint.get('type')=='fixed' and joint.find('mimic') is None
        T=origin_transform(joint.find('origin'))
        fronts.append((T@np.r_[np.array([-27.15,front_y,12.4])*.001,1])[:3])
        rear.append((T@np.r_[np.array([-27.15,rear_y,12.4])*.001,1])[:3])
        v=model.stl_vertices(common.ROOT/'gripper_models'/file).reshape(-1,3)
        ring=v[abs(v[:,1]-front_y)<.001]*.001
        points=ring@T[:3,:3].T+T[:3,3];cross_sections.append(points)
        radii=np.linalg.norm(points-fronts[-1],axis=1)
        assert np.all(np.minimum(abs(radii-.0074),abs(radii-.0124))<1e-8)
        visual=root.find(f"link[@name='{name}']/visual/geometry/mesh")
        collision=root.find(f"link[@name='{name}']/collision/geometry/mesh")
        assert visual.attrib==collision.attrib
        assert visual.get('filename').endswith('/'+file)
        assert visual.get('scale')=='.001 .001 .001'
    assert np.allclose(fronts,[manifest['left_tool']['sleeve_front_center_hand_m']]*2,atol=1e-15)
    assert np.allclose(rear,[manifest['left_tool']['sleeve_rear_center_hand_m']]*2,atol=1e-15)
    # Opposite semicircles, not two overlaid half shells.
    assert cross_sections[0][:,0].min()>-1e-8 and cross_sections[0][:,0].max()>.012
    assert cross_sections[1][:,0].max()<1e-8 and cross_sections[1][:,0].min()<-.012
    a=cross_sections[0].copy();a[:,0]*=-1
    assert cKDTree(cross_sections[1]).query(a)[0].max()<1e-8


def test_stick_is_perpendicular_and_exposure_is_from_front_not_center(draft):
    _,root,manifest=draft
    T=origin_transform(root.find("joint[@name='left_drumstick_fixed']/origin"))
    length=model.stick_length(root,'left_drumstick')
    assert length==pytest.approx(.38685)
    assert model.stick_length(root,'right_drumstick')==pytest.approx(.4094)
    assert np.allclose(T[:3,2],[0,1,0])
    tip=T[:3,3]+T[:3,2]*length/2;back=T[:3,3]-T[:3,2]*length/2
    left=manifest['left_tool'];front=np.array(left['sleeve_front_center_hand_m'])
    assert np.allclose(back,left['sleeve_rear_center_hand_m'])
    assert np.allclose(tip,left['tip_hand_m'])
    assert np.linalg.norm(tip-front)==pytest.approx(10.25*.0254)
    assert np.linalg.norm(front-back)==pytest.approx(.1265)
    assert left['mounting_physically_verified'] is False


def test_mounting_base_features_retained_and_closed_target_matches_launcher(draft):
    # Same CAD attachment-foot vertices as the old left half, unchanged files.
    def foot(file):
        v=np.unique(model.stl_vertices(common.ROOT/'gripper_models'/file).reshape(-1,3),axis=0)
        return v[v[:,2]<-12.49]
    expected=foot('gripperLeft.stl')
    for file in ('gripper2left.stl','gripper2right.stl'):
        assert np.array_equal(foot(file),expected)
    # Read the launcher constant without importing/initializing motor modules.
    tree=ast.parse((common.ROOT/'camera_playback/left_hold.py').read_text())
    degrees=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
                 and any(isinstance(t,ast.Name) and t.id=='LEFT_GRIPPER_CLOSED_DEGREES' for t in n.targets))
    info=draft[2]['left_tool']
    assert info['closed_motor_raw_rad']==pytest.approx(np.radians(degrees))
    assert info['closed_motor_joint_rad']==-info['closed_motor_raw_rad']


def test_center_points_forward_and_endpoint_uses_same_rigid_attachment(draft):
    folder,_,info=draft;m=Model(folder/'robot.urdf')
    center=np.radians([0,0,0,0,-50,0,0])
    for q in (center,common.endpoint(side='left')):
        frames=m.transforms(dict(zip(common.LEFT_JOINTS,q)))
        hand=frames['openarmx_left_hand'];stick=frames['left_drumstick']
        tip=stick[:3,3]+stick[:3,2]*model.LEFT_STICK_LENGTH/2
        assert np.allclose(tip,(hand@np.r_[info['left_tool']['tip_hand_m'],1])[:3])
        if np.array_equal(q,center):assert stick[0,2]>.7  # world forward = +X
    # Draft plausibility, NOT a calibrated snare measurement or a planner target.
    assert np.allclose(tip,[.39902229,.20028415,.21058989],atol=1e-7)


def test_candidate_cache_changes_when_collision_mesh_changes(draft,monkeypatch):
    folder,_,_=draft
    context=dict(scene_sha256='scene',state_sha256='state',calibration=str(common.DEFAULT_CALIBRATION))
    req=dict(moving_side='left',urdf=str(folder/'robot.urdf'),srdf=str(folder/'robot.srdf'),
             goal=common.endpoint(side='left').tolist(),collision_policy='observed_obstacles')
    before=algorithms.pool_key(context,req,config.DEFAULT,60)
    real=algorithms.digest
    monkeypatch.setattr(algorithms,'digest',lambda p:'changed' if str(p).endswith('/gripper2left.stl') else real(p))
    assert algorithms.pool_key(context,req,config.DEFAULT,60)!=before


@pytest.mark.parametrize('link',['missing','openarmx_left_hand'])
def test_missing_stick_cylinder_not_silently_assigned_right_length(draft,link):
    with pytest.raises(ValueError,match='Missing stick'):
        model.stick_length(draft[1],link)
