"""Real OpenGL rendering with synthetic/saved data; never camera or CAN access."""
import json
import socket

import numpy as np
import pytest
from vtkmodules.util.numpy_support import vtk_to_numpy

from drum_planner import common,depth,snapshot_preview
from drum_planner.trajectory_view import PlanDisplay
from drum_planner.visual_mesh import visual_surfaces


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    old=socket.socket
    def guarded(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:raise AssertionError('Renderer must not open CAN')
        return old(family,*a,**kw)
    def denied(*a,**kw):raise AssertionError('Renderer must not open a camera')
    monkeypatch.setattr(socket,'socket',guarded)
    monkeypatch.setattr(depth,'DepthCamera',denied);monkeypatch.setattr(snapshot_preview,'DepthCamera',denied)
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')


@pytest.fixture(scope='module')
def plan():return PlanDisplay(common.ROOT/'planner_runs/software_demo/plan.json')


@pytest.fixture
def viewport():
    from PyQt5 import QtWidgets
    from drum_planner.vtk_view import VtkViewport
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    view=VtkViewport();view.resize(850,650);view.show();app.processEvents()
    try:yield view
    finally:view.shutdown();view.close();app.processEvents()


def test_robot_keeps_exact_visual_geometry_and_materials_on_gpu(viewport,plan):
    from drum_planner.vtk_view import RobotActors
    robot=RobotActors(viewport,plan);original=list(visual_surfaces(plan.root))
    assert len(robot.mesh_entries)==len(original)==24
    for (name,triangles,local,layer),(expected,faces,origin,colors) in zip(robot.mesh_entries,original):
        assert name==expected and np.array_equal(triangles,faces) and np.array_equal(local,origin)
        assert np.array_equal(vtk_to_numpy(layer.polydata.GetPoints().GetData()),faces.reshape(-1,3))
        assert layer.polydata.GetNumberOfPolys()==len(faces)
        assert np.array_equal(vtk_to_numpy(layer.polydata.GetCellData().GetScalars()),np.uint8(np.rint(colors*255)))
        assert layer.actor.GetProperty().GetOpacity()==1 and layer.actor.GetProperty().GetRepresentation()==2
        assert not layer.actor.GetProperty().GetBackfaceCulling()
    viewport.fit(robot.focus_bounds());robot.draw(0);viewport.render()
    assert viewport.report()['engine']=='VTK/OpenGL' and not viewport.errors


def test_playback_updates_only_actor_matrices_not_meshes(viewport,plan):
    from drum_planner.vtk_view import RobotActors
    robot=RobotActors(viewport,plan)
    layers=[layer for _,_,_,layer in robot.mesh_entries]
    addresses=[layer.polydata.GetPoints().GetAddressAsString('') for layer in layers]
    goal=plan.obj['outward'][-1]['time_s']
    for t in (0,goal,goal+1,plan.duration):
        robot.draw(t);frames=plan.frames(t)
        assert np.array_equal(robot.last_positions,plan.positions(t))
        for name,triangles,local,layer in robot.mesh_entries:
            matrix=layer.actor.GetUserMatrix()
            assert np.allclose([[matrix.GetElement(r,c) for c in range(4)] for r in range(4)],frames[name]@local)
        T=frames['right_drumstick']
        assert np.allclose(robot.last_tip,T[:3,3]+T[:3,2]*plan.obj['model']['stick_length_from_mount_m']/2)
    assert addresses==[layer.polydata.GetPoints().GetAddressAsString('') for layer in layers]
    assert all(layer.geometry_uploads==layer.color_uploads==1 for layer in layers)
    for name,_,_,layer in robot.mesh_entries:
        if name.startswith('openarmx_left_'):assert layer.transform_updates==1
    robot.set_visible(False);assert not any(layer.get_visible() for layer in layers)
    robot.set_visible(True);assert all(layer.get_visible() for layer in layers)


def sample_scene():
    raw=np.full((40,48),1000,dtype=np.uint16)
    return snapshot_preview.prepare_scene(dict(sequence=1,received_monotonic=0.,value_scale_m=.001),
        raw,{'depth':{'K':[[500,0,20],[0,500,16],[0,0,1]]}},np.eye(4))


def test_rotation_zoom_color_changes_and_refit_never_rebuild_frozen_mesh(viewport):
    packet,mesh=sample_scene();before=(mesh.vertices.tobytes(),mesh.faces.tobytes())
    viewport.set_surface(mesh,mesh.camera_depth_m,False)
    point_address=viewport.surface.polydata.GetPoints().GetAddressAsString('')
    viewport.fit(np.array([[-.1,-.1,.8],[.1,.1,1.2]]))
    assert viewport.surface.mapper.GetNumberOfClippingPlanes()==6
    for a in (-55,0,120):viewport.set_angles(30,a);viewport.render()
    viewport.zoom(1.1)
    viewport.set_surface(mesh,mesh.camera_depth_m,False)
    assert viewport.surface.color_uploads==1
    viewport.set_color_range(.8,1.2,'World height (m)')
    viewport.set_surface(mesh,mesh.vertices[:,2],True)
    viewport.fit(np.array([[-1,-1,0],[1,1,2]]));viewport.render()
    assert viewport.surface.geometry_uploads==1 and viewport.surface.color_uploads==2
    assert viewport.surface.polydata.GetNumberOfPolys()==len(mesh.faces)
    assert point_address==viewport.surface.polydata.GetPoints().GetAddressAsString('')
    assert before==(mesh.vertices.tobytes(),mesh.faces.tobytes())
    assert viewport.report()['surface_triangles']==len(mesh.faces)


@pytest.mark.parametrize('near_first',[False,True])
def test_real_depth_buffer_occludes_far_triangle_regardless_of_actor_order(viewport,near_first):
    from drum_planner.vtk_view import MeshLayer
    viewport.axes.SetVisibility(False);viewport.colorbar.SetVisibility(False)
    tri=np.array([[-.7,-.7,0],[.7,-.7,0],[0,.7,0]])
    layers=[]
    for near in ([True,False] if near_first else [False,True]):
        layer=MeshLayer(viewport);layer.set_geometry(tri+[0,0,.3 if near else 0],[[0,1,2]])
        layer.set_colors([[0,1,0,1] if near else [1,0,0,1]]);layers.append(layer)
    c=viewport.camera;c.SetPosition(0,0,3);c.SetFocalPoint(0,0,0);c.SetViewUp(0,1,0)
    viewport.render();image=viewport.capture_image()
    color=image.pixelColor(image.width()//2,image.height()//2)
    assert color.green()>240 and color.red()<10
    assert not viewport.errors


def test_native_framebuffer_can_be_saved_with_controls(viewport,tmp_path):
    from drum_planner.vtk_view import MeshLayer
    layer=MeshLayer(viewport)
    layer.set_geometry([[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]]);layer.set_colors([[1,0,0,1]])
    viewport.fit(np.array([[-.1,-.1,-.1],[1.1,1.1,.1]]));viewport.set_title('Saved framebuffer test')
    viewport.render();viewport.save_window(viewport,tmp_path/'view.png')
    assert (tmp_path/'view.png').stat().st_size>1000
    report=viewport.report();assert report['capabilities'] and not report['render_errors']
    viewport.shutdown();assert viewport.report()['capabilities']==report['capabilities']


def test_empty_cloud_and_render_errors_are_explicit(viewport):
    viewport.set_points(np.empty((0,3)),np.empty(0));viewport.render()
    assert viewport.cloud.polydata.GetNumberOfPoints()==0
    viewport.errors.append('injected render error')
    with pytest.raises(RuntimeError,match='injected render error'):viewport.render()


def test_no_camera_playback_uses_vtk_and_preserves_controls(plan,tmp_path):
    from drum_planner.offline_view import OfflinePlaybackView
    view=OfflinePlaybackView(plan,autoplay=True)
    try:
        assert view.clock.playing and view.play_button.isEnabled()
        assert view.worker.submissions==0 and view.viewport.report()['engine']=='VTK/OpenGL'
        assert 'NO CAMERA' in view.banner.text() and not view.freeze_button.isEnabled()
        goal=plan.obj['outward'][-1]['time_s'];view.seek(goal)
        assert np.array_equal(view.robot.last_positions,plan.req['goal'])
        view.seek(plan.duration);assert np.array_equal(view.robot.last_positions,plan.req['start'])
        view.folder=tmp_path;view.save_view();assert (tmp_path/'preview_view.png').is_file()
        view.toggle_freeze();assert not view.frozen
        view.window.close();assert view.poll()==ord('q')
    finally:view.close()


def test_blocked_offline_plan_remains_unplayable(plan,tmp_path):
    from drum_planner.offline_view import OfflinePlaybackView
    obj=json.loads(plan.path.read_text());obj['status']='blocked_start';obj.pop('cycle')
    (tmp_path/'plan.json').write_text(json.dumps(obj))
    (tmp_path/'backend_request.json').write_text(json.dumps(plan.req))
    view=OfflinePlaybackView(PlanDisplay(tmp_path/'plan.json'),autoplay=True)
    try:
        assert not view.clock.playing and not view.play_button.isEnabled()
        assert view.clock.position==0 and view.worker.submissions==0
    finally:view.close()
