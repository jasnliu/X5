"""Offline rendering regression: complete visual surfaces, not triangle islands."""
import builtins
from pathlib import Path
import socket
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from drum_planner import common, depth
from drum_planner.model import stl_vertices
from drum_planner.trajectory_view import PlanDisplay, RobotArtists
from drum_planner.triangle_artist import TriangleCollection
from drum_planner.visual_mesh import (visual_surfaces, _collada_parts, cylinder_triangles,
                                      simplify_surface, origin_transform)


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original = socket.socket
    def guarded(family=socket.AF_INET, *args, **kwargs):
        if family == socket.AF_CAN: raise AssertionError('Rendering must not open CAN')
        return original(family, *args, **kwargs)
    def denied(*args, **kwargs): raise AssertionError('Rendering must not open a camera')
    monkeypatch.setattr(socket, 'socket', guarded)
    monkeypatch.setattr(depth, 'DepthCamera', denied)


@pytest.fixture(scope='module')
def plan(): return PlanDisplay(common.ROOT/'planner_runs/software_demo/plan.json')


def edges(triangles):
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    edges = np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    return vertices, faces, unique, counts


def test_regular_links_keep_every_original_visual_face_and_material(plan):
    surfaces = {n: (t, local, colors) for n, t, local, colors in visual_surfaces(plan.root)}
    assert len(surfaces) == len(plan.root.findall('.//visual')) == 24
    for link in plan.root.findall('link'):
        name = link.get('name')
        if '_link' not in name or name not in surfaces: continue
        visual = link.find('visual'); mesh = visual.find('geometry/mesh')
        parts = _collada_parts(mesh.get('filename').removeprefix('file://'))
        scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        expected = np.concatenate([t for t, _ in parts]) * scale
        if np.prod(scale) < 0: expected = expected[:, [0, 2, 1]]
        triangles, local, colors = surfaces[name]
        assert np.array_equal(triangles, expected)  # No dropped triangles, including mirrored left links.
        assert np.allclose(local, origin_transform(visual.find('origin')))
        assert np.array_equal(colors, np.concatenate([np.tile(c, (len(t), 1)) for t, c in parts]))


def test_right_custom_halves_and_solid_closed_stick_are_exact(plan):
    surfaces = {n: t for n, t, _, _ in visual_surfaces(plan.root)}
    for side in ('left', 'right'):
        mesh = plan.root.find(f"link[@name='openarmx_right_{side}_finger']/visual/geometry/mesh")
        triangles = surfaces[f'openarmx_right_{side}_finger']
        assert np.array_equal(triangles, stl_vertices(mesh.get('filename').removeprefix('file://')).astype(float)*.001)
        assert len(triangles) == 3174
    stick = surfaces['right_drumstick']
    assert np.ptp(stick[:, :, 2]) == pytest.approx(plan.obj['model']['stick_length_from_mount_m'])
    assert np.max(np.linalg.norm(stick[:, :, :2], axis=2)) == pytest.approx(.0074)
    assert np.all(edges(stick)[3] == 2)  # Every edge has two faces: ends are capped, no holes.
    assert not np.array_equal(surfaces['openarmx_left_left_finger'], surfaces['openarmx_right_left_finger'])


def test_collada_scene_units_accessor_material_and_urdf_origin(tmp_path):
    path = tmp_path/'fixture.dae'
    path.write_text('''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
      <asset><unit meter=".001"/><up_axis>Z_UP</up_axis></asset>
      <library_effects><effect id="effect"><profile_COMMON><technique><lambert>
        <diffuse><color>0.2 0.6 0.8 1</color></diffuse>
      </lambert></technique></profile_COMMON></effect></library_effects>
      <library_materials><material id="mat"><instance_effect url="#effect"/></material></library_materials>
      <library_geometries><geometry id="geo"><mesh>
        <source id="positions"><float_array id="points">99 0 0 0 99 10 0 0 99 0 10 0 99</float_array>
          <technique_common><accessor source="#points" count="3" stride="4" offset="1"/></technique_common>
        </source><vertices id="verts"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles count="1" material="alias"><input semantic="VERTEX" source="#verts" offset="0"/>
          <input semantic="NORMAL" source="#ignored" offset="2"/><p>0 8 0 1 8 0 2 8 0</p></triangles>
      </mesh></geometry></library_geometries>
      <library_visual_scenes><visual_scene id="scene"><node><translate>100 0 0</translate><node>
        <matrix>1 0 0 0 0 1 0 20 0 0 1 0 0 0 0 1</matrix><instance_geometry url="#geo">
          <bind_material><technique_common><instance_material symbol="alias" target="#mat"/></technique_common></bind_material>
        </instance_geometry></node></node></visual_scene></library_visual_scenes>
      <scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')
    root = ET.fromstring(f'''<robot><link name="fixture"><visual><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
      <geometry><mesh filename="file://{path}" scale="1 -1 1"/></geometry></visual>
      <collision><geometry><mesh filename="/must/not/load/collision.stl"/></geometry></collision></link></robot>''')
    name, triangles, local, colors = next(visual_surfaces(root))
    assert name == 'fixture'
    assert np.allclose(triangles, [[[.1, -.02, 0], [.1, -.03, 0], [.11, -.02, 0]]])
    assert np.allclose(local[:3, 3], [1, 2, 3])
    assert np.allclose(local[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    assert np.allclose(colors, [[.2, .6, .8, 1]])
    root[0][0].append(ET.fromstring('<material><color rgba=".9 .1 .2 .5"/></material>'))
    assert np.allclose(next(visual_surfaces(root))[3], [[.9, .1, .2, 1]])
    path.write_text(path.read_text().replace('Z_UP', 'Y_UP'))
    with pytest.raises(ValueError, match='Z_UP'): list(visual_surfaces(root))


def test_render_lod_preserves_surface_topology():
    pytest.importorskip('vtkmodules')
    original = cylinder_triangles(.02, .1, segments=600)
    reduced = simplify_surface(original, 150)
    assert len(reduced) < len(original)
    for triangles in (original, reduced):
        v, f, e, counts = edges(triangles)
        assert np.all(counts == 2)  # Would fail with the old [::N] triangle sampling.
        assert len(v)-len(e)+len(f) == 2
    assert np.allclose(np.ptp(reduced.reshape(-1, 3), axis=0), [.04, .04, .1], atol=.0005)


def test_no_vtk_draws_full_mesh_instead_of_dropping_faces(monkeypatch):
    original_import = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name.startswith('vtkmodules'): raise ImportError('No optional LOD library')
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    mesh = cylinder_triangles(.02, .1, segments=80)
    assert simplify_surface(mesh, 20) is mesh


@pytest.mark.parametrize('angles', [(25, -55), (80, 120), (-15, 45)])
def test_vectorized_projection_matches_stock_matplotlib(angles):
    fig = Figure(); FigureCanvasAgg(fig); ax = fig.add_subplot(projection='3d'); ax.view_init(*angles)
    rng = np.random.default_rng(42); triangles = rng.random((75, 3, 3)); colors = rng.random((75, 4)); colors[:, 3] = 1
    artists = [cls(triangles, facecolors=colors, edgecolor='none', alpha=1., antialiaseds=False)
               for cls in (Poly3DCollection, TriangleCollection)]
    for a in artists: ax.add_collection3d(a)
    ax.M = ax.get_proj()
    assert artists[0].do_3d_projection() == pytest.approx(artists[1].do_3d_projection())
    assert np.array_equal(artists[0].get_facecolor(), artists[1].get_facecolor())
    for p, q in zip(artists[0].get_paths(), artists[1].get_paths()): assert np.allclose(p.vertices, q.vertices)
    fig.canvas.draw()
    artists[1].set_verts([])
    assert np.isnan(artists[1].do_3d_projection())


def test_solid_robot_tracks_plan_and_visibility_without_skeleton(plan):
    fig = Figure(); FigureCanvasAgg(fig); ax = fig.add_subplot(projection='3d')
    robot = RobotArtists(ax, plan)
    assert len(ax.lines) == 0
    assert all(poly.get_alpha() == 1. for _, _, _, poly in robot.mesh_entries)
    goal = plan.obj['outward'][-1]['time_s']
    for t in (0., goal, goal+1, plan.duration):
        robot.draw(t)
        assert np.array_equal(robot.last_positions, plan.positions(t))
        frame = plan.frames(t)['right_drumstick']
        assert np.allclose(robot.last_tip, frame[:3, 3]+frame[:3, 2]*plan.obj['model']['stick_length_from_mount_m']/2)
    for visible in (False, True):
        robot.set_visible(visible)
        assert all(poly.get_visible() == visible for _, _, _, poly in robot.mesh_entries)
    fig.canvas.draw()
