"""Display-only URDF visual surfaces. Never used for collision checking.

Read the bundled Blender COLLADA visuals (including materials and scene nodes),
the custom STL fingers, and URDF primitives. Do not subsample triangle lists:
that removes faces rather than simplifying a surface and leaves a broken model.
"""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .model import stl_vertices

DEFAULT_COLOR = (.65, .67, .70, 1.)
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def origin_transform(origin):
    transform = np.eye(4)
    if origin is not None:
        transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return transform


def _collada_parts(path):
    """The bundled assets use indexed triangles, solid colors and Z-up metres.

    Explicitly reject unsupported surface formats rather than silently showing
    half a robot. Cameras/lights and line annotations are not robot surfaces.
    """
    root = ET.parse(path).getroot()
    if root.findtext('c:asset/c:up_axis', default='Y_UP', namespaces=NS) != 'Z_UP':
        raise ValueError(f'Expected a Z_UP robot visual: {path}')
    unit = root.find('c:asset/c:unit', NS)
    metres = float(unit.get('meter', '1')) if unit is not None else 1.
    elements = {el.get('id'): el for el in root.iter() if el.get('id')}

    def ref(url):
        if not url.startswith('#') or url[1:] not in elements:
            raise ValueError(f'Unsupported COLLADA reference {url} in {path}')
        return elements[url[1:]]

    def material_color(target):
        material = ref(target)
        effect = ref(material.find('c:instance_effect', NS).get('url'))
        color = effect.find('.//c:diffuse/c:color', NS)
        if color is None:
            raise ValueError(f'Expected a solid diffuse robot material: {path}')
        return np.fromstring(color.text, sep=' ')

    def positions(vertices):
        source_ref = next(i.get('source') for i in vertices.findall('c:input', NS)
                          if i.get('semantic') == 'POSITION')
        source = ref(source_ref)
        accessor = source.find('c:technique_common/c:accessor', NS)
        values = np.fromstring(ref(accessor.get('source')).text, sep=' ')
        offset = int(accessor.get('offset', '0')); stride = int(accessor.get('stride', '1'))
        count = int(accessor.get('count'))
        return values[offset:offset + count * stride].reshape(count, stride)[:, :3]

    def node_parts(node, parent):
        transform = parent.copy()
        for child in node:
            tag = child.tag.split('}')[-1]
            if tag not in ('matrix', 'translate', 'rotate', 'scale'):
                continue
            values = np.fromstring(child.text, sep=' '); local = np.eye(4)
            if tag == 'matrix': local = values.reshape(4, 4)
            elif tag == 'translate': local[:3, 3] = values
            elif tag == 'scale': local[:3, :3] = np.diag(values)
            else:
                local[:3, :3] = Rotation.from_rotvec(
                    values[:3] / np.linalg.norm(values[:3]) * np.deg2rad(values[3])).as_matrix()
            transform = transform @ local
        for instance in node.findall('c:instance_geometry', NS):
            mesh = ref(instance.get('url')).find('c:mesh', NS)
            if mesh is None: raise ValueError(f'Expected a triangle robot mesh: {path}')
            bindings = {m.get('symbol'): material_color(m.get('target'))
                        for m in instance.findall('.//c:instance_material', NS)}
            for primitive in mesh:
                tag = primitive.tag.split('}')[-1]
                if tag in ('source', 'vertices', 'lines', 'extra'): continue
                if tag != 'triangles':
                    raise ValueError(f'Unsupported robot visual primitive {tag}: {path}')
                inputs = primitive.findall('c:input', NS)
                stride = 1 + max(int(i.get('offset', '0')) for i in inputs)
                vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
                indices = np.fromstring(primitive.findtext('c:p', namespaces=NS), dtype=int, sep=' ')
                indices = indices.reshape(-1, stride)[:, int(vertex.get('offset', '0'))]
                triangles = positions(ref(vertex.get('source')))[indices].reshape(-1, 3, 3)
                if len(triangles) != int(primitive.get('count')):
                    raise ValueError(f'Robot visual triangle count mismatch: {path}')
                triangles = (triangles @ transform[:3, :3].T + transform[:3, 3]) * metres
                color = bindings.get(primitive.get('material'), DEFAULT_COLOR)
                yield triangles, np.asarray(color)
        for child in node.findall('c:node', NS): yield from node_parts(child, transform)
        for instance in node.findall('c:instance_node', NS):
            yield from node_parts(ref(instance.get('url')), transform)

    scene = ref(root.find('c:scene/c:instance_visual_scene', NS).get('url'))
    parts = [part for node in scene.findall('c:node', NS) for part in node_parts(node, np.eye(4))]
    if not parts: raise ValueError(f'No robot visual surfaces in {path}')
    return parts


def simplify_surface(triangles, target_faces):
    """Render-only edge collapse, retaining topology and material boundaries.

    VTK is optional: without it draw *all* original faces (slower, never holes).
    A target is not a hard limit: keep extra faces when topology, boundaries or
    the 0.25 mm display-error setting prevent further simplification.
    """
    if target_faces is None or len(triangles) <= target_faces: return triangles
    try:
        from vtkmodules.vtkCommonCore import vtkPoints
        from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
        from vtkmodules.vtkFiltersCore import vtkDecimatePro
        from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
    except ImportError:
        return triangles
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    points = vtkPoints(); points.SetData(numpy_to_vtk(vertices, deep=True))
    cells = np.column_stack((np.full(len(triangles), 3), indices.reshape(-1, 3))).astype(np.int64)
    polys = vtkCellArray(); polys.SetCells(len(triangles), numpy_to_vtkIdTypeArray(cells.ravel(), deep=True))
    mesh = vtkPolyData(); mesh.SetPoints(points); mesh.SetPolys(polys)
    decimate = vtkDecimatePro(); decimate.SetInputData(mesh)
    decimate.SetTargetReduction(1. - target_faces / len(triangles))
    decimate.PreserveTopologyOn(); decimate.SplittingOff(); decimate.BoundaryVertexDeletionOff()
    decimate.SetFeatureAngle(90.)
    decimate.AccumulateErrorOn(); decimate.SetErrorIsAbsolute(1); decimate.SetAbsoluteError(.00025)
    decimate.Update(); output = decimate.GetOutput()
    if output.GetNumberOfPolys() == 0: return triangles
    faces = vtk_to_numpy(output.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    return vtk_to_numpy(output.GetPoints().GetData())[faces].copy()


@lru_cache(maxsize=64)
def _mesh_parts(path, mtime_ns, size, scale, target_faces):
    # The stat values invalidate the cache if the user replaces a CAD asset.
    if Path(path).suffix.lower() == '.dae': parts = _collada_parts(path)
    elif Path(path).suffix.lower() == '.stl': parts = [(stl_vertices(path).astype(float), DEFAULT_COLOR)]
    else: raise ValueError(f'Unsupported robot visual file: {path}')
    total = sum(len(t) for t, _ in parts)
    out = []
    for triangles, color in parts:
        # Some vendor DAE assets also rely on the URDF's .001 scale. Apply it
        # BEFORE any metric display simplification, and account for mirroring.
        triangles = triangles * scale
        if np.prod(scale) < 0: triangles = triangles[:, [0, 2, 1]]
        # Keep the body, arm links and custom fingers exactly as supplied.
        # Only the extremely dense vendor hand/finger CAD needs render LOD.
        if Path(path).suffix.lower() == '.dae' and total > 20000 and target_faces:
            triangles = simplify_surface(triangles, max(64, round(target_faces * len(triangles) / total)))
        triangles.setflags(write=False)
        out.append((triangles, np.asarray(color)))
    return out


def cylinder_triangles(radius, length, segments=40):
    angles = np.arange(segments) * 2 * np.pi / segments
    bottom = np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.full(segments, -length / 2)))
    top = bottom.copy(); top[:, 2] = length / 2
    nxt = np.roll(np.arange(segments), -1)
    return np.concatenate((np.stack((bottom, bottom[nxt], top[nxt]), axis=1),
                           np.stack((bottom, top[nxt], top), axis=1),
                           np.stack((np.tile([0, 0, -length / 2], (segments, 1)), bottom[nxt], bottom), axis=1),
                           np.stack((np.tile([0, 0, length / 2], (segments, 1)), top, top[nxt]), axis=1)))


def visual_surfaces(root, target_faces=1200):
    """Yield (link name, complete triangles, visual-origin transform, colors)."""
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            geometry = visual.find('geometry'); mesh = geometry.find('mesh')
            name = link.get('name')
            if mesh is not None:
                path = Path(mesh.get('filename').removeprefix('file://')).resolve()
                scale = tuple(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
                stat = path.stat(); parts = _mesh_parts(str(path), stat.st_mtime_ns, stat.st_size, scale, target_faces)
            elif geometry.find('cylinder') is not None:
                cylinder = geometry.find('cylinder')
                parts = [(cylinder_triangles(float(cylinder.get('radius')), float(cylinder.get('length'))),
                          (.78, .57, .32, 1.) if name in ('right_drumstick','left_drumstick') else DEFAULT_COLOR)]
            else:
                raise ValueError(f'Unsupported robot visual geometry: {name}')
            material = visual.find('material')
            if material is not None and material.find('color') is None:
                material = root.find(f"material[@name='{material.get('name')}']")
            override = material.find('color') if material is not None else None
            triangles = np.concatenate([t for t, _ in parts])
            colors = np.concatenate([np.tile(color, (len(t), 1)) for t, color in parts])
            if override is not None: colors[:] = np.fromstring(override.get('rgba'), sep=' ')
            # Robot is solid, not the old translucent collision/skeleton proxy.
            colors[:, 3] = 1.
            yield name, triangles, origin_transform(visual.find('origin')), colors
