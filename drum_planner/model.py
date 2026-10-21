"""Generate an isolated calibrated collision model; never edit the shared URDF."""
import itertools
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from camera_calibration.core import ROOT, JOINTS, digest
from .common import calibration, atomic_json, DEFAULT_CALIBRATION

MOUNT_Z = .015  # Provisional nominal finger attachment plane; physical check REQUIRED.
STICK_RADIUS = .0148 / 2
GRIPPER_LENGTH = .130
STICK_EXPOSED = 11 * .0254


def stl_vertices(path):
    data = Path(path).read_bytes()
    n = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + n * 50:
        raise ValueError('Expected binary STL; refusing guessed units/format')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(data, dtype=dtype, count=n, offset=84)['vertices'].copy()


def generate(folder, calibration_path=DEFAULT_CALIBRATION):
    folder = Path(folder).resolve(); folder.mkdir(parents=True, exist_ok=True)
    cal = calibration(calibration_path)
    root = ET.parse(ROOT / 'model/openarmx.urdf').getroot()
    # q remains in the recorded encoder_to_joint convention. Incorporating the
    # offset into each joint origin gives FK(q + offset) without shifting targets
    # or accidentally applying the existing J5 convention a second time.
    # Position bounds must undergo the SAME change of coordinates:
    #     lower_model <= q_saved + offset <= upper_model.
    # Keeping the old numeric bounds after shifting the origin incorrectly
    # rejects legal lower-end poses and admits some illegal upper-end poses.
    # This translates the interval; it does not widen it, clip measurements,
    # estimate new hardware stops, or change any motor/encoder zero.
    limit_mapping = {}
    for name, offset in cal['kinematic_correction']['offsets_rad'].items():
        joint = root.find(f"joint[@name='{name}']")
        origin = joint.find('origin')
        R = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        corrected = R * Rotation.from_rotvec(axis / np.linalg.norm(axis) * offset)
        origin.set('rpy', ' '.join(f'{x:.17g}' for x in corrected.as_euler('xyz')))
        limit = joint.find('limit')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        mapped_lower, mapped_upper = lower - offset, upper - offset
        limit.set('lower', f'{mapped_lower:.17g}')
        limit.set('upper', f'{mapped_upper:.17g}')
        limit_mapping[name] = dict(offset_rad=offset, original_model_lower_rad=lower,
                                  original_model_upper_rad=upper,
                                  saved_coordinate_lower_rad=mapped_lower,
                                  saved_coordinate_upper_rad=mapped_upper)

    assemblies = []
    # The two inner semicircles fit radius 7.4 mm and centers (+/-14.75,-23.35)
    # in STL mm coordinates. Opposing halves are translated so these axes coincide.
    # The yaw/mount plane remain provisional: hardware execution is unavailable.
    for suffix, file, cx in [('left', 'gripperLeft.stl', -14.75), ('right', 'gripperRight.stl', 14.75)]:
        path = ROOT / 'gripper_models' / file
        v = stl_vertices(path)
        if not np.allclose([v[:, :, 2].min(), v[:, :, 2].max()], [-12.5, 130.], atol=.001):
            raise ValueError('CAD dimensions changed; review gripper placement')
        top = v.reshape(-1, 3); top = top[top[:, 2] > 129.99]
        radii = np.linalg.norm(top[:, :2] - [cx, -23.35], axis=1)
        if not np.all(np.minimum(abs(radii - 7.4), abs(radii - 12.4)) < .01):
            raise ValueError('CAD bore no longer matches the measured 14.8 mm stick')
        link_name = f'openarmx_right_{suffix}_finger'
        link = root.find(f"link[@name='{link_name}']")
        for child in list(link):
            link.remove(child)
        for kind in ('visual', 'collision'):
            part = ET.SubElement(link, kind)
            geom = ET.SubElement(part, 'geometry')
            ET.SubElement(geom, 'mesh', filename='file://' + str(path), scale='.001 .001 .001')
        joint = next(j for j in root.findall('joint') if j.find('child').get('link') == link_name)
        joint.set('type', 'fixed')
        for tag in ('axis', 'limit', 'mimic'):
            el = joint.find(tag)
            if el is not None: joint.remove(el)
        xyz = [-.02335, -cx * .001, MOUNT_Z]
        joint.find('origin').set('xyz', ' '.join(map(str, xyz)))
        joint.find('origin').set('rpy', '0 0 1.5707963267948966')
        assemblies.append(dict(link=link_name, mesh=str(path), sha256=digest(path), scale=.001,
                               xyz_m=xyz, rpy_rad=[0., 0., np.pi/2], bore_radius_m=STICK_RADIUS))

    stick = ET.SubElement(root, 'link', name='right_drumstick')
    length = GRIPPER_LENGTH + STICK_EXPOSED
    for kind in ('visual', 'collision'):
        part = ET.SubElement(stick, kind)
        geom = ET.SubElement(part, 'geometry')
        ET.SubElement(geom, 'cylinder', radius=str(STICK_RADIUS), length=str(length))
    joint = ET.SubElement(root, 'joint', name='right_drumstick_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='openarmx_right_hand')
    ET.SubElement(joint, 'child', link='right_drumstick')
    ET.SubElement(joint, 'origin', xyz=f'0 0 {MOUNT_Z + length/2}', rpy='0 0 0')
    # Package lookup is unnecessary; no vendor hardware launches/plugins are used.
    for mesh in root.findall('.//mesh'):
        name = mesh.get('filename')
        if name.startswith('package://openarmx_description/'):
            path = ROOT / 'vendor/openarmx_description' / name.split('package://openarmx_description/')[1]
            if not path.is_file(): raise ValueError(f'Missing mesh {path}')
            mesh.set('filename', 'file://' + str(path))
    if root.find('ros2_control') is not None or root.find('.//transmission') is not None:
        raise ValueError('Hardware tags are forbidden in the planner model')
    ET.indent(root)
    ET.ElementTree(root).write(folder / 'robot.urdf', encoding='unicode', xml_declaration=True)

    # Do NOT import the vendor's broad "Never" exclusions: custom tools invalidate
    # them. Exempt only immediate assembly neighbors and rigidly connected pieces.
    srdf = ET.Element('robot', name=root.get('name'))
    group = ET.SubElement(srdf, 'group', name='right_arm')
    for name in JOINTS: ET.SubElement(group, 'joint', name=name)
    groups = [{link.get('name')} for link in root.findall('link')]
    allowed = set()
    for joint in root.findall('joint'):
        a, b = joint.find('parent').get('link'), joint.find('child').get('link')
        allowed.add(tuple(sorted((a, b))))
        if joint.get('type') == 'fixed':
            matches = [g for g in groups if a in g or b in g]
            merged = set().union(*matches)
            groups = [g for g in groups if g not in matches] + [merged]
    for group in groups:
        allowed.update(tuple(sorted(pair)) for pair in itertools.combinations(group, 2))
    allowed.add(('openarmx_left_left_finger', 'openarmx_left_right_finger'))
    for a, b in sorted(allowed):
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='Adjacent or rigid assembly')
    ET.indent(srdf)
    ET.ElementTree(srdf).write(folder / 'robot.srdf', encoding='unicode', xml_declaration=True)
    info = dict(schema='x5-planner-model-v1', original_model_sha256=cal['model_sha256'],
                calibrated_urdf_sha256=digest(folder/'robot.urdf'), srdf_sha256=digest(folder/'robot.srdf'),
                calibration_sha256=digest(calibration_path), joint_coordinates='encoder_to_joint; corrections embedded in joint origins',
                joint_limits_coordinate_rule='q_model = q_saved + offset; saved limits = original model limits - offset',
                joint_limit_mapping=limit_mapping, joint_limit_intervals_widened=False,
                physical_joint_stops_independently_verified=False,
                gripper_motor_joint_closed_rad=float(np.radians(7)), gripper_joints_fixed=True,
                gripper_assemblies=assemblies, stick_radius_m=STICK_RADIUS,
                stick_length_from_mount_m=length, stick_exposed_length_m=STICK_EXPOSED,
                mounting_face_hand_z_m=MOUNT_Z, tool_geometry_physically_verified=False,
                left_stick_geometry_verified=False, hardware_execution_available=False,
                warning='STL bore assembly is checked mathematically. Mount plane/yaw and left stick require physical verification. '
                        'Mesh mounting is provisional, not a certified physical model.')
    atomic_json(folder / 'model_manifest.json', info)
    return info
