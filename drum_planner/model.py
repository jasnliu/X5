"""Generate an isolated calibrated collision model; never edit the shared URDF."""
import itertools
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from camera_calibration.core import ROOT, JOINTS, digest
from .common import calibration, atomic_json, DEFAULT_CALIBRATION, arm_side, arm_joints

MOUNT_Z = .015  # Provisional nominal finger attachment plane; physical check REQUIRED.
STICK_RADIUS = .0148 / 2
GRIPPER_LENGTH = .130
STICK_EXPOSED = 11 * .0254

# Left traditional-grip draft, dimensions recovered from gripper2*.stl (mm).
# Both CAD bores are parallel to CAD Y, centered at X=-27.15, Z=12.4.
# Rotate the second half 180 degrees about Z, then translate the halves in X:
# they form ONE sleeve, not two opposed projecting tubes. This draft clocks
# the sleeve along hand +Y: forward at LEFT_CENTER, near the snare at record1.
# The shared nominal mounting plane is retained, but mounting yaw/translation
# are NOT independently measured. In particular, do not claim that the raw
# motor angle is a calibrated conversion to slider displacement.
LEFT_BORE_OFFSET = .02715
LEFT_BORE_HEIGHT = .0124
LEFT_BORE_REAR = -.03075
LEFT_BORE_FRONT = .09575
LEFT_STICK_EXPOSED = 10.25 * .0254
LEFT_STICK_LENGTH = LEFT_BORE_FRONT - LEFT_BORE_REAR + LEFT_STICK_EXPOSED
LEFT_CLOSED_MOTOR_RAW_RAD = float(np.radians(14.16))


def stick_length(root, link):
    """Length of this arm's actual URDF cylinder, including historical models.

    Do not use the right-arm manifest length for a different/custom left tool.
    Native FK/collision already uses this same cylinder and its fixed transform.
    """
    cylinder = root.find(f"link[@name='{link}']/collision/geometry/cylinder")
    if cylinder is None:
        raise ValueError(f'Missing stick collision cylinder: {link}')
    length = float(cylinder.get('length'))
    if not np.isfinite(length) or length <= 0:
        raise ValueError(f'Invalid stick length: {link}')
    return length


def _left_traditional_tool(root, assemblies):
    for suffix, file, x, yaw, front_y in (
            ('left', 'gripper2left.stl', LEFT_BORE_OFFSET, 0., 95.75),
            ('right', 'gripper2right.stl', -LEFT_BORE_OFFSET, np.pi, -95.75)):
        path = ROOT / 'gripper_models' / file
        v = stl_vertices(path).reshape(-1, 3)
        expected_y = [-30.75, 95.75] if front_y > 0 else [-95.75, 30.75]
        if not np.allclose([v.min(0), v.max(0)],
                           [[-27.15, expected_y[0], -12.5], [14.75, expected_y[1], 24.8]], atol=.001):
            raise ValueError('Traditional-grip CAD dimensions changed; review left placement')
        front = v[np.abs(v[:, 1] - front_y) < .001]
        radii = np.linalg.norm(front[:, [0, 2]] - [-27.15, 12.4], axis=1)
        if not len(front) or not np.all(np.minimum(abs(radii - 7.4), abs(radii - 12.4)) < .01):
            raise ValueError('Traditional-grip CAD bore no longer matches 14.8 mm stick')
        link_name = f'openarmx_left_{suffix}_finger'
        link = root.find(f"link[@name='{link_name}']")
        for child in list(link): link.remove(child)
        for kind in ('visual', 'collision'):
            geom = ET.SubElement(ET.SubElement(link, kind), 'geometry')
            ET.SubElement(geom, 'mesh', filename='file://' + str(path), scale='.001 .001 .001')
        joint = next(j for j in root.findall('joint') if j.find('child').get('link') == link_name)
        joint.set('type', 'fixed')
        for tag in ('axis', 'limit', 'mimic'):
            el = joint.find(tag)
            if el is not None: joint.remove(el)
        xyz = [x, 0., MOUNT_Z]; rpy = [0., 0., float(yaw)]
        joint.find('origin').set('xyz', ' '.join(map(str, xyz)))
        joint.find('origin').set('rpy', ' '.join(map(str, rpy)))
        assemblies.append(dict(link=link_name, mesh=str(path), sha256=digest(path), scale=.001,
                               xyz_m=xyz, rpy_rad=rpy, bore_radius_m=STICK_RADIUS))
    stick = ET.SubElement(root, 'link', name='left_drumstick')
    for kind in ('visual', 'collision'):
        geom = ET.SubElement(ET.SubElement(stick, kind), 'geometry')
        ET.SubElement(geom, 'cylinder', radius=str(STICK_RADIUS), length=str(LEFT_STICK_LENGTH))
    center_y = LEFT_BORE_REAR + LEFT_STICK_LENGTH / 2
    joint = ET.SubElement(root, 'joint', name='left_drumstick_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='openarmx_left_hand')
    ET.SubElement(joint, 'child', link='left_drumstick')
    # URDF cylinder +Z points along hand +Y, toward the exposed tip.
    ET.SubElement(joint, 'origin', xyz=f'0 {center_y} {MOUNT_Z + LEFT_BORE_HEIGHT}',
                  rpy=f'{-np.pi/2} 0 0')


def stl_vertices(path):
    data = Path(path).read_bytes()
    n = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + n * 50:
        raise ValueError('Expected binary STL; refusing guessed units/format')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(data, dtype=dtype, count=n, offset=84)['vertices'].copy()


def generate(folder, calibration_path=DEFAULT_CALIBRATION, *, moving_side='right'):
    arm_side(moving_side)
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
    for arm in (('right', 'left') if moving_side == 'left' else ('right',)):
        if arm == 'left':
            _left_traditional_tool(root, assemblies)
            continue
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
            link_name = f'openarmx_{arm}_{suffix}_finger'
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

        stick = ET.SubElement(root, 'link', name=f'{arm}_drumstick')
        length = GRIPPER_LENGTH + STICK_EXPOSED
        for kind in ('visual', 'collision'):
            part = ET.SubElement(stick, kind)
            geom = ET.SubElement(part, 'geometry')
            ET.SubElement(geom, 'cylinder', radius=str(STICK_RADIUS), length=str(length))
        joint = ET.SubElement(root, 'joint', name=f'{arm}_drumstick_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link=f'openarmx_{arm}_hand')
        ET.SubElement(joint, 'child', link=f'{arm}_drumstick')
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
    if moving_side == 'left':
        group = ET.SubElement(srdf, 'group', name='left_arm')
        for name in arm_joints('left'): ET.SubElement(group, 'joint', name=name)
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
    if moving_side == 'left':
        info.update(moving_side='left', left_tool_geometry='traditional perpendicular grip; gripper2 STL pair; draft mounting',
                    left_kinematic_offsets_applied=False,
                    left_tool=dict(stick_link='left_drumstick', stick_length_m=LEFT_STICK_LENGTH,
                        stick_radius_m=STICK_RADIUS, stick_exposed_length_m=LEFT_STICK_EXPOSED,
                        sleeve_length_m=LEFT_BORE_FRONT - LEFT_BORE_REAR,
                        bore_axis_hand=[0., 1., 0.],
                        sleeve_rear_center_hand_m=[0., LEFT_BORE_REAR, MOUNT_Z + LEFT_BORE_HEIGHT],
                        sleeve_front_center_hand_m=[0., LEFT_BORE_FRONT, MOUNT_Z + LEFT_BORE_HEIGHT],
                        tip_hand_m=[0., LEFT_BORE_FRONT + LEFT_STICK_EXPOSED, MOUNT_Z + LEFT_BORE_HEIGHT],
                        rear_stick_protrusion_m=0., closed_motor_raw_rad=LEFT_CLOSED_MOTOR_RAW_RAD,
                        closed_motor_joint_rad=-LEFT_CLOSED_MOTOR_RAW_RAD,
                        closure='fixed CAD halves meeting at bore; no inferred motor-to-slider conversion',
                        mounting_physically_verified=False,
                        mounting_assumption='nominal 15 mm hand mount plane; draft hand +Y clocking selected for forward center / snare goal'))
    atomic_json(folder / 'model_manifest.json', info)
    return info
