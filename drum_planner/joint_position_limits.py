"""Local measured-coordinate operating bounds, NOT vendor geometry or drive settings.

The owner reports a right J2 mechanical stop near -1 degree in start_beat's
encoder_to_joint display. Use 0 degrees as the interior operating floor, with
an explicitly checked transition from/to an acquired rest inside that margin.
Apply AFTER kinematic coordinate conversion: do not shift this measured bound
by the camera calibration offset. No encoder values/endpoints are clipped.
"""
import math
import xml.etree.ElementTree as ET

RIGHT_J2 = 'openarmx_right_joint2'
RIGHT_J2_OBSERVED_STOP_DEG = -1.
RIGHT_J2_LOWER_DEG = 0.
REST_MODEL_ROUNDOFF_RAD = 1e-12  # 5.7e-11 degrees; polynomial endpoint arithmetic only


def policy():
    return dict(schema='x5-right-j2-operating-bound-v2',joint=RIGHT_J2,
        coordinate_convention='encoder_to_joint',lower_deg=RIGHT_J2_LOWER_DEG,
        observed_approximate_stop_deg=RIGHT_J2_OBSERVED_STOP_DEG,
        source='Operator-reported mechanical stop; provisional 1-degree operating clearance',
        independently_hardware_verified=False,other_bounds_changed=False,
        rest_model_roundoff_rad=REST_MODEL_ROUNDOFF_RAD,
        rest_transition='Exact acquired rest above reported stop; monotone departure/return below operating floor')


def command_lower(reference=None):
    """The interior operating margin is not a mechanical limit on a real rest.

    Do not project feedback to zero. Admit only the actual reference pose, and
    only above the reported stop. rest_paths verifies the entire transition.
    """
    floor=math.radians(RIGHT_J2_LOWER_DEG)
    if reference is None:return floor
    if len(reference)!=7 or not all(math.isfinite(float(x)) for x in reference):
        raise ValueError('Expected seven finite acquired right-arm joints')
    q=float(reference[1])
    if q<=math.radians(RIGHT_J2_OBSERVED_STOP_DEG):
        raise ValueError(f'Acquired right J2 is {math.degrees(q):.6f} degrees, at/below the reported '
                         f'{RIGHT_J2_OBSERVED_STOP_DEG:g}-degree stop; not clipped. Correct the physical rest and Calculate again.')
    return min(floor,q)


def model_lower(reference=None):
    lower=command_lower(reference)
    if lower<math.radians(RIGHT_J2_LOWER_DEG):
        # Native bounds comparisons are exact. PPoly endpoint evaluation can
        # differ from the stored rest by a few ULPs; retain the measured pose
        # and coefficients, never clip them. This is NOT tracking clearance.
        return max(math.nextafter(math.radians(RIGHT_J2_OBSERVED_STOP_DEG),math.inf),
                   lower-REST_MODEL_ROUNDOFF_RAD)
    return lower


def apply(root, mapping, side, reference=None):
    """Narrow only selected RIGHT J2; all geometry/other intervals stay exact."""
    if side != 'right':return
    limit=root.find(f"joint[@name='{RIGHT_J2}']/limit")
    previous=float(limit.get('lower'));upper=float(limit.get('upper'))
    lower=max(previous,model_lower(reference))
    if not math.isfinite(lower) or not math.isfinite(upper) or lower>=upper:
        raise ValueError('Invalid right J2 operating interval')
    limit.set('lower',f'{lower:.17g}')
    # Retain the nominal calibration mapping as provenance; consumers reading
    # saved_coordinate_lower_rad must get the actual generated URDF bound.
    entry=mapping[RIGHT_J2]
    entry['nominal_saved_coordinate_lower_rad']=previous
    entry['saved_coordinate_lower_rad']=lower
    entry['position_limit_policy']=policy()
    entry['rest_reference_rad']=reference


def require_current_model(manifest, urdf, side):
    """New physical preparation/Start needs recalculation, not a legacy proof.

    Deliberately NOT called by legacy plan loading/prefix recovery: changing
    software must not erase an already-held arm's recorded recovery route.
    """
    if side!='right':return
    if manifest.get('joint_position_policy')!=policy():
        raise ValueError('Right J2 operating limit changed to 0 degrees; Calculate again before Start. Existing recovery data are retained.')
    limit=ET.parse(urdf).getroot().find(f"joint[@name='{RIGHT_J2}']/limit")
    lower=float(limit.get('lower'));upper=float(limit.get('upper'))
    entry=manifest.get('joint_limit_mapping',{}).get(RIGHT_J2,{})
    expected=max(entry.get('nominal_saved_coordinate_lower_rad',float('nan')),
                 model_lower(entry.get('rest_reference_rad')))
    if (not math.isfinite(lower) or not math.isfinite(upper) or lower!=expected
            or lower>=upper or entry.get('position_limit_policy')!=policy()
            or entry.get('saved_coordinate_lower_rad')!=lower
            or entry.get('saved_coordinate_upper_rad')!=upper):
        raise ValueError('Right J2 operating limit/model mismatch; Calculate again')


def require_goal_bounds(lower):
    """A new generated endpoint cannot claim current policy on old bounds."""
    if len(lower)!=7 or not math.isfinite(float(lower[1])) or lower[1]<=math.radians(RIGHT_J2_OBSERVED_STOP_DEG):
        raise ValueError('Generated goal reserve uses obsolete right J2 operating bounds; regenerate with the 0-degree minimum')


def feedback_limits(manifest, limits, side):
    """Feedback may lie inside the margin; commanded curves are checked separately."""
    limits=[list(row) for row in limits]
    if side=='right' and manifest.get('joint_position_policy')==policy():
        limits[1][0]=math.radians(RIGHT_J2_OBSERVED_STOP_DEG)
    return limits


def bind_request(req, manifest):
    if manifest.get('joint_position_policy')==policy():
        req['joint_position_context']=dict(policy=policy(),
            rest_reference_rad=manifest['joint_limit_mapping'][RIGHT_J2]['rest_reference_rad'])


def require_request(req,manifest):
    if req.get('moving_side','right')!='right':return
    expected={};bind_request(expected,manifest)
    if req.get('joint_position_context')!=expected.get('joint_position_context'):
        raise ValueError('Right J2 rest reference changed; Calculate again')
    reference=expected['joint_position_context']['rest_reference_rad']
    if reference is not None and reference!=req.get('scene_robot_start',req['start']):
        raise ValueError('Right J2 rest reference differs from acquired camera-mask pose')
