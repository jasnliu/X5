"""Offline right Cartesian geometry and blocker audit; no CAN access."""
import json
from unittest.mock import Mock
import numpy as np
from cartesian_goal.ik import CartesianIK,IK_TOLERANCE_M
from centering.motors import SPEED
from safe_zone.geometry import Model,Zone,RIGHT_TCP,MEMBERSHIP_BUFFER_M

model=Model('model/openarmx.urdf')
zone=Zone.load('right_zones/zone1.json',model.digest,RIGHT_TCP)
joints={joint.get('name'):joint for joint in model.joints}
limits=[joints[f'openarmx_right_joint{i}'].find('limit') for i in range(1,8)]
lower=np.array([float(limit.get('lower')) for limit in limits])
upper=np.array([float(limit.get('upper')) for limit in limits])
origin=np.zeros(7);home=np.zeros(7);home[6]=upper[6]

def position(q):
    values={f'openarmx_right_joint{i+1}':float(q[i]) for i in range(7)}
    return model.transforms(values)[RIGHT_TCP][:3,3]

def signed_distance(point):
    normals=zone.hull.equations[:,:3]
    distance=(normals@point+zone.hull.equations[:,3])/np.linalg.norm(normals,axis=1)
    return float(np.max(distance))

origin_tcp=position(origin);home_tcp=position(home)
path_distance=[]
for fraction in np.linspace(0.,1.,1001):path_distance.append(signed_distance(position(origin+(home-origin)*fraction)))
target=origin_tcp+np.array([0.,0.,.05])

assert home[6]==1.4
assert not zone.contains(origin_tcp,MEMBERSHIP_BUFFER_M)
assert zone.contains(home_tcp,MEMBERSHIP_BUFFER_M)
assert zone.contains(target,MEMBERSHIP_BUFFER_M)

actual_ik=CartesianIK(model,zone,lower,upper,SPEED,'right',RIGHT_TCP,home,origin)
actual_solution=actual_ik.solve([0,0,.05])
assert actual_solution.error_m<=IK_TOLERANCE_M
assert actual_ik.path_inside(actual_solution.joints)

# Prove the right-side IK itself works when geometry membership is permissive;
# this does not weaken the application or the saved zone.
permissive=Mock();permissive.contains.return_value=True
test_ik=CartesianIK(model,permissive,lower,upper,SPEED,'right',RIGHT_TCP,home,origin)
solution=test_ik.solve([0,0,.05])
assert solution.error_m<=IK_TOLERANCE_M
np.testing.assert_allclose(test_ik.position(solution.joints)-origin_tcp,[0,0,.05],atol=IK_TOLERANCE_M)

print(json.dumps({
    'mode':'offline-no-can',
    'right_zone_points':len(zone.points),
    'cartesian_origin_joint_degrees':np.degrees(origin).tolist(),
    'cartesian_origin_tcp_m':origin_tcp.tolist(),
    'right_center_joint_degrees':np.degrees(home).tolist(),
    'right_center_tcp_m':home_tcp.tolist(),
    'origin_signed_outside_original_mm':signed_distance(origin_tcp)*1000,
    'center_signed_outside_original_mm':signed_distance(home_tcp)*1000,
    'center_buffered_margin_mm':(MEMBERSHIP_BUFFER_M-signed_distance(home_tcp))*1000,
    'zero_to_center_buffered_violations':sum(d>MEMBERSHIP_BUFFER_M for d in path_distance),
    'zero_to_center_samples':len(path_distance),
    'up_5cm_target_inside_buffered_zone':zone.contains(target,MEMBERSHIP_BUFFER_M),
    'actual_zone_result':'accepted: center and solved path are inside zone1',
    'actual_zone_ik_error_mm':actual_solution.error_m*1000,
    'actual_zone_solution_degrees':np.degrees(actual_solution.joints).tolist(),
    'permissive_geometry_ik_error_mm':solution.error_m*1000,
    'permissive_geometry_solution_degrees':np.degrees(solution.joints).tolist(),
},indent=2))
