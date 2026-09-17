"""No CAN access. Checks a running viewer's ROS outputs against offline FK."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from safe_zone.geometry import Model, TCP
rclpy.init()
n=Node('safe_zone_verification')
b=Buffer(); listener=TransformListener(b,n)
state={}
n.create_subscription(JointState,'/joint_states',lambda m: state.update(q=dict(zip(m.name,m.position))),10)
n.create_subscription(MarkerArray,'/safe_zone/markers',lambda m: state.update(markers=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
end=time.monotonic()+10
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
    if 'q' in state and 'markers' in state and b.can_transform('world',TCP,rclpy.time.Time()):break
assert 'q' in state and 'markers' in state
tr=b.lookup_transform('world',TCP,rclpy.time.Time()).transform.translation
fk=Model('model/openarmx.urdf').transforms(state['q'])[TCP][:3,3]
np.testing.assert_allclose([tr.x,tr.y,tr.z],fk,atol=1e-8)
markers=state['markers'].markers
assert len(markers)==5
assert len(markers[0].points)==8
assert len(markers[1].points)>0
print('PASS: joint states, TF vs FK, sampled points and hull markers received')
n.destroy_node();rclpy.shutdown()
