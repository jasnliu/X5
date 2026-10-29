"""Arm-generic FK/Jacobians from the exact generated, calibrated tool model."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ..common import arm_joints, arm_side, digest


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def transform(origin):
    T=np.eye(4)
    if origin is not None:
        T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    if not np.isfinite(T).all():raise ValueError('Invalid URDF transform')
    return T


class Kinematics:
    def __init__(self,path,side):
        self.side=arm_side(side);self.joints=arm_joints(side);self.sha256=digest(path)
        root=ET.parse(path).getroot()
        if root.find('ros2_control') is not None:raise ValueError('No hardware control model allowed')
        parents={j.find('child').get('link'):j for j in root.findall('joint')}
        self.chain=[];link=side+'_drumstick';seen=set();moving=[]
        cylinder=root.find(f"link[@name='{link}']/collision/geometry/cylinder")
        if cylinder is None:raise ValueError('Missing modeled stick cylinder')
        self.tip_local=transform(root.find(f"link[@name='{link}']/collision/origin"))
        self.tip_local=self.tip_local@np.array([[1,0,0,0],[0,1,0,0],[0,0,1,float(cylinder.get('length'))/2],[0,0,0,1]])
        self.lower=np.zeros(7);self.upper=np.zeros(7)
        while link!='world':
            if link in seen or link not in parents:raise ValueError('Disconnected or cyclic tool chain')
            seen.add(link);j=parents[link];index=None;axis=None
            if j.get('type')!='fixed':
                name=j.get('name')
                if name not in self.joints or j.get('type')!='revolute' or j.find('mimic') is not None:
                    raise ValueError('Unsupported joint in tool chain')
                index=self.joints.index(name);moving.append(index)
                axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
                self.lower[index]=float(j.find('limit').get('lower'));self.upper[index]=float(j.find('limit').get('upper'))
            self.chain.insert(0,(transform(j.find('origin')),index,axis,link))
            link=j.find('parent').get('link')
        if sorted(moving)!=list(range(7)):raise ValueError('Expected seven selected-arm joints')

    def fk(self,q,jac=False):
        q=np.asarray(q,float);single=q.ndim==1;q=np.atleast_2d(q)
        if q.shape[1]!=7 or not np.isfinite(q).all():raise ValueError('Expected finite N by 7 joints')
        T=np.tile(np.eye(4),(len(q),1,1));origins=np.zeros((len(q),7,3));axes=origins.copy();wrist=None
        for origin,index,axis,link in self.chain:
            T=T@origin
            if index is not None:
                origins[:,index]=T[:,:3,3];axes[:,index]=T[:,:3,:3]@axis
                motion=np.tile(np.eye(4),(len(q),1,1))
                motion[:,:3,:3]=Rotation.from_rotvec(q[:,index,None]*axis).as_matrix();T=T@motion
            if link==f'openarmx_{self.side}_hand':wrist=T[:,:3,3].copy()
        T=T@self.tip_local;tip=T[:,:3,3];axis=T[:,:3,2]
        result=dict(tip=tip,axis=axis,R=T[:,:3,:3],wrist=wrist)
        if jac:
            result.update(Jtip=np.cross(axes,tip[:,None,:]-origins).transpose(0,2,1),
                          Jaxis=np.cross(axes,axis[:,None,:]).transpose(0,2,1),
                          Jrotation=axes.transpose(0,2,1))
        return {k:v[0] if single else v for k,v in result.items()}

    def verify_native(self,oracle,q):
        q=np.atleast_2d(q);local=self.fk(q);native=oracle.call('fk',q=q.tolist())
        errors={key:float(np.max(abs(local[key]-np.asarray(native[nkey]))))
                for key,nkey in [('tip','tip_xyz_m'),('wrist','wrist_xyz_m')]}
        if max(errors.values())>1e-9:raise ValueError('New FK disagrees with native collision model: '+str(errors))
        np.testing.assert_allclose(self.lower,oracle.lower,atol=1e-12,rtol=0)
        np.testing.assert_allclose(self.upper,oracle.upper,atol=1e-12,rtol=0)
        return errors


def target(kin,q):
    f=kin.fk(q);axis=f['axis'];guide=np.eye(3)[np.argmin(abs(axis))]
    a=np.cross(axis,guide);a/=np.linalg.norm(a);b=np.cross(axis,a)
    return dict(tip=f['tip'],axis=axis,R=f['R'],tangent=np.array([a,b]))


def error(kin,q,goal,kind,jac=False):
    f=kin.fk(q,jac=jac);position=(f['tip']-goal['tip'])/.3
    if kind=='tip-axis':
        rotation=goal['tangent']@f['axis']
        if jac:Jr=goal['tangent']@f['Jaxis']
    elif kind=='full-pose':
        rotation=Rotation.from_matrix(goal['R'].T@f['R']).as_rotvec()
        if jac:
            theta=np.linalg.norm(rotation);K=skew(rotation)
            coefficient=1/12 if theta<1e-5 else (1-(theta/2)/np.tan(theta/2))/theta**2
            Jr=(np.eye(3)-.5*K+coefficient*K@K)@goal['R'].T@f['Jrotation']
    else:raise ValueError('Unknown task constraint')
    residual=np.r_[position,rotation]
    return (residual,np.vstack((f['Jtip']/.3,Jr))) if jac else residual


def task_proof(kin,q,goal,kind):
    f=kin.fk(q)
    position=float(np.linalg.norm(f['tip']-goal['tip']))
    direction=float(np.arccos(np.clip(f['axis']@goal['axis'],-1,1)))
    orientation=float(np.linalg.norm(Rotation.from_matrix(goal['R'].T@f['R']).as_rotvec()))
    # Numerical IK tolerance, NOT a permission to relocate a drum target.
    valid=position<=1e-5 and direction<=1e-4 and (kind!='full-pose' or orientation<=1e-4)
    return dict(valid=bool(valid),tip_error_m=position,axis_error_rad=direction,
                full_orientation_error_rad=orientation,tool_roll_free=kind=='tip-axis')
