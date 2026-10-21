"""Device-independent broad-route scoring; never a collision validator.

Weights reproduce the user's reviewed route comparison. Every compared route
has the same start/goal, robot, frozen world, and final motion limits. The
fastest fully validated candidate is the normalization reference, not the
selection objective. Timing only gates impractical slowdowns and breaks ties.
"""
import numpy as np

WEIGHTS = dict(workspace_economy=.40, broad_bending=.30, obstacle_proximity=.20,
               joint_economy=.10)
NORMALIZATION_FLOORS = dict(workspace_economy=1., broad_bending=5.,
                            obstacle_proximity=.05, joint_economy=1.)
PATH_SAMPLES = 801
CLEARANCE_SAMPLES = 201
BENDING_SPACING_M = .05
COMFORT_CLEARANCE_M = .10
MAX_DURATION_RATIO = 2.


def sample_path(q):
    """Uniform joint-arc progress, NOT time, so pauses cannot improve the score."""
    q=np.asarray(q,dtype=float)
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all():
        raise ValueError('Style scoring requires finite seven-joint path samples')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    keep=np.r_[True,np.diff(arc)>1e-12];arc=arc[keep];q=q[keep]
    if len(q)<2 or arc[-1]<=1e-10:raise ValueError('No motion to score')
    grid=np.linspace(0.,arc[-1],PATH_SAMPLES)
    result=np.column_stack([np.interp(grid,arc,q[:,i]) for i in range(7)])
    result[0]=q[0];result[-1]=q[-1]
    return result


def broad_turning(x,spacing=BENDING_SPACING_M):
    """Total 3D chord turning at ~5 cm scale; small guiding wiggles aren't bends."""
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(x,axis=0),axis=1))]
    keep=np.r_[True,np.diff(arc)>1e-10]
    if arc[-1]<1e-9:return 0.
    grid=np.linspace(0.,arc[-1],max(3,int(np.ceil(arc[-1]/spacing))+1))
    curve=np.column_stack([np.interp(grid,arc[keep],x[keep,i]) for i in range(3)])
    directions=np.diff(curve,axis=0);length=np.linalg.norm(directions,axis=1)
    directions=directions[length>1e-10]/length[length>1e-10,None]
    if len(directions)<2:return 0.
    cosine=np.sum(directions[:-1]*directions[1:],axis=1)
    return float(np.degrees(np.arccos(np.clip(cosine,-1.,1.))).sum())


def metrics(q,features):
    q=np.asarray(q,dtype=float)
    if q.shape!=(PATH_SAMPLES,7) or not np.isfinite(q).all():
        raise ValueError('Incomplete style joint samples')
    if features.get('joint_bounds_valid') is not True:
        raise ValueError('Style curve exceeds joint limits')
    distances=np.asarray(features.get('distal_clearance_estimate_m'),dtype=float)
    if (distances.shape!=(CLEARANCE_SAMPLES,) or not np.isfinite(distances).all()
            or np.any((distances<0)|(distances>COMFORT_CLEARANCE_M))):
        raise ValueError('Invalid/incomplete soft clearance estimates')
    lengths=[];direct=[];bends=[];detail={}
    for name in ('wrist','tip'):
        x=np.asarray(features.get(name+'_xyz_m'),dtype=float)
        if x.shape!=(PATH_SAMPLES,3) or not np.isfinite(x).all():
            raise ValueError('Invalid/incomplete style FK samples')
        length=float(np.linalg.norm(np.diff(x,axis=0),axis=1).sum())
        displacement=float(np.linalg.norm(x[-1]-x[0]));turn=broad_turning(x)
        lengths.append(length);direct.append(displacement);bends.append(turn)
        detail[name]=dict(travel_m=length,direct_distance_m=displacement,broad_turning_deg=turn)
    joint_travel=float(np.abs(np.diff(q,axis=0)).sum())
    joint_direct=float(np.abs(q[-1]-q[0]).sum())
    components=dict(workspace_economy=sum(lengths)/max(sum(direct),.01),
                    broad_bending=sum(bends),
                    obstacle_proximity=float(np.mean((1.-distances/COMFORT_CLEARANCE_M)**2)),
                    joint_economy=joint_travel/max(joint_direct,.001))
    return dict(components=components,**detail,joint_travel_rad=joint_travel,
                joint_direct_rad=joint_direct,minimum_distal_padding_estimate_m=float(distances.min()),
                clearance_is_soft_estimate_not_safety_certificate=True,
                joint_arc_samples=PATH_SAMPLES,clearance_samples=CLEARANCE_SAMPLES)


def normalizers(reference):
    return {key:max(float(reference['components'][key]),NORMALIZATION_FLOORS[key]) for key in WEIGHTS}


def score(measured,scales):
    terms={key:100.*weight*float(measured['components'][key])/float(scales[key])
           for key,weight in WEIGHTS.items()}
    if not all(np.isfinite(v) and v>=0 for v in terms.values()):
        raise ValueError('Nonfinite or negative style score')
    return dict(total=float(sum(terms.values())),weighted_components=terms)
