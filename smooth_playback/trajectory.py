"""Endpoint-exact, corridor-constrained trajectories; no hardware I/O."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import make_interp_spline, PPoly
from scipy.ndimage import gaussian_filter1d
from camera_playback.trajectory import load_playback_trajectory
from safe_zone.geometry import Model, Zone, RIGHT_TCP, MEMBERSHIP_BUFFER_M

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'recordings/record3.json'
METHODS = ('baseline', 'legacy_gui', 'retimed', 'matched', 'smooth', 'smooth_csp04', 'smooth_200hz', 'smooth_slow', 'paced', 'paced_200hz', 'paced_soft', 'paced_soft200', 'paced_damped', 'paced_damped200', 'paced_kp10', 'paced_kp20', 'paced_kp30', 'paced_precise')
JOINT_CORRIDOR = math.radians(1.5)
TCP_CORRIDOR = .010


class Geometry:
    def __init__(self):
        self.model = Model(ROOT / 'model/openarmx.urdf')
        self.zone = Zone.load(ROOT / 'right_zones/zone1.json', self.model.digest, RIGHT_TCP)
        root = ET.parse(ROOT / 'model/openarmx.urdf').getroot()
        limits = [root.find(f"joint[@name='openarmx_right_joint{i}']/limit") for i in range(1, 8)]
        self.lower = np.array([float(e.get('lower')) for e in limits])
        self.upper = np.array([float(e.get('upper')) for e in limits])
        self.center = np.array([0.] * 6 + [self.upper[6]])

    def tcp(self, q):
        side = getattr(self, 'side', 'right')
        return self.model.transforms({f'openarmx_{side}_joint{i+1}': float(v) for i,v in enumerate(q)})[f'openarmx_{side}_hand_tcp'][:3,3]

    def check(self, q, measured=False):
        q = np.asarray(q)
        tolerance = 3 * 25.14 / 65535 if measured else 1e-9
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('Invalid joint vector')
        if np.any(q < self.lower-tolerance) or np.any(q > self.upper+tolerance):
            raise ValueError('Joint limit violation')
        if not self.zone.contains(self.tcp(q), MEMBERSHIP_BUFFER_M):
            raise ValueError('Pose outside existing right zone1 TCP envelope')

    def check_line(self, a, b, measured_start=False):
        # <= 0.25 degree joint spacing; synchronized straight joint-space path.
        n = max(2, int(np.ceil(np.max(np.abs(np.array(b)-a))/math.radians(.25)))+1)
        for q in np.linspace(a,b,n):
            self.check(q, measured=measured_start)

    def check_relaxed_start(self,a):
        """Allow at most 1 degree of passive sag beyond a modeled joint stop.

        This only authorizes inward recovery to the nearest legal pose; it does
        not enlarge playback limits or permit further outward commands.
        """
        a=np.asarray(a,dtype=float)
        legal=np.clip(a,self.lower,self.upper)
        if not np.isfinite(a).all() or np.max(np.abs(a-legal))>math.radians(1.):
            raise ValueError('Relaxed start is more than 1 degree beyond modeled limits; operator recovery required')
        for q in np.linspace(a,legal,21):
            if not self.zone.contains(self.tcp(q),MEMBERSHIP_BUFFER_M):
                raise ValueError('Inward startup recovery leaves zone1')
        self.check_line(legal,self.center)
        return legal


def extrema(pp, derivative=0):
    p = pp.derivative(derivative)
    lo, hi = pp.x[0], pp.x[-1]
    roots = p.derivative().roots(extrapolate=False)
    roots = roots[np.isfinite(roots) & (roots >= lo) & (roots <= hi)]
    t = np.r_[pp.x, roots]
    return float(np.max(np.abs(p(t))))


@dataclass
class Trajectory:
    name: str
    curves: list
    source_duration: float
    scale: float
    first: np.ndarray
    last: np.ndarray
    metadata: dict
    reference_map: object = None
    command_schedule: object = None

    @property
    def duration(self): return self.source_duration*self.scale

    def at(self, elapsed):
        if elapsed <= 0: return self.first.copy()
        if elapsed >= self.duration: return self.last.copy()
        return np.array([p(elapsed/self.scale) for p in self.curves])


def build(geometry=None):
    g = geometry or Geometry()
    old = load_playback_trajectory(SOURCE, g.model, g.zone, g.lower, g.upper, g.center, .4, playback_speed=.8)
    # Fail rather than silently accept clipping of the operational endpoints.
    import json
    original = json.loads(SOURCE.read_text())['samples']
    if not np.array_equal(old.first_joints, original[0]['positions_rad']) or not np.array_equal(old.last_joints, original[-1]['positions_rad']):
        raise ValueError('Original endpoints require clipping; refusing playback')
    duration = old.original_duration_s
    raw_t, raw_q = old.source_times, old.joints
    curves = [PPoly(old.curve_coefficients[:,:,j][:,::-1].T, raw_t) for j in range(7)]
    result = {}
    for name in ('baseline','retimed'):
        result[name] = make_trajectory(name, curves, duration, old.first_joints, old.last_joints,
                                       old.time_scale if name=='baseline' else None)
    # Preserve all methods; widest passing smoothing window wins OFFLINE only.
    # Quintic interpolation has zero first and second derivatives at both ends.
    grid = np.linspace(0, duration, int(np.ceil(duration/.01))+1)
    values = np.column_stack([np.interp(grid,raw_t,raw_q[:,j]) for j in range(7)])
    audit = []
    for sigma in (.14, .10, .07, .04):
        filtered = gaussian_filter1d(values, sigma/(grid[1]-grid[0]), axis=0, mode='nearest')
        knots = np.linspace(0,duration,int(np.ceil(duration/.20))+1)
        q = np.column_stack([np.interp(knots,grid,filtered[:,j]) for j in range(7)])
        q[0],q[-1] = old.first_joints,old.last_joints
        bs = [make_interp_spline(knots,q[:,j],k=5,bc_type=([(1,0.),(2,0.)],[(1,0.),(2,0.)])) for j in range(7)]
        candidate = make_trajectory('smooth',[PPoly.from_spline(b) for b in bs],duration,old.first_joints,old.last_joints)
        try:
            metrics = validate(candidate, g, raw_t, raw_q)
            candidate.metadata.update(metrics, gaussian_sigma_s=sigma, knot_spacing_s=float(knots[1]-knots[0]))
            audit.append(dict(sigma=sigma, accepted=True, **metrics))
            result['smooth'] = candidate
            break
        except ValueError as e:
            audit.append(dict(sigma=sigma,accepted=False,reason=str(e)))
    if 'smooth' not in result: raise ValueError(f'No smoothing candidate stayed in corridor: {audit}')
    result['matched'] = make_trajectory('matched', curves, duration, old.first_joints,
                                        old.last_joints, result['smooth'].scale)
    result['matched'].metadata['comparison_note'] = 'Original fitted path at exactly the smooth method duration and 100 Hz'
    result['matched'].metadata['limits_rad'] = None
    for name,hz in (('smooth_csp04',100),('smooth_200hz',200)):
        base=result['smooth']
        result[name]=Trajectory(name,base.curves,base.source_duration,base.scale,
                                base.first.copy(),base.last.copy(),dict(base.metadata))
        result[name].metadata.update(name=name,firmware_speed_rad_s=.4,command_hz=hz,
                                     comparison_note='Same smooth path, lower CSP firmware speed ceiling')
    base=result['smooth']
    slow=make_trajectory('smooth_slow',base.curves,duration,base.first,base.last,base.scale*2.)
    slow.metadata.update(gaussian_sigma_s=base.metadata['gaussian_sigma_s'],
                         knot_spacing_s=base.metadata['knot_spacing_s'],
                         firmware_speed_rad_s=.4,acceleration_continuous_into_hold=True,
                         comparison_note='Same constrained quintic path, twice the duration, 100 Hz, CSP 0.4')
    result['smooth_slow']=slow
    # Timing-restricted candidate: redistribute progress along the same nearby
    # path, rather than stretching the whole original time base for one spike.
    from .retime_prototype import fit as path_dependent_timing
    from scipy.interpolate import PchipInterpolator
    pc,pt,ps=path_dependent_timing(base,amax=1.,vmax=.70,smoothing=2,edge=.08)
    paced=make_trajectory('paced',pc,float(pt[-1]),base.first,base.last,4.6/float(pt[-1]))
    paced.reference_map=PchipInterpolator(pt,ps)
    paced.metadata.update(limits_rad=[.72,1.5,40.],acceleration_continuous_into_hold=True,
                           gaussian_sigma_s=base.metadata['gaussian_sigma_s'],
                           original_recording_duration_s=duration,recording_duration_ratio=4.6/duration,
                           timing_method='arc-length acceleration-constrained LP, smoothed squared speed, endpoint ramps',
                           comparison_note='Timing-restricted 4.6 second path playback; original record3 is 3.841315 seconds')
    peaks=[paced.metadata[k] for k in ('peak_velocity_rad_s','peak_acceleration_rad_s2','peak_piecewise_jerk_rad_s3')]
    if any(p>limit for p,limit in zip(peaks,[.72,1.5,40.])):raise ValueError('Paced trajectory exceeds its dynamic limits')
    result['paced']=paced
    result['paced_200hz']=Trajectory('paced_200hz',paced.curves,paced.source_duration,paced.scale,
        paced.first.copy(),paced.last.copy(),dict(paced.metadata,name='paced_200hz',command_hz=200),paced.reference_map)
    pc,pt,ps=path_dependent_timing(base,amax=1.,vmax=[.35,.35,.35,.70,.35,.35,.35],smoothing=2,edge=.08)
    soft=make_trajectory('paced_soft',pc,float(pt[-1]),base.first,base.last,4.8/float(pt[-1]))
    soft.reference_map=PchipInterpolator(pt,ps)
    soft.metadata.update(limits_rad=[.72,1.5,40.],acceleration_continuous_into_hold=True,
                         gaussian_sigma_s=base.metadata['gaussian_sigma_s'],
                         original_recording_duration_s=duration,recording_duration_ratio=4.8/duration,
                         firmware_joint_speeds_rad_s=[.4,.4,.4,.8,.4,.4,.4],
                         timing_method='joint-specific velocity limits, arc-length acceleration-constrained LP',
                         comparison_note='4.8 seconds; only J4 needs the 0.8 CSP ceiling, others use 0.4')
    for key,limit in zip(('peak_velocity_rad_s','peak_acceleration_rad_s2','peak_piecewise_jerk_rad_s3'),[.72,1.5,40.]):
        if soft.metadata[key]>limit:raise ValueError('Paced-soft dynamic limit exceeded')
    result['paced_soft']=soft
    result['paced_soft200']=Trajectory('paced_soft200',soft.curves,soft.source_duration,soft.scale,
        soft.first.copy(),soft.last.copy(),dict(soft.metadata,name='paced_soft200',command_hz=200),soft.reference_map)
    for name,hz in (('paced_damped',100),('paced_damped200',200)):
        result[name]=Trajectory(name,soft.curves,soft.source_duration,soft.scale,
            soft.first.copy(),soft.last.copy(),dict(soft.metadata,name=name,command_hz=hz,position_gain_factor=.5),soft.reference_map)
    for cap in (10.,20.,30.):
        name=f'paced_kp{int(cap)}'
        result[name]=Trajectory(name,soft.curves,soft.source_duration,soft.scale,
            soft.first.copy(),soft.last.copy(),dict(soft.metadata,name=name,command_hz=200,position_gain_cap=cap),soft.reference_map)
    result['paced_precise']=Trajectory('paced_precise',soft.curves,soft.source_duration,soft.scale,
        soft.first.copy(),soft.last.copy(),dict(soft.metadata,name='paced_precise',command_hz=200,
            position_gain_cap=10.,precise_endpoint=True,
            endpoint_note='Soft settle, guarded 0.5 s original-gain restoration ramp, then 0.05 degree endpoint verification; never relax here'),soft.reference_map)
    capture=ROOT/'playback_results/legacy_gui_timing.json'
    if capture.exists():
        timing=json.loads(capture.read_text());schedule=timing['commands'];previous=-1.
        for row in schedule:
            stamp=float(row['time_s']);q=np.asarray(row['positions_rad'],dtype=float)
            if not previous<stamp<=old.duration_s+.15:raise ValueError('Invalid captured GUI timestamp')
            previous=stamp;g.check(q)
            if np.max(np.abs(q-result['baseline'].at(stamp)))>math.radians(.5):
                raise ValueError('Captured GUI command does not match the original playback curve')
        b=result['baseline']
        result['legacy_gui']=Trajectory('legacy_gui',b.curves,b.source_duration,b.scale,b.first.copy(),b.last.copy(),
            dict(b.metadata,name='legacy_gui',command_hz=None,captured_command_count=len(schedule),
                 captured_median_interval_s=timing['median_command_interval_s'],
                 captured_max_interval_s=timing['max_command_interval_s'],
                 timing_capture_sha256=hashlib.sha256(capture.read_bytes()).hexdigest(),
                 comparison_note='Physical replay of unchanged GUI command timing captured in simulation; not live camera/audio workload'),
            command_schedule=schedule)
    checked_shapes={}
    for name, candidate in result.items():
        key=id(candidate.curves)
        if key not in checked_shapes:
            checked_shapes[key]=validate(candidate,g,raw_t,raw_q)
            # Timing/rate variants share precisely the same geometric curve.
            # Representative return paths are not whole-arm collision certification.
            for t in np.linspace(0,candidate.duration,17):
                g.check_line(candidate.at(t),g.center)
            g.check_line(g.center,candidate.first)
        candidate.metadata.update(checked_shapes[key])
        candidate.metadata['source_sha256'] = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    result['smooth'].metadata['smoothing_search'] = audit
    return g,result


def make_trajectory(name, curves, duration, first, last, scale=None):
    maxima = [max(extrema(p, d) for p in curves) for d in (1,2,3)]
    limits = [.55, .8, 6.]
    if scale is None:
        scale = max(1., *((m/l)**(1/d) for d,(m,l) in enumerate(zip(maxima,limits),1)))
        scale *= 1.01
    return Trajectory(name,curves,duration,float(scale),first.copy(),last.copy(),dict(
        name=name,duration_s=float(duration*scale),time_scale=float(scale),
        peak_velocity_rad_s=maxima[0]/scale,peak_acceleration_rad_s2=maxima[1]/scale**2,
        peak_piecewise_jerk_rad_s3=maxima[2]/scale**3,
        acceleration_continuous_into_hold=name=='smooth',
        command_hz=50 if name=='baseline' else 100,
        firmware_speed_rad_s=.8,limits_rad=[.55,.8,6.] if name!='baseline' else None))


def validate(trajectory,g,raw_t,raw_q):
    ts=np.unique(np.r_[np.linspace(0,trajectory.source_duration,2001),raw_t])
    qs=np.array([trajectory.at(t*trajectory.scale) for t in ts])
    reference_times=ts if trajectory.reference_map is None else trajectory.reference_map(ts)
    original=np.column_stack([np.interp(reference_times,raw_t,raw_q[:,j]) for j in range(7)])
    deviation=float(np.max(np.abs(qs-original)))
    if deviation>JOINT_CORRIDOR:
        raise ValueError(f'Joint corridor {math.degrees(deviation):.4f} deg > 1.5 deg')
    # Analytic polynomial extrema enforce limits between the dense checks too.
    for j,p in enumerate(trajectory.curves):
        roots=p.derivative().roots(extrapolate=False)
        roots=roots[np.isfinite(roots)&(roots>=0)&(roots<=trajectory.source_duration)]
        vals=p(np.r_[0.,trajectory.source_duration,roots,p.x])
        if np.min(vals)<g.lower[j]-1e-9 or np.max(vals)>g.upper[j]+1e-9:
            raise ValueError(f'Curve exceeds joint {j+1} limits')
    max_tcp=0.
    for q,r in zip(qs[::5],original[::5]):
        g.check(q)
        max_tcp=max(max_tcp,float(np.linalg.norm(g.tcp(q)-g.tcp(r))))
    g.check(trajectory.last)
    if max_tcp>TCP_CORRIDOR: raise ValueError(f'TCP corridor {max_tcp*1000:.3f} mm > 10 mm')
    return dict(max_joint_deviation_deg=math.degrees(deviation),max_tcp_deviation_mm=max_tcp*1000,
                exact_first=bool(np.array_equal(trajectory.at(0),raw_q[0])),
                exact_last=bool(np.array_equal(trajectory.at(trajectory.duration),raw_q[-1])))


def transition(a,b):
    """Rest-to-rest quintic on a validated straight joint-space segment."""
    a,b=np.asarray(a),np.asarray(b)
    delta=float(np.max(np.abs(b-a)))
    duration=max(1., 1.875*delta/.28, math.sqrt(5.774*delta/.5), (60*delta/2.)**(1/3))
    def at(t):
        s=np.clip(t/duration,0.,1.)
        return a+(b-a)*(10*s**3-15*s**4+6*s**5)
    return duration, at
