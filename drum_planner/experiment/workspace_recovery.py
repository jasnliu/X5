"""Bounded, endpoint-preserving workspace recovery for software experiments.

Legacy successful proposals are tried first, unchanged. Recovery fits the SAME
quintic B-spline that timing/playback use, avoiding interpolation overshoot.
Control bounds enclose the entire curve; repeated end controls give rest ends.
If needed, temporary distal-posture targets guide a departure, never a visited
waypoint. End controls are restored to the REAL goal before any route is tested.
No recordings, scene modifications, relaxed limits, RRT paths or result caches.
"""
import copy
import time

import numpy as np
from scipy.interpolate import BSpline, PPoly

from .. import style
from ..timing.rounding import as_source
from . import gesture, workspace_curve as workspace
from .fluidity import measure, reversals

VERSION = 'bounded-workspace-recovery-v1'
WRIST_GUIDES = ((-.5, .3), (-.5, -.3), (.5, .3), (.5, -.3))
MAX_SEARCH_SECONDS = 20.


class CurveFit(workspace.CurveFit):
    """Seven free joint controls plus three fixed controls at EACH end.

    The shared objective/Jacobian is reused, but not its interpolating spline.
    Nonnegative partition-of-unity basis weights guarantee joint-position bounds
    between samples as well as at controls. No post-fit spline interpolation.
    """
    def __init__(self, oracle, req, offset, options=None, deadline=float('inf')):
        super().__init__(oracle, req, offset, options, deadline)
        m = self.n + 4
        self.knots = np.r_[np.zeros(6), np.linspace(0, 1, m-4)[1:-1], np.ones(6)]
        self.B = BSpline(self.knots, np.eye(m), 5)(self.s)
        self.Bi = self.B[:, 3:-3]
        self.QJ = np.kron(self.Bi, np.eye(7)).reshape(len(self.s), 7, self.nq)
        progress = self.s**3 * (10 + self.s*(-15 + 6*self.s))
        self.C = workspace.bezier_basis(progress)
        self.CJ = np.kron(self.C, np.eye(3)).reshape(len(self.s), 3, 6)
        self.chord = self.a + (self.b-self.a)*progress[:, None]
        self.qline = self.start + (self.goal-self.start)*progress[:, None]

    def nodes(self, x):
        return np.vstack((np.repeat(self.start[None], 3, axis=0),
                          np.asarray(x[:self.nq]).reshape(-1, 7),
                          np.repeat(self.goal[None], 3, axis=0)))

    def polynomial(self, controls):
        pp = [PPoly.from_spline(BSpline(self.knots, controls[:, j], 5)) for j in range(7)]
        return PPoly(np.stack([p.c for p in pp], axis=-1), pp[0].x)


def position_bounds(path, lower, upper):
    """Independent polynomial-extrema check, including every piece boundary."""
    for j in range(7):
        p = PPoly(path.c[:, :, j], path.x, extrapolate=False)
        roots = p.derivative().roots(extrapolate=False)
        roots = roots[np.isfinite(roots) & (roots >= 0) & (roots <= 1)]
        values = p(np.r_[path.x, roots])
        if not np.isfinite(values).all() or values.min() < lower[j]-1e-12 or values.max() > upper[j]+1e-12:
            return False
    return True


def restore_goal(controls, goal):
    actual = controls.copy()
    actual[-3:] = goal
    return actual


def blend_tail(fit, guided, nominal, alpha):
    # Preserve the collision-critical departure. Convex blends of bounded
    # controls remain bounded; no joint-value clipping or endpoint alteration.
    weights = np.clip((fit.knots[:len(guided)]-.25)/.375, 0., 1.)[:, None]*alpha
    return (1-weights)*guided + weights*nominal


def screen(oracle, req, fit, controls, detail, index):
    path = fit.polynomial(controls)
    if not position_bounds(path, oracle.lower, oracle.upper):
        detail['reason'] = 'continuous_joint_bounds'
        return None
    samples = path(np.linspace(0, 1, 1001))
    for q in (samples[::25], samples):
        check = oracle.check(q, edges=True, stop=True)
        if len(check['valid']) != len(q) or not all(check['valid']):
            detail['reason'] = 'collision_or_zone'
            return None
    source = as_source(path, dict(request=req))
    samples = style.sample_path(samples)
    metrics = workspace.path_metrics(samples, gesture.fk(oracle, samples))
    return dict(index=index, path=source['path_waypoints'], timing=source['timing'],
                duration_s=source['timing']['duration_s'], workspace=metrics, gesture=metrics['gesture'],
                workspace_fit=detail, source=detail['family'], stop_at_corners=False, _samples=samples)


def rank(oracle, rows, params):
    if len(rows) > 1:
        features = oracle.call('style', paths=[r['_samples'].tolist() for r in rows])['rows']
        if len(features) != len(rows):
            raise ValueError('Incomplete recovery scoring reply')
    else:
        features = [None]*len(rows)
    for row, feature in zip(rows, features):
        samples = row.pop('_samples')
        row['metrics'] = (measure(samples, feature, dict(reversal_threshold_deg=2.,
                            joint_spacing_rad=.05, spatial_spacing_m=.05)) if feature else
                          dict(reversals=reversals(samples, np.radians(2.)),
                               clearance_scoring='single screened candidate; final validation required'))
        proximity = row['metrics'].get('baseline', {}).get('components', {}).get('obstacle_proximity', 0.)
        row['score'] = workspace.score(row['workspace'], proximity, params)
        row['workspace_fit'].update(score=row['score'], rankable=True)
    return sorted(rows, key=lambda r: (round(r['score'], 8), r['index']))


def stages(oracle, req, params, seconds, log):
    """Lazily try bounded shapes, then four symmetric distal-posture families.

    A consumer accepting any validated timed route stops all further fitting.
    All candidates are checked against the ORIGINAL request/world. The search
    cap excludes mandatory ranking/final certification, like the primary path.
    """
    deadline = time.monotonic() + min(max(0., seconds), MAX_SEARCH_SECONDS)
    endpoints = oracle.check([req['start'], req['goal']])
    if len(endpoints['valid']) != 2 or not all(endpoints['valid']):
        return
    index = 14
    for reverse in (False, True):
        # A route near its arrival obstacle can need the same construction
        # from the other end. This is a FRESH fit, never a reused outward leg
        # or reversal through previously visited checkpoints. The camera mask
        # and passive arm remain those of the original request in both cases.
        for rows, report in direction_stages(oracle, req, params, deadline, log, reverse, index):
            index = max([index]+[r.get('index', index) for r in report['searches']])
            yield rows, report


def direction_stages(oracle, req, params, deadline, log, reverse, index):
    fit_req = copy.deepcopy(req)
    if reverse:
        fit_req['start'], fit_req['goal'] = fit_req['goal'], fit_req['start']
    nominal = {}
    for wrist_delta in (None, *WRIST_GUIDES):
        if time.monotonic() >= deadline:
            return
        began = time.monotonic(); attempts = []; rows = []
        virtual = copy.deepcopy(fit_req)
        if wrist_delta is not None:
            g = np.asarray(fit_req['goal'], float).copy(); g[5:] += wrist_delta
            if np.any(g < oracle.lower) or np.any(g > oracle.upper):
                continue  # Not a destination clamp; simply omit this guide.
            virtual['goal'] = g.tolist()
        ends = gesture.fk(oracle, np.array([fit_req['start'], virtual['goal']]))['wrist_xyz_m']
        offsets = workspace.arc_offsets(*np.asarray(ends))
        family = 'bounded_workspace_curve' if wrist_delta is None else 'wrist_guided_workspace_curve'
        log(f'WORKSPACE RECOVERY: {family}; fit from {"goal" if reverse else "start"}' +
            ('' if wrist_delta is None else f'; distal guide {wrist_delta} rad'))
        for offset_id, offset in enumerate(offsets):
            if time.monotonic() >= deadline:
                break
            fit = CurveFit(oracle, virtual, offset, dict(prior_weight=2.), deadline)
            controls, detail = fit.solve()
            detail.update(family=family, guide_delta_rad=wrist_delta, offset=offset_id,
                          fit_direction='goal_to_start' if reverse else 'start_to_goal',
                          accepted=False, fit_applies_to='actual curve' if wrist_delta is None else 'virtual guide ONLY')
            if controls is None or detail.get('fit_max_m', np.inf) > workspace.OPTIONS['max_fit_m']:
                detail['reason'] = detail.get('reason', 'cartesian_curve_fit_too_far')
                attempts.append(detail); continue
            if wrist_delta is None:
                nominal[offset_id] = controls.copy()
            controls = restore_goal(controls, fit_req['goal'])
            variants = [(0., controls)]
            if wrist_delta is not None and offset_id in nominal:
                variants += [(a, blend_tail(fit, controls, nominal[offset_id], a)) for a in (.25, .5, .75, 1.)]
            for alpha, actual in variants:
                if time.monotonic() >= deadline:
                    break
                index += 1
                record = dict(detail, index=index, tail_blend=alpha)
                attempts.append(record)
                row = screen(oracle, req, fit, actual[::-1] if reverse else actual, record, index)
                if row is not None:
                    rows.append(row)
        rows = rank(oracle, rows, params)
        elapsed = time.monotonic()-began
        report = dict(policy=VERSION, pool_elapsed_s=elapsed, searches=attempts,
            candidates=[{k:r[k] for k in ('index','score','duration_s','workspace','metrics','gesture','source')} for r in rows],
            profile=dict(recovery_s=elapsed))
        log(f'WORKSPACE RECOVERY: ranked {len(rows)}/{len(attempts)} actual-goal curves in {elapsed:.3f}s; final timing check required')
        yield rows, report
