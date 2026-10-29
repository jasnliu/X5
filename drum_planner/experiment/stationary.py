"""A selected start equal to a target means zero travel, not a fitted loop."""
import numpy as np
from scipy.interpolate import PPoly

from ..timing import core
from .motion_settings import summary


def candidate(req, oracle, report):
    report.update(selected=None,candidates=[],searches=[],pool_elapsed_s=0.,
                  joint_limit_check=oracle.meta['joint_limit_check'],stationary=True)
    checked=oracle.check([req['start']])
    if checked['valid'] != [True]:
        report.update(reason='nominal_endpoint_invalid',details=checked)
        return []
    q=np.asarray(req['start']);c=np.zeros((6,1,7));c[-1,0]=q
    timing=dict(stop_at_corners=False,time_scale=1.,duration_s=1.,
                coefficients=np.moveaxis(c,-1,0).tolist(),
                polynomial_breaks_unscaled_s=[[0.,1.]]*7)
    return [dict(index=0,path=[q.tolist(),q.tolist()],timing=timing,
                 workspace_fit=dict(accepted=True,stationary=True),source='Identical start and target')]


def law(source, limits, quality):
    path=core.original(source)
    if np.any(path.c[:-1]!=0) or not np.array_equal(path(0),path(1)):
        raise ValueError('Stationary timing requested for a moving path')
    result=dict(schema='x5-fixed-path-timing-v1',kind='joint_polynomial',
        method='experiment_stationary',curve=core.encode(PPoly(path.c.copy(),path.x.copy())),
        stationary=True,duration_s=0.,limits=limits.tolist(),peaks=np.zeros((3,7)).tolist(),
        geometry_preserved=True,source_path_sha256=core.identity(source),
        details=dict(actual_backend='analytic_cpu',requested_backend='cpu_sparse',
            backend_label='No movement — start equals target; checked hold only',
            selected_stage='stationary',fallback_used=False,fallback_reason=None,warnings=[],history=[],
            calculation_seconds=0.,timing_quality=quality,cpu_reference_used=False,result_cache_used=False,
            global_optimum_claimed=False))
    result['details']['constraint_summary']=summary([result],limits)
    return result
