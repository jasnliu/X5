"""Finite exact-parameter jobs, preserving each attempt in the ordinary archive."""
import math
from .methods import METHODS


def validate_jobs(jobs):
    if not isinstance(jobs,list) or not 1<=len(jobs)<=2000:raise ValueError('Expected 1–2000 finite jobs')
    for j in jobs:
        if set(j)-{'method','parameters','repetitions','stage','interval','intervals'}:raise ValueError('Unknown job key')
        METHODS[j['method']].parameters(j.get('parameters'))
        n=j.get('repetitions',1)
        if type(n) is not int or not 1<=n<=10000:raise ValueError('Invalid job count')
        periods=j.get('intervals',[j.get('interval',.25)])
        if not periods or not all(isinstance(v,(int,float)) and math.isfinite(v) and 0<=v<=60 for v in periods):
            raise ValueError('Invalid job intervals')
    return jobs


def run_jobs(engine,jobs,pause=None):
    for job in validate_jobs(jobs):
        next_start=None
        for i in range(job.get('repetitions',1)):
            while pause and pause.is_set():engine.idle()
            schedule=None
            if 'intervals' in job:
                if next_start is None:next_start=engine.backend.now()
                while engine.backend.now()<next_start:
                    engine.idle(min(.01,max(.0005,next_start-engine.backend.now())))
                schedule={'requested_start':next_start}
            trial=engine.trial(job['method'],job.get('parameters'),job.get('stage','calibration'),schedule=schedule)
            if 'intervals' in job:
                next_start=trial['score']['release_at']+job['intervals'][i%len(job['intervals'])]
            else:
                until=engine.backend.now()+job.get('interval',.25)
                while engine.backend.now()<until:engine.idle()
