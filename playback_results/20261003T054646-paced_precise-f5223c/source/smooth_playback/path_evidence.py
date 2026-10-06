"""Offline geometric check of actual measured motion against the raw recording.

Nearest recorded-path phase is used because local retiming is intentional.
This is sampled motor-side evidence, not continuous collision certification.
"""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from .trajectory import ROOT,SOURCE,Geometry


def main():
    source=json.loads(SOURCE.read_text());samples=source['samples']
    times=np.array([s['time_s'] for s in samples])
    original=np.array([s['positions_rad'] for s in samples])
    phase=np.linspace(times[0],times[-1],10001)
    reference=np.column_stack([np.interp(phase,times,original[:,j]) for j in range(7)])
    tree=cKDTree(reference);g=Geometry();results=[]
    for p in sorted((ROOT/'playback_results').glob('2026*-paced_precise-*')):
        if not (p/'summary.json').exists():continue
        summary=json.loads((p/'summary.json').read_text())
        if not summary['success']:continue
        events=[json.loads(s) for s in (p/'events.jsonl').read_text().splitlines()]
        start=next(e['t'] for e in events if e['phase']=='PLAYBACK')
        grid=np.arange(start+.02,start+summary['trajectory']['duration_s'],.02)
        with (p/'feedback.csv').open() as f:rows=list(csv.DictReader(f))
        actual=[]
        for j in range(1,8):
            r=[x for x in rows if x['side']=='right' and int(x['motor'])==j]
            actual.append(np.interp(grid,[float(x['monotonic_s']) for x in r],[float(x['position_rad']) for x in r]))
        actual=np.array(actual).T;_,nearest=tree.query(actual);ref=reference[nearest]
        errors=np.degrees(np.abs(actual-ref))
        tcp=[float(np.linalg.norm(g.tcp(q)-g.tcp(r))*1000) for q,r in zip(actual,ref)]
        item=dict(session=str(p.relative_to(ROOT)),samples=len(grid),
                  max_joint_difference_at_nearest_recorded_phase_deg=float(errors.max()),
                  max_tcp_difference_at_nearest_recorded_phase_mm=max(tcp),
                  note='Actual feedback at 50 Hz versus nearest Euclidean seven-joint pose on densely interpolated raw record3; phase may differ by design. Host receipt timestamps. Sampled proximity, not a bound between samples or whole-arm collision guarantee.')
        (p/'measured_path_proximity.json').write_text(json.dumps(item,indent=2)+'\n');results.append(item)
    (ROOT/'playback_results/measured_path_proximity.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
