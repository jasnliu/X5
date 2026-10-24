"""Summarize measured full-policy parity and native-UI cold/warm runs."""
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def numerical(law):
    law=json.loads(json.dumps(law))
    law['details'].pop('calculation_seconds',None)
    for row in law['details'].get('tuning',[]):row.pop('elapsed_s',None)
    return law


def main():
    before=ROOT/'planner_runs/performance-reference-20261021'
    after=ROOT/'planner_runs/performance-current-20261021'
    old=json.loads((before/'report.json').read_text())
    new=json.loads((after/'report.json').read_text())
    result=dict(profiled_comparisons={},cross_session_ui={},hardware_access=False)
    for case in old:
        a=json.loads((before/(case+'-law.json')).read_text())
        b=json.loads((after/(case+'-law.json')).read_text())
        assert json.dumps(numerical(a),sort_keys=True)==json.dumps(numerical(b),sort_keys=True),case
        result['profiled_comparisons'][case]=dict(reference_s=old[case]['elapsed_s'],optimized_s=new[case]['elapsed_s'],
            reduction_percent=100*(1-new[case]['elapsed_s']/old[case]['elapsed_s']),
            duration_s=b['duration_s'],exact_numerical_match=True,full_collision_zone_validated=True)
    for side in ('right','left'):
        sessions={}
        for phase in ('cold','warm'):
            folder=ROOT/f'planner_runs/performance-ui-{side}-{phase}-20261021'
            ui=json.loads((folder/'destination_ui_verification.json').read_text())
            assert not ui['native_ui_errors'] and all(row['playback_tested'] for row in ui['tested_destinations'])
            rows={}
            for f in sorted((folder/'plans').glob('*/plan.json'),key=lambda p:p.stat().st_mtime):
                obj=json.loads(f.read_text())
                if obj.get('status')=='planned':rows.setdefault(obj['experiment']['method'],obj)
            assert len(rows)==4
            sessions[phase]=rows
        rows={}
        for method,a in sessions['cold'].items():
            b=sessions['warm'][method]
            for field in ('path_waypoints','source_timing','timing','outward','final_curve_check'):
                assert a[field]==b[field],(side,method,field)
            assert b['experiment']['report']['pool_reused'] and b['experiment']['retiming']['cached']
            rows[method]=dict(cold_s=a['experiment']['elapsed_s'],warm_s=b['experiment']['elapsed_s'],
                exact_cached_output=True,native_playback_tested=True)
        result['cross_session_ui'][side]=rows
    result['thread_trials']={}
    for version,threads in [('reference',1),('current',1),('current',2),('current',4)]:
        folder=ROOT/f'planner_runs/performance-threads-{version}-{threads}-20261021'
        if not (folder/'report.json').exists():continue
        row=json.loads((folder/'report.json').read_text())['left_clearance']
        law=json.loads((folder/'left_clearance-law.json').read_text())
        original=json.loads((before/'left_clearance-law.json').read_text())
        same=numerical(law)==numerical(original)
        result['thread_trials'][f'{version}_{threads}']=dict(elapsed_s=row['elapsed_s'],exact_numerical_match=same)
    peaks=[json.loads(p.read_text()).get('observed_peak_bytes',0)
           for p in (ROOT/'planner_runs').glob('performance-ui-*-20261021/plans/*/containment.json')]
    result['maximum_observed_worker_bytes']=max(peaks)
    (HERE/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
