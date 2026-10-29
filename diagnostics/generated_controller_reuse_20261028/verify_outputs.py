"""Audit saved fake-motor traces and preserved source; no device interfaces."""
import ast,csv,hashlib,json,socket,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
def no_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:raise AssertionError('Physical CAN forbidden')
sys.addaudithook(no_can)
from camera_playback.generated_planning import load
base=Path(__file__).resolve().parent
before=json.loads((base/'before_hashes.json').read_text())
preserved={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in before.items()}
assert all(preserved.values()),preserved
assert ast.dump(ast.parse((base/'before/generated_transport.py').read_text())) == ast.dump(ast.parse((ROOT/'camera_playback/generated_transport.py').read_text()))
reports=[]
for path in [base/'ui_report.json',base/'system_ui/ui_report.json']:
    obj=json.loads(path.read_text());assert obj['passed'];mission,curves=load(ROOT/obj['mission'])
    results=[];total=0;maximum=0.
    for trace in Path(obj['session']).glob('stream-*/trace.csv'):
        side=trace.parent.name.split('-')[1];c=curves[side]
        rows=list(csv.DictReader(trace.open()))
        if not rows:continue
        timestamps=np.array([float(r['monotonic_s']) for r in rows])
        errors=[]
        for r in rows:
            q=np.array([float(r[f'q{i}']) for i in range(1,8)])
            tau=float(r['path_time_s']);expected=c.at(tau)
            errors.append(float(np.max(abs(q-expected))))
        error=max(errors);assert error<1e-12
        maximum=max(maximum,error);total+=len(rows)
        result=json.loads((trace.parent/'result.json').read_text())
        assert result['executor']=='plan_drums.baseline.motion.Controller'
        if result['success'] and result['controller_phase']=='SETTLING_GOAL':
            assert float(rows[-1]['elapsed_s'])==c.duration
            np.testing.assert_array_equal(np.array([float(rows[-1][f'q{i}']) for i in range(1,8)]),c.at(c.duration))
        results.append(dict(file=str(trace.relative_to(ROOT)),samples=len(rows),max_curve_error_rad=error,
            median_interval_s=float(np.median(np.diff(timestamps))) if len(rows)>1 else None,
            ready_after_s=result['ready_after_s'],start_ack_after_s=result['start_ack_after_s'],
            success=result['success'],error=result['error']))
    reports.append(dict(report=str(path.relative_to(ROOT)),samples=total,max_curve_error_rad=maximum,traces=results))
report=dict(passed=True,hardware_access=False,preserved=preserved,transport_executable_ast_unchanged=True,ui_audits=reports)
(base/'preservation_and_trace_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(passed=True,preserved=len(preserved),audits=[{k:v for k,v in r.items() if k!='traces'} for r in reports]),indent=2))
for file in ['camera_playback/generated_controller.py','camera_playback/generated_stream.py','camera_playback/generated_transport.py',
             'tests/test_generated_stream.py','tests/test_generated_controller_startup.py','tests/generated_controller_ui_check.py']:
    compile((ROOT/file).read_text(),file,'exec')
