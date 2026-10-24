"""Summarize device-free final-checkpoint-to-start return verification."""
import json
import re
import shutil
from pathlib import Path

from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment.checkpoints import RETURN_POLICY,SCHEMA

folder=ROOT/'diagnostics/checkpoint_return_20261021'
rows=[];peak=0
old_report=json.loads((ROOT/'diagnostics/checkpoints_20261021/verification.json').read_text())
old_plans={(r['side'],r['method'],r['checkpoints']):r['plan'] for r in old_report['results']}
for side in ('right','left'):
    run=ROOT/f'planner_runs/checkpoint-return-ui-{side}-20261021'
    report=json.loads((run/'checkpoint_ui_verification.json').read_text())
    assert not report['native_ui_errors'] and len(report['checks'])==15
    assert report['motor_commands']==0 and report['camera_access'] is False
    for row in report['checks']:
        assert row['status']=='planned' and row['complete_cycle_played'],row
        plan_path=Path(row['path']);plan=json.loads(plan_path.read_text())
        request=json.loads((plan_path.parent/'backend_request.json').read_text())
        old=json.loads(Path(old_plans[side,row['method'],row['checkpoints']]).read_text())
        assert plan['outward']==old['outward'],'Outward path/timing changed'
        if row['checkpoints']==1:
            assert plan['cycle']==old['cycle'] and plan['timing']==old['timing']
        else:
            assert [s['timing'] for s in plan['checkpoint_sequence']['segments']]==[s['timing'] for s in old['checkpoint_sequence']['segments']]
        assert plan['motion_authorized'] is False and plan['hardware_execution_available'] is False
        result=dict(side=side,method=row['method'],checkpoints=row['checkpoints'],
                    outward_with_holds_s=plan['duration_outward_s'],
                    full_cycle_s=plan['cycle'][-1]['time_s'],plan=str(plan_path))
        if row['checkpoints']>1:
            assert row['new_return_checked']
            sequence=plan['checkpoint_sequence'];back=sequence['return_leg']
            assert sequence['schema']==SCHEMA and sequence['return_policy']==RETURN_POLICY
            assert back['start']==request['goal'] and back['goal']==request['start']
            assert json.loads(Path(back['plan']).read_text())['status']=='planned'
            back_request=json.loads((Path(back['plan']).parent/'backend_request.json').read_text())
            assert back_request['scene_robot_start']==request['start']
            assert back_request['start']==request['goal'] and back_request['goal']==request['start']
            assert plan['final_curve_check']['valid']
            assert all(leg['samples_checked']>0 and leg['max_time_step_s']<=.002
                       and leg['edge_joint_l1_step_rad']<=.003
                       for leg in plan['final_curve_check']['legs'])
            assert len(plan['final_curve_check']['legs'])==row['checkpoints']+1
            result['independent_return_s']=back['timing']['duration_s']
            result['return_plan']=back['plan']
        limits_path=plan_path.parent/'containment.json'
        if limits_path.exists():
            limits=json.loads(limits_path.read_text())
            assert limits['MemoryMax']=='3221225472' and 'AF_CAN' not in limits['RestrictAddressFamilies']
            peak=max(peak,limits['observed_peak_bytes'])
        rows.append(result)
    shutil.copy2(run/'checkpoints.png',folder/f'ui_{side}.png')
shutil.copy2('/tmp/x5-checkpoint-return-regression.log',folder/'regression.log')
log=(folder/'regression.log').read_text()
assert not re.search(r'\d+ failed|\d+ errors?',log)
passed=int(re.search(r'(\d+) passed',log)[1])
shutil.copy2('/tmp/x5-checkpoint-return-supplemental.log',folder/'supplemental.log')
passed+=int(re.search(r'(\d+) passed',(folder/'supplemental.log').read_text())[1])
atomic_json(folder/'verification.json',dict(native_cycles=len(rows),independent_return_cycles=20,
    regression_tests_passed=passed,all_5_methods_both_arms=True,checkpoint_counts=[1,2,3],
    camera_access=False,motor_commands=0,peak_worker_bytes=peak,results=rows,
    left_extra_endpoints='Two test-only +0.015/+0.030 rad joint-1 variations; user destination folder unchanged',
    controls='add/remove/max/exclusion/order/refresh/cancel/retry/play/pause/scrub/speed/loop/goal/start',
    single_destination='Legacy behavior unchanged',outward_samples_and_timing_identical_to_before=True,
    return_policy=RETURN_POLICY))
text=f'''# Final-checkpoint return correction — 2026-10-21

Multi-checkpoint experiment playback now plans an additional independent leg from
its final selected checkpoint to the original saved start. It no longer reverses
the outward checkpoint sequence or revisits earlier checkpoints as required stops.
The same selected planner, timing policy, original frozen scene/self-mask, passive
arm, zones and collision margins apply to the return. Two-second outward checkpoint
holds remain. A blocked return is reported without silently substituting a reversed
checkpoint route; retry and selection controls remain usable.

Single-destination previews and all hardware modes are unchanged. Endpoint-only
return targets are private result artifacts, not additions to user destinations.
Old reverse-route aggregate previews use a different schema and are not reused;
unchanged individual-leg searches and timing remain cacheable.

## Verification

- {passed} regression tests passed; source-preservation checks cover hardware,
  path generators, timing, model, scene, calibration, saved poses and destinations.
  One existing SLSQP trial-step bounds warning remains (also in the prior report);
  no test failures. This is not robot encoder clipping.
- 30 native Qt/VTK full-cycle playback checks: five methods, both arms, one/two/three
  destinations. All planned and played successfully. Twenty multi-checkpoint cycles
  used newly planned returns with complete collision/zone proofs.
- All 30 outward sample sequences and analytic timing laws equal the previous
  successful checkpoint runs exactly; all single-destination cycles also match.
- Exact return-law evaluation, final-to-original endpoints, original snapshot mask,
  outward holds, end-at-start, play/pause, scrub, speed, loop, Go to goal/start,
  add/remove/max/exclusion, cancellation/retry and stale-result rejection checked.
- Failure injection verifies blocked-return recovery without partial playback.
- No camera access, CAN access or motor commands. Left's additional endpoints are
  small test-only variations because only one user destination currently exists.
- Worker memory cap remains 3 GiB; observed peak {peak/2**20:.1f} MiB. Serial work and
  cancellation remain intact. Multi-checkpoint time ceiling includes the new return
  leg, without reducing solver refinement quality.

The older `diagnostics/checkpoints_20261021/REPORT.md` records the superseded
reverse-through-checkpoints behavior. This report and `verification.json` document
the correction. Large plans/caches stay local under `planner_runs/`.

## Full-cycle timing evidence

| Arm | Method | Checkpoints | To final incl. holds (s) | New return (s) | Full cycle (s) |
|---|---|---:|---:|---:|---:|
'''
for row in rows:
    back=f"{row['independent_return_s']:.3f}" if 'independent_return_s' in row else 'legacy reverse'
    text+=f"| {row['side']} | {row['method']} | {row['checkpoints']} | {row['outward_with_holds_s']:.3f} | {back} | {row['full_cycle_s']:.3f} |\n"
(folder/'REPORT.md').write_text(text)
print(f'PASS: {passed} regression tests, {len(rows)} native cycles, 20 independently planned returns')
