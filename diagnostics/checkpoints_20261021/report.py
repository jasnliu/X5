"""Summarize completed, device-free checkpoint verification; fails if incomplete."""
import json
import shutil
from pathlib import Path
from drum_planner.common import ROOT,atomic_json,digest

D=ROOT/'diagnostics/checkpoints_20261021'
reports={};rows=[];cli={};scene={};peak=0
for side in ('right','left'):
    root=ROOT/f'planner_runs/checkpoints-ui-{side}-20261021'
    report=json.loads((root/'checkpoint_ui_verification.json').read_text())
    assert not report['native_ui_errors'] and len(report['checks'])==15
    assert all(r['status']=='planned' and r.get('complete_cycle_played') for r in report['checks'])
    for r in report['checks']:
        plan=json.loads(Path(r['path']).read_text())
        assert plan['motion_authorized'] is False and plan['hardware_execution_available'] is False
        sequence=plan.get('checkpoint_sequence',{})
        rows.append(dict(side=side,method=r['method'],checkpoints=r['checkpoints'],
            outward_with_intermediate_holds_s=plan['duration_outward_s'],
            movement_outward_s=sequence.get('movement_outward_s',plan['duration_outward_s']),
            full_cycle_s=plan['cycle'][-1]['time_s'],plan=r['path']))
        limits_path=Path(r['path']).parent/'containment.json'
        if limits_path.exists():
            limits=json.loads(limits_path.read_text())
            peak=max(peak,limits['observed_peak_bytes'])
            assert limits['MemoryMax']=='3221225472' and 'AF_CAN' not in limits['RestrictAddressFamilies']
    reports[side]=report
    shutil.copy2(root/'checkpoints.png',D/f'ui_{side}.png')
    cli[side]=json.loads((ROOT/f'planner_runs/checkpoints-cli-{side}-20261021/experiment_view_report.json').read_text())
    assert cli[side]['generations']==0 and cli[side]['selected_checkpoints']==[None] and cli[side]['motor_commands']==0
    scene[side]=json.loads((ROOT/f'planner_runs/checkpoints-scene-{side}-20261021/verification.json').read_text())
    assert scene[side]['legacy_native_parity'] and scene[side]['snapshot_mask_invariant']
shutil.copy2('/tmp/x5-checkpoints-regression.log',D/'regression.log')
log=(D/'regression.log').read_text();assert '309 passed, 1 warning' in log
before=json.loads((D/'before_hashes.json').read_text())
changed=[p for p,h in before.items() if not p.startswith('tests/') and digest(ROOT/p)!=h]
expected=['drum_planner/experiment/app.py','drum_planner/experiment/worker.py',
          'drum_planner/experiment/algorithms.py','drum_planner/cpp/experiment_oracle.cpp']
assert set(changed)==set(expected),changed
atomic_json(D/'verification.json',dict(native_cycles=30,all_5_methods_both_arms=True,
    checkpoint_counts=[1,2,3],ui_reports=reports,cli_startup=cli,scene_checks=scene,
    regression_tests_passed=309,observed_worker_peak_bytes=peak,camera_access=False,motor_commands=0,
    source_preservation_changed=changed,results=rows))
text='''# Experiment checkpoints — 2026-10-21

## Delivered
Only the right/left experiment UI gains optional ordered checkpoints. First
destination is checkpoint 1; maximum three total. Add/Remove last controls,
arm-specific dropdowns, immediate-previous exclusion (A → B → A is allowed),
unselected startup, automatic serial calculation once every shown box is chosen,
cancellation/invalidation when selections change, and unchanged one-destination
behavior. Each leg starts at the preceding exact full joint endpoint.

Each checkpoint holds for two nominal seconds, then the complete route reverses
back to saved rest, including intermediate holds on return. The usual preview
speed scales the clock. Every leg uses its own unchanged analytic timing law;
there is no interpolated shortcut or spline across stopped joins.

The original five generators/rankers, motion limits, timing implementations,
models/tools, zones, collision margins and hardware source files are preserved.
The shared pool key gains an optional snapshot-pose identity. The experiment-only
native oracle pins perception self-masking to the original saved camera/robot
snapshot while allowing later planning starts to move. Legacy requests are
unchanged; production `moveit_plan.cpp` is untouched.

## Verification
- **309 tests passed**. One existing SLSQP time-density trial-bounds warning,
  not clipping of robot joint angles; numerical old/new timing parity passes.
- **30 full native Qt/VTK cycles passed**: five methods × one/two/three
  checkpoints × both arms. Tested add/remove/max, dropdown exclusion, earlier
  selection changes, refresh, rapid cancellation/reaping, retry, stale-route
  rejection, method switching, play/pause, timeline, speed, loop, final goal/start,
  exact holds/return and retained scene geometry. No native UI exceptions.
- All final generated legs passed the normal dense collision/zone and analytic
  motion-limit checks. New composition checks exact joins and rest endpoints.
  No-feasible-leg recovery, frozen-file tampering, cache identities and float
  timestamp/endpoint edge cases have additional unit coverage.
- Both public `./plan_drums.sh experiment --right/--left --seconds 2` commands
  launched and closed with no error, zero automatic calculations, and no selected
  destination. Full interactive playback was tested separately above.
- Native FCL parity: legacy binary versus new binary produced identical metadata
  and 31 collision-query results per arm. Changing a later leg start while keeping
  the snapshot pose gave the same self-mask and collision results; the regression
  also detects the incorrect alternative of masking at the new simulated pose.
- **No camera access, CAN access or physical motion.** Saved inputs only.

Left currently has only `destinationL1.json`. Its additional checkpoints in the
native test were explicitly test-only joint-1 variations of +0.015/+0.030 rad,
kept under `planner_runs`, not added to the user's destinations folder. Right
used the three real saved destinations R1 → R2 → R3. Original destinations and
recordings remain byte-identical.

## Findings resolved during verification
The first right test hit the old 540-second combined worker limit during a
particularly slow final 192-control Single arc retiming stage. Multi-checkpoint
workers now allow the search budget plus 600 seconds of postprocessing per leg
(2160 s for three legs at 120 s search each). This changes only the outer timeout,
not solver iterations, tolerances, path selection, motion limits or checks.
The fresh final right run passed. Completed legs still use content-checked
caches. Memory remains 3 GiB, swap 128 MiB, CAN excluded, and Cancel/reaping
remain available; `worker_limits.json` records actual OS properties.

Checkpoint arrival poses are returned exactly despite floating-point subtraction
roundoff. Display samples that collapse to one absolute timestamp are merged;
this never changes the underlying analytic curve or timing law.

## Evidence
`verification.json` contains all 30 result paths, controls tested, per-case times,
CLI startup and native scene-parity results. `ui_right.png` / `ui_left.png` are
actual native previews. `before_hashes.json` and `reference/` preserve the
pre-change audit; raw plans/caches/logs remain local under `planner_runs`/`/tmp`.

Run `tests/experiment_checkpoints_ui_check.py` with sourced ROS/workspace,
`PYTHONPATH=$PWD`, planner Python, `--side right|left --output NEW_RUN_FOLDER` for
native reproduction. `tests/checkpoint_scene_check.py` reproduces scene invariance;
`--old-binary` optionally checks a pre-change binary (local binary not committed).
'''
text+=f'\nLargest observed final planning-worker memory: **{peak/2**20:.1f} MiB** (not the whole desktop/UI).\n'
(D/'REPORT.md').write_text(text)
print('VERIFIED',len(rows),'native cycles; 309 tests; peak MiB',peak/2**20)
