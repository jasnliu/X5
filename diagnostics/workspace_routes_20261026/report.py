"""Assemble compact evidence; original captures and full runs stay untouched."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
D=Path(__file__).resolve().parent;ROOT=D.parents[1]
primary=json.loads((D/'final_primary/summary.json').read_text());extra=json.loads((D/'final_extra/summary.json').read_text());audit=json.loads((D/'audit/summary.json').read_text())
baseline=json.loads((D/'baseline/summary.json').read_text())
def show(rows):
 return '\n'.join(f"| {r['case']} | {r['status']} | {r.get('movement_s',0) or 0:.3f} | {r['elapsed_s']:.3f} |" for r in rows)
def geom(r):
 g=r['candidate']['workspace']['gesture']
 return f"| {r['case']} | {g['reversal_count']} | {g['joint_excess_deg']:.1f} | {g['wrist']['detour_ratio']:.3f} | {g['tip']['detour_ratio']:.3f} |"
inputs={}
for name,path in [('incident','planner_runs/20261026T040710Z-experiment-f81b35/acquisition'),('second','planner_runs/20261026T033257Z-experiment-225d96/acquisition'),('reference','diagnostics/r1_r2_wrist_speed_20261024/session/acquisition')]:
 root=ROOT/path;scene=json.loads((root/'camera/scene.json').read_text());files={}
 for rel in ('measured_start.json','camera/scene.json',*[f'camera/{k}' for k in scene['files_sha256']]):
  p=root/rel;sha=hashlib.sha256(p.read_bytes()).hexdigest();files[rel]=sha
  if rel.startswith('camera/') and rel!='camera/scene.json':assert sha==scene['files_sha256'][p.name]
 inputs[name]=dict(path=path,files_sha256=files,captured_utc=scene['captured_utc'],calibration_sha256=scene['calibration_sha256'])
(D/'input_provenance.json').write_text(json.dumps(inputs,indent=2)+'\n')
rootcause=json.loads((D/'root_cause.json').read_text())
endpoints=[]
for target in ('R1','R2'):
 data=json.loads((D/f'evidence/incident_right_default_{target}.json').read_text());q=data['source']['request']['goal']
 endpoints.append(dict(destination=target,joints_deg=np.degrees(q).tolist(),
      min_joint_limit_margin_deg=float(np.degrees(np.minimum(np.array(q)-data['limits_lower'],np.array(data['limits_upper'])-q)).min()),
      endpoint_fk=audit[0 if target=='R1' else 1]['endpoint_fk']))
(D/'endpoints.json').write_text(json.dumps(endpoints,indent=2)+'\n')
raw={}
for target in ('R1','R2'):
 rows=json.loads((D/f'round_prototype/incident_right_default_{target}/summary.json').read_text())
 valid=[r for r in rows if r.get('valid')];best=min(valid,key=lambda r:r['score'])
 raw[target]={k:best[k] for k in ('width','score','metrics','reversals')}
(D/'discarded_rrt_quality.json').write_text(json.dumps(raw,indent=2)+'\n')
quality='\n'.join(geom(r) for r in primary)
text=f'''# Workspace route recovery — 2026-10-26 UTC

## Result and scope

**Verified:** normal experiment generates validated routes to the current R1 and
R2 from both preserved real-camera acquisitions that previously failed. All 16
final planning cases passed; all 20 component legs passed a separate independent
1 ms native collision/zone audit. Native Qt/VTK selection, full outward playback,
scrubbing, cancellation and worker cleanup also passed for both targets.

Everything was **offline**. No camera opened, CAN queried, motors enabled/disabled,
grippers adjusted or arms moved. Sources are the exact actual Gemini captures,
NOT synthetic scenes. No saved route is used as a runtime template/cache.

Changes are confined to normal experiment proposal orchestration, its new
`workspace_recovery.py`, menu description, documentation, tests and this study.
URDF, calibration, captured data, destinations, hardware planners, motion limits,
all timing implementations, shared `workspace_curve.py`, and `--fast` are unchanged.

## What failed, and is R1 sensible?

R1 and R2 both pass endpoint joint-position bounds, collision and selected zone
checks in the incident snapshot. R1 is a bent-elbow pose (~90 degrees at J4), not
an out-of-range or impossible saved endpoint in this model. Its calibrated stick
tip is approximately (0.519, -0.370, 0.497) m in robot coordinates; R2's is
(0.630, -0.092, 0.372) m. See `endpoints.json` and the goal screenshots.

Before this change, the incident scene rejected **10/14 proposals** for continuous
joint-bound violations and **4/14** for collision/zone screening, for each target.
The original fitter bounded its knots, but subsequently reinterpolated them into
a different played spline. In the R1 diagnostic, J2 undershot its calibrated
lower bound between knots (up to 0.1793 degrees). We did not hide this by loosening
tolerance. The other proposals hit observed depth with the drumstick near the
initial departure. Extra timing optimization cannot repair invalid geometry.

This is separate from the later FCL startup error caused by the passive left
arm's captured position exceeding its joint bound. That invalid capture was NOT
used to make passing routes and its limit enforcement remains unchanged.

## Adaptation

1. Try original moderate/strong workspace proposals first, unchanged.
2. Only if they fail, fit the **exact played quintic B-spline**. Bounded controls
   and a nonnegative partition-of-unity basis enclose the entire curve in joint
   limits. Three fixed controls at each end enforce exact rest endpoints; the
   interior is C4. Independent polynomial-extrema checks remain mandatory.
3. If required, try four symmetric distal-posture guides, J6/J7 offsets
   (-0.5,+0.3), (-0.5,-0.3), (+0.5,+0.3), (+0.5,-0.3) radians. These are generic
   optimization initializations, not drum-specific waypoints. Restore the actual
   endpoint controls before screening. Reject out-of-bounds guides rather than
   clipping them. Candidate diagnostics distinguish virtual fit error from the
   actual final curve.
4. Validated late-control blends reduce needless wrist steering while preserving
   the initial departure. Rank actual curves with the original geometry/clearance
   objective. If one direction cannot find a curve, independently fit from the
   opposite endpoint, then reverse its parameterization. Nothing is copied from
   an outbound mission leg or a recording.
5. Retain the existing Balanced per-joint CPU timing and final 2 ms validation.
   Search remains lazy, serial, cancellable and capped at 20 recovery seconds
   within the per-leg search budget. Validation/ranking can add wall time.

Raw RRT guides and various rounding/posture/extra-clearance alternatives were
explored but rejected as defaults: their valid paths bent too much. On the SAME
incident endpoints, the best tested compact-kernel RRT curves had geometry-only
scores 8.07 (R1) and 7.14 (R2), versus 2.31 and 2.71 for the selected new curves
(excluding clearance penalties for this comparison). No obstacle or limit was
removed to achieve the improvement. Full rejected trials remain locally saved.

## Cold-process CLI results

Wall time includes launcher, setup/initialization, fresh geometry, timing, dense
validation and process cleanup. It excludes live acquisition because every test
was deliberately offline. Each row used a fresh process/output folder; no result
or timing cache/CPU-reference qualification. Ordinary OS/library file caches
were not flushed. Do not call these physical movement measurements.

### Before

| Case | Status | Predicted outward movement (s) | Cold calculation (s) |
|---|---|---:|---:|
{show(baseline)}

### Final source — primary reproductions

| Case | Status | Predicted outward movement (s) | Cold calculation (s) |
|---|---|---:|---:|
{show(primary)}

### Transfers, cycles, sensitivity and left regression

| Case | Status | Predicted outward movement (s) | Cold calculation (s) |
|---|---|---:|---:|
{show(extra)}

For multi-checkpoint rows, “outward” includes the intermediate 2 s R1 hold, but
not the final 2 s R2 hold or separately planned return. The incident cycle has
3.370 s to R1, 1.657 s R1→R2, and 3.343 s R2→start: **8.371 s movement / 12.371 s
including both holds**. The second capture also passes a complete cycle.
The +/- cases are synthetic 0.005-rad (~0.286-degree) perturbations of all seven
selected-arm start AND goal joints against the unchanged actual incident scene;
they are not new measurements. The original acquired self-mask is preserved.

## Shape quality and preservation

| Case | Significant joint reversals | Excess joint travel (deg) | Wrist length / straight distance | Tip length / straight distance |
|---|---:|---:|---:|---:|
{quality}

All audited trajectories have **zero internal stops and zero deep slowdowns**
under the existing joint-speed-norm diagnostic. New R1/R2 have three significant
joint reversals vs four for the historical successful reference. New R2 has a
straighter wrist path but needs more stick sweep (tip detour 1.239 vs 1.108 in the
older reference). This is comparable broad motion, not a claim every metric is
better. The scene AND captured start differ from the historical reference, so
these numbers do not isolate an algorithmic movement-time gain. The reference
R1/R2 cases on identical old inputs retain bit-identical geometric AND timing
coefficients, not merely similar durations (`audit/reference_preservation.json`).

Native screenshots: [R1 goal](ui/R1_4.png), [R2 goal](ui/R2_4.png).
Mid-route and quarter-route screenshots also remain locally available. Both
paths were played in the actual Qt/VTK experiment window, not only plotted.

## Verification

- 16/16 final fresh-process cases; independent audit of 20/20 legs.
- {sum(r['collision_zone']['samples_checked'] for r in audit):,} states at <=1 ms plus curve/clock events,
  native edge subdivision <=0.003 rad L1, unchanged full-arm/stick/zone checks.
- Independent original polynomial-root V/A/jerk extrema; maximum fraction of any
  requested limit = {max(r['maximum_limit_fraction'] for r in audit):.9f}.
- Continuous joint-position extrema, exact endpoints, zero endpoint velocity
  and acceleration; no geometry change during timing.
- Original acquisition pose for self-mask on all legs/simulated starts; captured
  passive arm unchanged; scene, calibration and input payload hashes checked.
- Negative native test: the invalid passive-left snapshot still fails on its
  original calibrated joint limits before any recovery search (`negative_check.json`).
- Worker containment: 3 GiB RAM / 128 MiB swap, no AF_CAN; bounded serialized jobs.
- Native UI: selection, real playback, five scrubs per target, unchanged static
  scene mesh, Cancel, fresh retry and close-reaping all passed.
- New recovery regression suite: 19/19 passed. Broader selected suite: 277
  passed, 7 failed. All seven are pre-existing historical hash-preservation
  assertions referring to the earlier `depth.py` diagnostics change or the
  recaptured R1 destination. Both files match the task-entry hashes exactly;
  assertions were NOT weakened or rewritten. See `preexisting_test_failures.json`
  and complete `pytest.log`. The task-specific 1,305-file preservation audit is
  in `preservation.json`.

This establishes software feasibility under the approved observed-obstacle
policy, NOT physical safety/speed or clearance through unobserved space. New
live captures can legitimately differ. No guarantee of a global optimum or
universal endpoint reachability is made.

## Reproduction / evidence

Normal user command is unchanged: `./plan_drums.sh experiment --right` (or left).
It still captures fresh inputs on launch. To replay the exact failing scene
without camera/encoder access:

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment --right --offline \\
  --state planner_runs/20261026T040710Z-experiment-f81b35/acquisition/measured_start.json \\
  --scene planner_runs/20261026T040710Z-experiment-f81b35/acquisition/camera/scene.json \\
  --destinations R1 R2 --headless --output /tmp/x5-recovery-recheck-unique
```

`run_cases.py NEW_TAG` repeats six primary cold runs; add `extra_cases.json` for
the transfer/sensitivity suite. Use a fresh tag/output directory. Source ROS and
workspace environments as documented in AGENTS.md. `audit.py` audits final saved
runs and `ui_check.py` exercises native UI (its output directory must be new).
Original raw captures/full run outputs remain local and unmodified; compact
summaries, curve/law regression fixtures, scripts, hashes and goal screenshots
are curated. The regression fixtures are evidence only; production never reads
them. `input_provenance.json` identifies exact actual-camera files and hashes.
'''
(D/'REPORT.md').write_text(text)
