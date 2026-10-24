# Single arc continuity-priority correction

Date: 2026-10-21. Software only; no camera/CAN acquisition or physical arm commands.

## Cause and correction

The incident baseline run selected a corridor polyline with four forced stops
because its geometric score (1.9623458) was 0.81% better than a valid continuous
arc (1.9782324). The path-preserving retimer correctly retained those corner stops;
the mistake was allowing that small geometric-score advantage to override the
intended Single arc motion family.

The shared `single_arc` ranking now uses ordered groups:

1. Validated continuous Single arc proposals (`stop_at_corners=false`).
2. Validated continuous compact/unrestricted fallback routes, if group 1 is empty.
3. Validated segmented routes, only if both continuous groups are empty.

The existing broad-arc score and deterministic rounded-score/index tie-break
apply **within** each group. This is not a duration reward, a finite stop penalty
that can be outweighed, or recording imitation. All candidates are still subject
to their original feasibility checks. The other three experiment methods retain
their original source eligibility, scores and ordering. No search expansion,
new smoothing, additional hardware gate or UI control was introduced.

`baseline` and both arm experiment modes use this same shared rule. Console logs
report the group counts, each candidate's group, and an explicit fallback notice
when applicable. Saved selection metadata records `continuous_arc_first_v1`.
The existing source-hashed pool key includes every changed implementation file,
so newly launched calculations cannot silently reuse the old selection policy.
Archived plans are not rewritten; restart an already-open UI and Calculate again.

## Incident regression

The test uses the actual start/scene captured by the user's problematic baseline
run, not the experiment's different default pose. The corrected production
backend is rerun from those inputs, rather than merely substituting a saved path.

| Route on the incident inputs | Outward at 1x | Planned intermediate stops |
| --- | ---: | ---: |
| Previous selected corridor candidate 2 | 6.344722 s | 4 |
| Corrected selected continuous arc candidate 12 | 3.868824 s | 0 |

The production experiment worker on the identical inputs and validated candidate
pool returns **exactly equal path and timing payloads** to baseline. The improved
speed is a consequence of selecting the continuous path, not a new speed score.
The already-liked saved right experiment still selects candidate 14 and retains
its 4.127995-second path and timing coefficients exactly.
The modes still have different default inputs: baseline measures a new start and
scene, while the experiment reuses its saved ones. This change does not force a
live arm to use an old resting pose or promise identical routes for different
starts. Equality was verified with explicitly identical inputs.

The new incident curve is checked with the unchanged native full robot/gripper/
stick collision and zone3 checks (20 mm baseline obstacle padding plus existing
native allowances). Exact-curve preview/playback parity, motion extrema, endpoint
poses and prefix returns from 20%, 60% and 100% of the outward path are checked.
The test output deliberately lacks current-acquisition authorization and is
rejected by the real hardware loader; it is usable only for offline preview.

## Fresh planning verification

The contained software run completed all ten requested backend outputs (incident
baseline, matched-input experiment, and all four choices for each saved arm).
Each passed its exact timing/preview/endpoints and collision/zone checks. It used
the existing 3 GiB hard memory cap and peaked at 497,721,344 bytes (~474.66 MiB).
No CAN address family was permitted; camera construction was explicitly denied.

| Saved experiment | Single arc outward | Planned intermediate stops |
| --- | ---: | ---: |
| Right | 4.127995 s | 0 |
| Left | 2.748484 s | 0 |

The shared priority rule also selects a true continuous arc for the left
experiment. The timing dispatcher is unchanged: already-continuous paths use the
existing fixed-path retimer, while left corner-fallback paths retain bounded v2
smoothing. Other method rankings are tested against the prior formulas on all
three captured candidate pools. The correction does not change their preferences.

See `backend_report.json`, `backend_resources.json` and `backend.log` for results.

## Regression and native UI verification

**891 tests passed in 284.63 seconds**, including 27 new selection-specific tests:
the exact reported failure, both saved arms, unchanged other-method scores,
continuity tiers, missing metadata, fallback logging, duration independence,
stable ties, cache invalidation and preservation of the unchanged code.
The full suite includes the existing timing, collision/zone, calibration,
recording, baseline-controller, rendering and experiment-queue regressions.
Peak regression RSS was 1,696,708 KiB (~1.62 GiB); zero swaps were reported.

Native Qt/VTK tests passed on the new outputs:

- Baseline: all 21 three-window workflow checks, including explicit Calculate,
  preview/play/pause/scrub/speed, isolated measured-display updates, enabled
  recovery controls, complete simulated playback, hold/prefix-return and injected
  simulated CAN-fault recovery. All three simulated owners finished at rest in
  `RELAXED`, with `safe_to_close=true`. No automatic midair release occurred.
- Right and left experiments: all four methods selectable/playable; exact goal
  and reverse, speed, loop, seek, render layers/colors/navigation, settings-dialog
  cancel, actual contained-worker cancel/resume, cached switching without scene
  reupload. Both report empty error lists. Screenshots were inspected.
- The actual `./plan_drums.sh baseline --simulate ... --seconds 3` launcher exited
  successfully with no startup/render errors, no owner and all five buttons enabled.

See `regression.log`, `ui.log`, the three UI reports, `cli_simulation_report.json`,
screenshots and `simulated_cycles/`. The intentionally injected CAN-fault text in
the baseline UI report is a successful recovery test, not an unexpected failure.
IPC authentication tokens are omitted from the curated report. No test owner was
left running. The UI test process peaked at 616,344 KiB RSS (~0.59 GiB).

## Preservation and reproducibility

`preserved_sources.json` records the unchanged timing engines and policies,
proposal generator functions, collision/zone implementation, models, recordings,
UI and physical-controller code. The older corner-smoothing preservation test
now excludes the two intentionally updated ranking/logging files; the new tests
retain byte-level checks of their unchanged proposal-generation functions.
Historical hash manifests are not edited.

`selection_cases.json` retains the incident and both saved experiment candidate
metadata for small, device-free regression tests. It is test evidence, not a
runtime seed or an input to trajectory generation. Runtime paths still derive
only from the supplied state, scene and goal.

Reproduction (with the original incident capture still available locally):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Pick a NEW output directory; the script refuses to overwrite its results.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m diagnostics.single_arc_priority_20261021.verify_backend \
  planner_runs/single_arc_priority_recheck
bash diagnostics/single_arc_priority_20261021/run_tests.sh
```

`run_ui.sh` exercises the generated `planner_runs/single_arc_priority_20261021`
results using native windows and simulated motor owners only. Its output folders
should not already exist when rerunning the complete script. Original incident
and default-input recordings/captures are never modified.

These are software planning, rendering and simulated controller checks, not
physical tracking, holding-torque or real-world clearance measurements.
