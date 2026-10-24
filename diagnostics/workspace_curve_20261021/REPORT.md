# Workspace curve mini-experiment — 2026-10-21

## Outcome
Added **Workspace curve — Cartesian-first broad sweep** as a fifth choice in both experiment modes. The original four remain unchanged, and production `singlearc` is unchanged.

**Viable as a prototype, not a demonstrated replacement for Single arc.** All four saved destinations generated fully checked continuous trajectories. The left case improves gripper-path simplicity strongly; the right cases have mixed trade-offs. More room around obstacles can come with longer paths and extra joint travel. Human preview remains important.

Run the usual commands, select a destination, then select the new method:
```bash
./plan_drums.sh experiment --right
./plan_drums.sh experiment --left
```
Close/reopen an already-running experiment window to load the fifth menu entry. No camera or arm is required for the saved-input experiment.

## Common-metric comparison against Single arc
Gripper means the calibrated hand/gripper-base origin. Turning is total 3-D chord turning at a 5 cm spatial scale, not rotation of a particular motor. Lower turning is one geometric proxy, not a complete humanlike-motion score. Times are simulated one-way movement times, excluding the 2 s hold and return.

| Destination | Gripper turning, Single arc → new | Tip turning, Single arc → new | Gripper length, Single arc → new | Movement seconds, Single arc → new |
|---|---:|---:|---:|---:|
| destinationR1 | 122.9° → 116.4° | 128.7° → 84.1° | 0.410 → 0.476 m | 4.020 → 4.205 |
| destinationR2 | 104.5° → 110.1° | 137.4° → 105.6° | 0.343 → 0.378 m | 3.224 → 3.526 |
| destinationR3 | 63.1° → 97.5° | 169.6° → 170.7° | 0.323 → 0.386 m | 2.558 → 3.843 |
| destinationL1 | 43.0° → 2.4° | 43.6° → 36.2° | 0.195 → 0.187 m | 2.834 → 2.840 |

| Destination | Joint reversals, Single arc → new | Extra joint travel, Single arc → new | Gripper curvature variation, Single arc → new | Distal clearance p10, Single arc → new |
|---|---:|---:|---:|---:|
| destinationR1 | 6 → 3 | 54.1° → 100.3° | 17.75 → 11.10 | 37.5 → 42.5 mm |
| destinationR2 | 4 → 5 | 34.5° → 90.5° | 5.43 → 8.55 | 32.5 → 47.5 mm |
| destinationR3 | 2 → 4 | 17.8° → 153.5° | 6.35 → 7.22 | 32.5 → 57.5 mm |
| destinationL1 | 4 → 2 | 35.5° → 44.0° | 2.49 → 0.15 | 27.5 → 32.5 mm |

Clearance is the existing world-only distal-link 5 mm padding-band estimate, capped at 100 mm. It is **not** exact Euclidean clearance or physical safety certification. The p10 statistic describes the lower tenth of sampled clearance values, not the minimum. Full collision/zone checks, including stick and custom grippers, remain mandatory and unchanged.

Interpretation:
- **L1:** substantially straighter gripper motion, less tip turning and fewer reversals at essentially the same duration. Extra joint travel still increases slightly.
- **R1:** less tip turning and fewer reversals, but a longer gripper path and substantially more joint travel. Not an unqualified improvement.
- **R2:** less tip turning and more lower-tail clearance, but more gripper turning, distance, reversals and joint travel.
- **R3:** larger clearance estimate, but more gripper turning/distance and much more joint travel; slower too. This is the clearest warning against declaring the fork generally better.

[Path traces](paths.png) show gripper-base curves (solid) and stick-tip curves (dashed). Obstacles are omitted from this comparison plot; use the normal preview for the full frozen scene.

## Bounded development
Five small shared-setting variants were screened on all four destinations. No destination IDs or recording interior motion enter the generator. The scene, per-arm starts, endpoints, zones, models, margins and timing policies are fixed.

| Variant | Feasible destinations | Decision |
|---|---:|---|
| initial | 0/4 | Too little route diversity; rejected |
| moderate | 4/4 | Useful right-arm alternatives; retain as part of shared search |
| broad | 4/4 | Preserves the strong left candidate; retain as part of shared search |
| tool | 2/4 | Lost R1/R2; not retained |
| posture | 2/4 | Lost R1/R3; not retained; stop tuning |

The final method explores the moderate and broad route priors together and applies the same ranking on every case. It does not choose a preset based on arm or destination name. The optimizer jointly fits a cubic Bezier target and a continuous nine-knot joint trajectory, with secondary stick-tip/joint regularization. The played quintic is also screened at 801 geometry samples against the 25 mm geometric fit tolerance; this fit screen is not a continuous-distance certificate. The fit tolerance does not change collision margins.

Scoring uses actual generated wrist/tip turning, curvature variation, detour length, joint backtracking/coordination and the existing soft clearance cost. Timing is applied afterward using the unchanged dispatcher. There is no segmented or Single arc fallback under the new method name. A failed fit stays a normal no-feasible-route result, and the other choices/retry controls remain available.

All available cases were development cases, not held-out tests. In particular there is only one left-arm destination. These results show viability on the saved scene, **not** proven generalization or physical tracking.

## Verification
- 20/20 final method/destination workers planned successfully (five methods × R1/R2/R3/L1).
- All 20 actual analytic playback laws independently passed dense collision/zone checks, exact endpoints and motion-limit checks.
- Native Qt/VTK: all 20 complete outward/hold/reverse animations played; play/pause, seek, speed, loop, goal/start, rendering controls, save-view, destination changes, refresh, cancellation/retry, empty/invalid destinations and a labelled no-route UI fixture passed.
- Left UI also started with Workspace curve selected before selecting the destination. Right UI retained the usual default method.
- Only saved state/scene inputs; no camera, CAN access or physical commands. Workers retain 3 GiB RAM / 128 MiB swap limits and CAN exclusion.
- Largest observed planning-worker memory: 345.5 MiB. This does not include all application/editor/browser memory.
- 293 regression tests plus an additional byte-exact routing/label preservation test passed. `regression.log` contains the runs. The only warning is the pre-existing SLSQP trial time-density bounds warning in an old/new numerical parity test; no motor angles were clipped. Original algorithm/timing/native-check/model/recording/actuator bytes are preserved. Removing the fifth-method additions also reconstructs the three edited existing experiment modules byte-for-byte.

## Files
- `comparison.json` / `comparison.csv`: common metrics and all 20 plan paths; evaluated from actual retimed curves, not optimizer objectives.
- `tuning_summary.json`, `study_design.json`, `prototype_v1.py`, `trial.py`: retained trials and reproducible settings, including discarded variants.
- `cases.json`, `before_hashes.json`, `final_sources.json`: inputs and preservation/provenance.
- `ui_right.json`, `ui_left.json`, `verification.json`: native UI checks and resource containment.
- `reference_runs.py`, `final_runs.py`, `report.py`, `run_tests.sh`: software-only reproduction programs.
- Full local plans and caches: `planner_runs/workspace-curve-study-20261021/`; large generated run trees remain local, not added to Git.
