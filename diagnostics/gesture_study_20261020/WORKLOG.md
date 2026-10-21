# Work history — completed; see final audit below

Root task: user approved an unattended, offline experiment; keep standalone baseline unchanged, scene/pose from prior comparison, record3 endpoint only, small perturbations, zone3 and baseline margins unchanged. Keep several candidates; replace previous experiment menu. No hardware/camera commands.

Memory consulted MEMORY.md line708 about offline vs physical evidence. Final must include CODEX_NOTIFY_COMPLETE comment then memory citation last (later memory instruction). No agents authorized/spawned.

Current public survivors (config/app/CLI already integrated): progress, sweep, single_arc, clearance. Default first progress. Shared generator `experiment/search.py` and `gesture.py`; cheap FK/corridor options added ONLY to experiment_oracle.cpp, not moveit_plan.cpp. Former five public IDs removed; fluidity.py retains reused diagnostic utilities. `prepare` defaults to hash-bound frozen state and scene in default_inputs.json, no camera. 120-second pool default. Existing UI retained and labels/settings updated.

Initial screening: 4 batches x8 hypotheses; `development/*/evaluation.json`. Strongly favored compact endpoint corridors. Refined: 15 batches (5 development tasks x3 seeds), `refine/*/evaluation.json`; all valid. Final four chosen prospectively in selection_before_holdout.md. Raw compact mode folded into common initializer/fallback, not public. Do NOT retune against heldouts.

Active: `batch.py heldout` running in unified exec session 13179, stdout heldout_batch.log. 18 batches total (6 reserved tasks x seeds19,127,509), ~45–60s/batch. It automatically runs evaluate.py at end. Everything serial, systemd per-job 3GiB/128MiB/96tasks, no AF_CAN. Sources frozen in frozen_before_holdout.json. Avoid editing gesture/search/algorithms/oracle generation during holdout. Study.py source only had an unused stage1 compatibility correction before freeze.

Completed tests: initial broad regression 514 passed in112s (regression_initial.log), focused later48 passed; then one extra prepare-no-camera test added, needs rerun. New tests/test_planner_gesture.py. Final method-registry count changed5→4 and queue tests adjusted; focused48 passed. Need final broad run again after all edits.

Remaining:
1. Wait for heldout batch, run summarize.py (writes summary.json). Review success and differences, no aggregate imitation+quality score. Need ensure expected four remain viable.
2. Independent production recheck helper verify_selected.py: run in contained Job for refine and heldout, maybe all33 cases (~3min), serial. Uses original acquisition req for self-mask, selected current public IDs, dense2ms, original C++ binary edge checks, polynomial extrema, endpoints and cycle. Output directories verification_refine/verification_heldout. Needs execute/test script (not yet run).
3. Run actual `./plan_drums.sh experiment --headless --method all --output <NEW folder>` to exercise real default saved inputs and all four modes. Native GUI integration `tests/check_gesture_ui.py SESSION REPORTDIR` with PYTHONPATH=. and ROS env. Test script injects explicit filesystem fixture for Recapture success/failure; NEVER real camera. It tests automatic caching, all full playback cycles, play/pause/seek/speed/loop, mesh reuse, camera navigation/layers/colors/save, settings preset/cancel, cancel/resume, budget invalidation, regeneration and recapture fixture/requeue. It has not been run yet, may need fixes. tests/check_fluidity_ui.py is a compatibility shim.
4. Inspect actual saved UI screenshot via view_image; fix layout if needed. Add screenshot at meaningful pose if desired. Test initial prepare no device confirmed unit only; actual CLI pending.
5. Generate report plots (matplotlib) and `REPORT.md` with final metrics/eliminations, separate similarity scores, comparison to ORIGINAL saved standalone baseline (NOT poor exploratory RRT control). References in reference_geometry.json / reference_similarity.json. Baseline extra69.17deg, wristdetour1.4708, wristturn248.8deg, tip+Xovershoot13.34cm, tip shape residual17.717cm, jointresidual8.908deg; duration5.778s. Record extra31.39, detour1.169, tip+X1.324cm but different start and collision-buffer failures. No physical speed/safety claims. Some new low-backtrack corner-stop winners are slow (20–115s); report honestly (speed not selection criterion).
6. Final broad regression same files from regression_initial.log command; include all planner/baseline/render unit tests, no hardware. Final preservation audit compare before_hashes.json: baseline dir, planning.py, style.py, moveit_plan.cpp, plan_drums.sh, start_beat.sh, recordings, grippers, camera calibration, model, zone3 unchanged. CLI changed experiment-only, docs targeted. Check model default/current inputs hashes, source freeze core equals final. Check no orphan experiment jobs, memory reports peak (so far ~350–400MB workers), no OOM.
7. Update verification.json and docs evidence. DRUM_PLANNER_EXPERIMENTS.md rewritten, DRUM_PLANNER.md experiment paragraph updated. requirements comment changed only; no dependencies installed. Don't modify memory.

Cwd /home/jason/Proyectos3/X5; source /opt/ros/jazzy/setup.bash + install/setup.bash for tests. Venv .venv-planner-experiment/bin/python, CPU Ryzen9 7940HS16threads,12GBRAM, integratedRadeon780M, no nvidia-smi. DISPLAY=:0. No git repository. No applicable AGENTS.md.

Useful commands:
source /opt/ros/jazzy/setup.bash
OPENBLAS_NUM_THREADS=1 .venv-planner-experiment/bin/python diagnostics/gesture_study_20261020/summarize.py
# Wrapping long scripts:
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.app import wait
wait(Job([PYTHON,'diagnostics/gesture_study_20261020/verify_selected.py',root,out],jobfolder,timeout=1800))
# GUI:
PYTHONPATH=. OPENBLAS_NUM_THREADS=1 .venv-planner-experiment/bin/python tests/check_gesture_ui.py SESSION OUT

No need to ask user anything. Continue through final delivery, not an interim stop.

## Update 00:29 local
All18 heldouts passed. Independent unchanged-production checks COMPLETE: refine41 distinct trajectories/597569 samples; heldout43/568227, all pass. Final regression517 passed110.62s. Protection audit40 files unchanged; core algorithm hashes unchanged afterholdout; defaultscene/state/calibration+allreferencedscenehashes verified. UI tiny status-wrap/combobox-size adjustment AFTERfreeze only (notsearch).
Actual CLI all4 methods passed. Initial native UI run played all4 complete cycles and passed all controls, but TEST HARNESS wrongly asserted camera_released on baseline snapshot fixture (field absent; no camera actuallyran). Fixed test to compare exact fixturehash instead. Firstattempt evidence ui_test_attempt1 andui_job preserved. New full rerun active session22890, logui_retry.log, CLIoutputui_session_v2, reportsui_test, servicejobui_retry_job. Wait until ui_full.json passed. No production algorithm failure. Final_validation.py session45672 exited1 because of oldtestassertion; use retryresults instead.
Pending: finishUIretry, run plot_results.py (containedJob) then inspect bothfigures, aggregateverification.json, runwrite_report.py (requiresverification.json), update docs evidence, cleanupcheck no activejobs. Reportwriter referencescanonical ui_test. It currently incorrectly saysinteriors never "objective weights" (remove that phrase: principles informed hypotheses, but no interior data supplied toplanner). Finalnotify+memorycitation aftercompletiononly.

## Numerical tie correction / final final validation
Full UI retry PASSED (all4 complete cycles andallcontrols) but observed two nearidenticalgeometry selections driven by2e-9 floatnoise. Applied production-style round(score,8) thenearliercandidateindex; stilltime-independent. Addedunitregression. Rerank_numerical_ties.py verified ALL33 initialgenerator seedchoices unchanged; onlysweep main19 index24→8 andsweep goal4/19 index24→5 changed. Oldresult/evaluationfiles preserved. precision_fix.json documents. Noobjectiveweights/proposals changed. Needupdatepreservationaudit toexplicitlyallow/document search.py numericalfixafterfreeze (do not falselyclaimallcorehashesunchanged). 
Current validation session55085 precision_validation.log: reruns postgeneration evaluations + independentchecks (cachedoldproofwhereappropriate), actualCLI ui_session_final, nativeUI ui_test, plots. Mustwaitforcompletion. Fullregression session21799 regression_precision_final.log expected518tests. Reportwriterupdateddisclosure. Needaggregateverification, runwrite_report, inspectplots/UI, auditnoorphanjobs, finalresponse.

## Final audit — complete

The pending items above are historical, not active work.

- `precision_validation.log`: all stages passed. Actual default-input CLI and
  native GUI completed successfully; all four methods played full cycles and
  all listed controls/recovery paths passed.
- `regression_precision_final.log`: 518 tests passed.
- Final selected curves: 40 development + 42 reserved, 1,077,765 dense states,
  unchanged production checker. The final audit verified every cached proof's
  SHA-256 against its current candidate file.
- `finish_audit.py` passed. Protected 40 files and all saved scene/state/calibration
  hashes match. Post-freeze changes are explicitly disclosed in
  `preservation_audit.json`, not falsely described as unchanged.
- Planning workers peaked at 373.7 MiB; initial compilation/setup peak was
  1,199.3 MiB. No logged OOM; no running experiment services remain.
- Final native UI screenshot and both plots visually inspected. No overlaps,
  clipped controls or render errors. All 20,184 scene triangles retained.
- `summary.json` refreshed after numerical tie correction. `REPORT.md` generated
  and reviewed, including slow-timing trade-offs, different recorded start,
  fallback provenance and limited generalization.
- No physical arm/CAN or live camera access. Recapture was tested using an
  explicit offline fixture only. Standalone hardware baseline unchanged.
