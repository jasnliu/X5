# Workspace curve hardware mode — 2026-10-22

## Delivered

- `./plan_drums.sh --hardware` (right by default), `--right`, or `--left`.
  Old `singlearc` / `baseline` commands are compatibility aliases.
- Hardware planning calls the same Workspace curve generator, settings,
  120-second per-leg search budget, selection policy, and arm-aware timing
  dispatcher as the experiment. No recording interiors or Single arc fallback.
- Arm-specific saved-destination dropdowns, Add checkpoint / Remove last /
  Refresh; maximum three total destinations. Selection only invalidates old
  work. **Calculate** alone starts acquisition/search. Opening the UI does not
  calculate; **Start** alone authorizes the selected motor owner.
- The existing three-window UI, independent preview/live poses, speed scaling,
  fixed two-second holds, Hold and reverse-to-rest recovery are retained.
- Normal multi-checkpoint playback follows independently checked legs from the
  measured start through the destinations, then a new final-to-original-start
  return. It does not retrace earlier checkpoints. Emergency recovery instead
  retraces the already-traveled mission prefix. Relaxation follows confirmed
  settling at the original supported start, not an ordinary midair fault.
- Exact analytic per-leg laws are used by preview, controller, dense final
  checking and prefix recovery; no new sampled-position refit/interpolation.
- Expensive multi-leg polynomial validation was moved before owner IPC opens,
  before any transport is constructed. Start reuses that validated law with
  input-hash binding rather than blocking the heartbeat loop. No deadline was
  loosened and no collision, zone, model, or motion limit was reduced.

## Verification (software only)

**No physical arm commands, camera access, or encoder queries were made.**
Fresh hardware acquisition/physical tracking remain untested in this task.

- **484 passed**: hardware state machine, legacy/selected-arm behavior, device
  isolation, Workspace generator, checkpoints, timing, performance parity,
  source preservation, packet integrity and new controls/preload regressions.
  One existing SLSQP internal optimizer-bound warning in the preserved timing
  comparison; no test failure. This is not clipping measured robot joints.
- **99 passed**: arm/model/visual, snapshot mesh, current-state planning and
  executor regressions.
- **10 passed**: repository artifact/ignore-policy checks. Total: **593 tests**.
- Actual `./plan_drums.sh --hardware --simulate ...` launchers passed for the
  default right arm and explicit `--left`. Both opened/closed without planning,
  acquiring a motor owner, or reporting an error.
- Native Qt/VTK integration passed on **both arms**, exercising one- and
  three-destination Calculate, exact-curve preview, play/pause, scrub, speed
  0.25/0.60/1.00, independent simulated owner Start, normal return/relax,
  Emergency Hold, traveled-prefix Reverse/relax, selection invalidation,
  add/remove/exclusion and Cancel. All five action buttons remained available.
  Six completed fake-owner cycles total. No render errors/final UI errors;
  frozen geometry uploaded only once per window. Screenshots reviewed.
- Real Workspace searches generated routes using saved camera/start inputs.
  Right native final verification reused plans generated earlier in this same
  task after the heartbeat fix. Left's folder has one destination, so two tiny
  J1 variants were created **only inside its ignored test run**, never added to
  `destinations/left` or presented as captured measurements.
- Independent native MoveIt/FCL exact-curve checks at 2 ms plus dense edges:

  | Arm | Destinations | Samples | Result |
  | --- | ---: | ---: | --- |
  | Right | 1 | 2,104 | all valid |
  | Right | 3, including new return | 9,378 | all valid |
  | Left | 1 | 1,421 | all valid |
  | Left | 3, including new return | 6,298 | all valid |

- Experiment/hardware wrapper parity: **10/10 legs** used identical inputs and
  checked local pools, selected the same candidate, and produced bit-identical
  path waypoints, source timing, retimed law, outward samples and duration.
  This does not equate a future live camera/start with an experiment's saved input.
- Source hashes confirm unchanged experiment generators/checkpoint composer,
  native collision code, timing algorithms, models, zones and user recordings/
  destinations. The historical audit tests now allow this explicitly requested
  hardware adaptation; the new audit guards the entire current shared stack.

During testing an early screenshot-helper call incorrectly used Qt `mapTo`
with an unrelated top-level window and crashed that test process. The helper now
captures the control window directly with QWidget.grab; subsequent native runs
passed. No production renderer change or quality reduction was needed. The
multi-leg owner heartbeat stall was a separate production issue fixed above.

## Evidence and repeatable entry points

- `verification.json`: sanitized UI/native/parity evidence (no owner tokens).
- `regression.log`, `supplemental.log`: fresh test results.
- `before_hashes.json`, `reference/`: pre-task source/data audit and hardware
  implementation references. Bulk plans/caches remain ignored.
- `tests/test_hardware_workspace.py`: unit/state/packet/selection regressions.
- `tests/hardware_workspace_ui_check.py --side right|left --output NEW_DIRECTORY`:
  saved-input native UI, real planning, fake motors only.
- `tests/hardware_workspace_parity_check.py --ui UI_RUN_DIRECTORY ... --output NEW_DIRECTORY`:
  exact wrapper parity using those test-generated checked local pools.

Source `/opt/ros/jazzy/setup.bash` and `install/setup.bash`; use the planner venv,
`PYTHONPATH="$PWD:$PYTHONPATH"`, `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`.
