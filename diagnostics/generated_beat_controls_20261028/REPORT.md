# Generated beat — direct stop/relax and upfront return

## Requested behavior

Normal `./start_beat.sh --hardware` now has separate **Emergency Relax**, **Emergency
Hold**, and **Stop + Close** actions. Explicit Relax/Stop bypasses pose, joint-bound,
collision, stationary-feedback and planning-readiness conditions. Closing the
window, Ctrl-C and Escape no longer demand a return to rest first (Escape relaxes
without closing). Ordinary faults can still Hold; the operator's explicit
torque-off action is always available.

The normal return is prepared **before Start**, stored with the fresh mission,
and included in the 3-D preview. Outbound stays **left → right**; return is
**right → left**, as requested for the return sequence.

## Implementation and scope

- `generated_transport.emergency_release()` reuses the established bounded
  emergency writer shutdown, blocks later GUI motion writes, then attempts
  disable packets for every arm/gripper drive on both buses. A failed writer,
  stale feedback or one failed bus does not skip attempts on the other drives.
  It does not enable anything, open grippers, command a pose or clip feedback.
- The app stops ESP32 schedules and releases its outputs independently. Cleanup
  errors cannot stop the arm-disable attempts or prevent closing the panel.
  Disable delivery errors remain visible; software cannot guarantee torque-off
  over a physically failed/disconnected CAN link. Explicit torque-off can let an
  unsupported arm fall; powered Hold remains a separate button.
- A successful explicit Relax allows capture/recalculation in the same window.
  Old closed strike sessions are cleared when a new mission is installed.
- Schema-v2 generated missions include a checksummed `return/return.json`.
  Both complete reverse paths are checked against the original scene and the
  correct opposite-arm poses during preparation. No native checking worker or
  path/timing optimization is launched when normal Return is clicked.
- Normal Return first asks the existing strike owners to finish their current
  strokes at prime, cancels further hits, then uses the prepared return. It does
  not emergency-freeze a normal stroke at drum contact and attempt a new path
  from that contact pose. A failed finish request switches to Hold instead of
  leaving an unsupervised swing running.
- Partial path returns crop the already-checked geometry and use the existing
  bounded time law. Off-path/invalid feedback may still block *movement*, but
  cannot block **Emergency Relax or Stop/Close**. No contact exemption, unchecked
  connector or altered joint/collision limit was introduced.
- The disconnected preview shows outward left/right, a two-second preview-only
  prime pause, then the cached right/left return ending at captured rest. The
  actual beat does not auto-stop after that visual pause.

Only generated-mode sources, its tests and documentation changed in this task.
The legacy recording modes, underlying strike algorithms, shared planner/model,
firmware, launchers, recordings and destination JSONs were preserved. See
[preservation.json](preservation.json) and [before.json](before.json).

## Verification — software only

**No live camera, CAN, microphone, ESP32 or physical arm tests were run.**

Fresh calculation used the previously saved real paired capture:

`planner_runs/20261028T023247Z-hardware-right-ae273b/calculate-7f013e4d/capture`

Results: [mission](verified/mission.json), [return](verified/return/return.json),
[preparation log](prepare.log), [bounded-worker record](prepare-job/job.json).

| Path | Movement | Dense samples plus connecting edges | Result |
|---|---:|---:|---|
| Left outbound | 3.764860 s | 1,884 | Valid |
| Right outbound | 2.743720 s | 1,373 | Valid |
| Right return | 12.067369 s | 6,036 | Valid, cached upfront |
| Left return | 15.917418 s | 7,961 | Valid, cached upfront |

The original conservative return time law was not changed. Full preparation,
including fresh DT1, both outbound routes and the cached return checks, took
33.766081 s on these saved inputs. This excludes live acquisition and compilation.
Loading the complete return at button time took **0.000287 s** in the measured
device-free selection check. Partial-prefix selection took 0.037–0.052 s without
a native worker: [timing evidence](return_selection_timing.json).

Tests:

- [regressions_final.log](regressions_final.log): **319 passed in 62.61 s**, final selected regression run,
  including existing recording/transport, hybrid, snare, hi-hat, and generated
  workflows. No historical test expectations were changed to hide source-hash
  failures; these are functional suites relevant to this change.
- [ui_final_verified.log](ui_final_verified.log), [ui_report.json](ui_report.json):
  actual Tk controls with fake motors. Start, genuine partial-left Hold/return,
  complete left/right approach, existing ride/audio handoff, normal stroke-finish
  Return, right/left playback and release at rest. `_spawn` is deliberately made
  to raise during Return, proving no end-of-run planning worker is used. Explicit
  Relax and active-window close succeed with deliberately stale feedback and an
  invalid simulated joint pose; capture/retry remains possible after Relax.
- [preview_report.json](preview_report.json): real Qt/VTK windows, frozen real
  mesh, both custom tools, outward and both cached return phases, final rest,
  scrub/play/pause and independent measured-state view. No rendering errors.
  Images: [controls](control_panel.png), [right return](return_right.png),
  [left return](return_left.png), [prime goals](generated_goals.png).
- `tests/test_generated_stop_return.py` specifically verifies unconditional
  16-drive disable attempts, failure isolation/retry, no geometry/freshness gate,
  no late Hold after explicit torque-off, unconditional close, cached full return,
  partial-prefix cropping, normal strike-finish handoff, and return preview.
- `git diff --check` and generated Python syntax checks pass. Development-failure
  logs are retained; use the final logs above rather than earlier reruns.
- [launcher_simulation.log](launcher_simulation.log): real shell launcher in
  device-free mode; Ctrl-C requests explicit Relax and the application exits
  cleanly, rather than requiring a resting-pose return.

## Operator command

```bash
./start_beat.sh --hardware
```

This command opens real devices and starts the existing hi-hat calibration;
it was **not** launched during this task. Device-free replay:

```bash
./start_beat.sh --generated-simulate diagnostics/generated_beat_controls_20261028/verified/mission.json
```

See [GENERATED_BEAT.md](../../GENERATED_BEAT.md) for current controls and behavior.
