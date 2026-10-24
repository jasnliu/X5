# Selected-arm Single arc hardware workspace — 2026-10-21

## Delivered

- Preferred commands: `./plan_drums.sh singlearc --right` and `--left`.
  Right is the default; `baseline` remains a compatibility alias, including
  recovery of historical right-arm sessions without side metadata.
- Same three-window workflow: manual Calculate, disconnected preview with
  play/pause/scrub/speed, connected measured-pose view, and five always-clickable
  controls. Window titles and start/guard messages identify the selected arm.
- Right destination: `recordings/record3.json` final joint pose. Left destination:
  `left_recordings/record1.json` final joint pose. No interior recording samples
  enter path generation. Both use the shared experiment Single arc selector and
  latest side-specific timing dispatch, unchanged.
- Left uses its accepted traditional-grip model and zone1; right retains zone3.
  Original gripper-point zone semantics, full collision bodies, and current
  occupied-volume margin policy are unchanged.
- Production Calculate still acquires the current arm pose and camera scene.
  Saved inputs are only used by the explicitly software-only verification below.
- Selected side propagates through the monitoring worker, isolated computation,
  planning, preparation, independent owner, CSP wire gate, recovery snapshot,
  measured renderer and session resume. Wrong-side plans/sessions cannot start.
- Right motor-8 target is unchanged (+7 degrees internal). Left uses the existing
  playback target (-14.16 degrees internal, +14.16 degrees raw). Opposite-arm
  traffic is state queries only, even on enable, hold, reversal and final release.
- Faults do not automatically relax. Normal/recovery release occurs only after
  returning to the verified original starting pose. Passive arm sway is not a
  selected-arm movement veto; current passive geometry is used for return checks.

## Verification boundary

**No physical CAN socket, camera, motor enable, target, hold, gripper, or relaxation
command was issued.** Tests use recorded depth, virtual encoder replies and
simulated motors. This verifies software behavior, not physical tracking or
clearance in the current kit. All synthetic acquisition eligibility was revoked
before the verification scripts exited. UI tests require `--simulate` owners;
Python audit hooks and systemd address-family restrictions deny AF_CAN.

## Full production calculations with virtual sensors

Only sensor access was substituted. Real query decoding, timestamp matching,
selected-arm motion monitoring, planning, timing, MoveIt/FCL dense validation,
packet validation and checked-prefix recovery executed.

| Selected arm | Outward duration | Calculation + checks | Encoder batches | Final curve samples + edges | Return samples + edges | Calculation memory peak |
|---|---:|---:|---:|---:|---:|---:|
| Right | 4.020221 s | 115.024 s | 2,194 | 2,012 | 3,951 | 462.38 MiB |
| Left | 2.834205 s | 105.164 s | 2,005 | 1,419 | 3,090 | 426.82 MiB |

Both produced valid trajectories and recovery proofs with the unchanged 20 mm
requested obstacle buffer plus 5 mm edge allowance. No missing-feedback error.
Jobs ran serially under the existing 3 GiB calculation limit, without AF_CAN.
See `right_calculation.json`, `left_calculation.json`, `calculation_jobs.json`.

## Native UI and independent simulated owners

Each side passed 23 workflow checks, including exactly three visible windows,
nonoverlapping controls, no planning before Calculate, preview play/pause/scrub/
speed, live-only movement, selected-arm invalidation/recalculation, passive sway,
unchanged active speed, Hold, checked Reverse/rest/release, full goal-hold-return,
injected CAN fault recovery, no surprise motion after a failed return, and saved
owner reconnect without a second enable. Three owners per side ended RELAXED;
all test-owned sockets were removed. Both literal `singlearc --SIDE --simulate
PLAN --seconds 3` CLI runs also exited cleanly. Screenshots were inspected.

See `right_ui.json`, `left_ui.json`, `right_cli.json`, `left_cli.json`, and the
small control/preview screenshots. Runtime plans, raw captures and IPC credentials
remain local and ignored by Git.

## Reproduction

Source `/opt/ros/jazzy/setup.bash` and `install/setup.bash`, then use:

- `python3 -m diagnostics.singlearc_arms_20261021.run_calculations`
- `bash diagnostics/singlearc_arms_20261021/run_ui.sh`
- `bash diagnostics/singlearc_arms_20261021/run_tests.sh`

The calculation/UI harnesses require fresh output directories; preserve/move an
old diagnostic run first rather than overwriting it. They never launch physical
playback. `before.json` preserves pre-change hashes; the regression test verifies
unchanged experimental algorithms, timing engines, models, recordings, encoder
conversion, shared motor transport, and `start_beat.sh`.

## Regression result

**951 tests passed in 313.70 seconds.** This includes the new 31 selected-arm
checks, existing hardware-state-machine/recovery tests, exact timing and corner
smoothing, experiment/path selection, collision/zone/voxel policy, renderers,
calibration, recording and read-only acquisition tests. `full_tests.log` records
the completed run. `git diff --check`, Python syntax parsing and shell syntax
checks also passed. No physical behavior is inferred from these software tests.

Post-documentation/preservation audit: **41 tests passed in 3.57 seconds**
(selected-arm tests plus curated-artifact/clean-checkout tests).
