# Joint operating-envelope testing

## Scope and current status

`./test_joint_limits.sh` is a **separate, entirely offline software tool** for
both arms. It prepares bounded tests, generates synthetic test data, scores
recorded telemetry, exports experiment-compatible candidate profiles, and
re-times unchanged saved Workspace paths. Existing experiment, `--fast`, camera,
calibration, motor-control and firmware programs are unchanged.

**This version has no hardware execution or acquisition command.** It cannot
move, enable, disable, hold, query encoders, open the camera or measure an arm
that is switched off. A future live collector/runner must supply physical
telemetry; that integration is a separate hardware-stage decision. The pure
feedback decoder and trace format are ready for that integration, not evidence
that it has been physically validated.

Running the demo proves the software workflow, **not robot performance**.
Actual testing establishes a successfully tested operating envelope for a
specified payload, posture, direction, controller, supply and duty cycle. It
cannot prove an absolute hardware maximum or a globally fastest route.

## Planned physical process — later, not executed now

### 1. Establish the test envelope

- Confirm installed motor/firmware versions, payload (including gripper/stick),
  supply, current/torque protections and controller settings. The local SDK maps
  J1–J2 to RS04, J3–J4 to RS03 and J5–J7 to RS00. This mapping must match the actual
  assembly; it is not established by a successful simulation.
- Consult the [manufacturer information](https://github.com/RobStride/Product_Information/)
  for the exact revisions. Protocol encoding ranges, no-load speeds and peak
  torque ratings are **not** safe loaded-arm operating limits. Ratings include
  conditions such as cooling; do not copy their maxima into a test profile.
- Keep existing joint-position bounds, current protections, tools, calibration,
  collision margins and observed-obstacle policy. No firmware zeroing, gains,
  torque/current increases, gripper-opening sweep or search for mechanical stops.
- Start at the existing software limits, with smaller test levels. Select
  representative poses, both excursion directions and bounded clear intervals.
  Do not assume R1/R2 is a supported position where an arm may be relaxed.

### 2. Low-demand validation, then supervised joint probes

- The future runner must acquire the actual startup poses and a fresh scene,
  plan/check the approach from supported rest, the probe and the return. Saved
  test preparation is not permission to execute a historical trajectory.
- Establish tracking and telemetry quality at low demand first. Use the same
  motor-control mode that will execute the final drum motion.
- Test one joint at a time while the other selected-arm joints hold their
  targets; the opposite arm remains query-only. Include both directions and
  repeated trials at multiple representative poses.
- Increase only within explicitly chosen ceilings. One Start authorizes one
  visible trial; no unattended escalating search or queued surprise movement.
  A failed trial stops escalation. Communication, current limiting, overheating,
  tracking lag and geometric obstruction must be reported separately, not all
  called a mechanical limit.
- Record commanded positions/time, actual position/velocity, current and its
  age, drive-reported torque, temperature, faults, drive state, sample times and
  feedback sequence/age. The existing normal CSP transport sets a **0.8 rad/s**
  playback cap (`centering/motors.py`); the future dedicated test runner must
  verify its applicable readback so controller saturation is not mistaken for
  a motor maximum. This task does not change that cap.
- Preserve ownership/HOLD through ordinary cancellation/fault/terminal loss.
  Offer an explicitly requested checked reverse of the traveled prefix to the
  measured supported rest before relax. Never automatically drop a suspended
  arm. Unusable feedback or a drive fault can make recovery unavailable; do not
  pretend HOLD or reverse is guaranteed in that state.

### 3. Derive a conservative candidate profile

- Require all requested repeats at each lower level before using a higher level.
  Require coverage of all seven joints, both excursion directions and at least
  two selected poses. Report missing evidence instead of filling it with defaults.
- Use **analytically known commanded V/A/J peaks from successfully tracked
  motions**, not maxima from noisy finite differences. Velocity-derived A/J
  estimates are reported at two smoothing windows as diagnostics only.
- Small strokes often cannot reach the requested velocity. The report exposes
  achieved/requested excitation separately. A demand never reached is **untested**,
  not a measured velocity ceiling. Longer clear strokes or later route trials
  may be needed; the software never automatically expands motion range.
- Choose the largest successfully repeated demands within each pose/direction,
  then the minimum demonstrated envelope across those conditions. Apply an
  explicit time-scaling reserve: default speed fraction 0.8 gives V×0.8, A×0.64,
  J×0.512. This is a configurable engineering margin, not a safety certification.
- Peaks obtained in different probes are not necessarily jointly achievable.
  The exported 21-value box is therefore a **planning candidate**, not a certified
  simultaneous hardware envelope. The evidence sidecar records that limitation.

### 4. Re-time and physically validate complete drum routes

- Re-time exactly the existing R1→R2 and R2→R1 curves with the candidate profile;
  do not change path generation while comparing limits. Keep scene, endpoints,
  tools, passive arm and geometry constant for the offline comparison.
- Later verify the full multi-joint motions physically, including repeatability,
  tracking, settling, stopping, accumulated temperature and realistic drum duty
  cycles. A single successful joint probe does not validate simultaneous motion.
- Report the fastest **successfully tested route** within the tested envelope.
  A planning prediction is not physical timing, and reaching an approved ceiling
  without failure means the true maximum remains unknown. No impact/striking
  performance or absolute global optimum is inferred by this tool.

## Offline use now — safe with the arm switched off

Run from the X5 checkout. Choose new output paths; old evidence is never overwritten.

```bash
# End-to-end software demos. Both are synthetic, not physical measurements.
./test_joint_limits.sh demo --right --output diagnostics/limits_demo_right
./test_joint_limits.sh demo --left  --output diagnostics/limits_demo_left

# Exercise failure detection and stopping escalation:
./test_joint_limits.sh demo --right --failure tracking \
  --output diagnostics/limits_demo_failure

# Create an editable study configuration. Original V/A/J limits are retained.
./test_joint_limits.sh init --right --output diagnostics/right_limit_study.json

# Use a successful single-leg saved experiment plan and its adjacent request.
# No camera or encoder access occurs. Its scene must be appropriate for the study.
./test_joint_limits.sh prepare --config diagnostics/right_limit_study.json \
  --plan /path/to/saved/plan.json \
  --poses destinations/right/destinationR1.json destinations/right/destinationR2.json \
  --output diagnostics/right_limit_probes

# Optional --joints 5 6 7 prepares a smaller study; no all-joint profile is then
# exported because J1–J4 remain unmeasured.
./test_joint_limits.sh simulate \
  --campaign diagnostics/right_limit_probes/campaign.json \
  --output diagnostics/right_limit_fake_traces

./test_joint_limits.sh analyze \
  --campaign diagnostics/right_limit_probes/campaign.json \
  --traces diagnostics/right_limit_fake_traces \
  --output diagnostics/right_limit_analysis
```

Simulation profiles require the explicit `--allow-synthetic-profile` option to
`analyze`; `demo` supplies that option itself. Their filenames begin
`SYNTHETIC_NOT_FOR_HARDWARE_`. They are never installed as defaults. Missing
payload/controller/supply labels, missing cohorts, malformed/duplicate records,
mixed synthetic/physical provenance or insufficient poses prevent export.

For later **recorded physical data**, use the same `analyze` command on the
physical trace directory. Its candidate JSON uses the existing experiment
profile schema, with a separate `.evidence.json` sidecar. Import that JSON using
the experiment's **Joint limits / profiles… → Load profile…**. It remains a
simulation timing profile, not a firmware configuration.

To forecast an existing path with any explicit profile:

```bash
./test_joint_limits.sh routes --plan /path/to/saved/plan.json \
  --profile /path/to/candidate_right.json --quality balanced \
  --output diagnostics/right_candidate_route
```

This performs fresh timing and unchanged native collision/zone checking, plus
an independent continuous-root limit check. It writes `timing.json` and a
forecast report, **not an executable hardware plan**. Path generation and live
acquisition are excluded from its reported calculation time.

## Outputs and trace contract

- `campaign.json`: settings, checked/blocked probes, exact C3 polynomial laws,
  original acquisition mask/request, geometric/model/data hashes and trial IDs.
  The opposite arm and original self-mask are preserved during preparation.
- `analysis/report.json`, `REPORT.md`, `joint_envelope.csv`: accepted/rejected
  trials, reasons, repeated-level coverage, missing conditions, commanded peaks,
  measured diagnostics, and candidate profile path or a clear no-export result.
- `<profile>.evidence.json`: profile identity, tested conditions, trial references,
  reserve and mandatory combined-route-validation caveat.

Each `.npz` trace must have `metadata` (JSON string) and numeric arrays:

| Field | Shape / meaning |
|---|---|
| `time_s` | N, strictly increasing elapsed time in the probe clock |
| `command_time_s` | N, timestamp of actual last command; nondecreasing and ≤ sample time |
| `command_q_rad` | N×7, actual targets; must match the identified law at command time |
| `q_rad`, `velocity_rad_s` | N×7, measured joint coordinates/velocity, not motor raw signs |
| `current_a`, `current_age_s` | N×7, measured filtered drive current and its age; no missing-as-zero substitution |
| `torque_nm`, `temperature_c` | N×7, drive-reported torque and temperature |
| `fault_bits`, `mode` | N×7, actual drive fault bits and running state |
| `feedback_age_s`, `feedback_sequence` | N×7, freshness and monotonically advancing measurement counters |

Metadata includes `origin` (`synthetic` or `physical_recording`), `side`,
`campaign_sha256`, `trial_sha256`, `trial_id`, and the exact campaign `conditions`.
Checksums detect mismatches; they do **not authenticate a claim that someone’s
imported file came from real hardware**. Imported physical data remains reported
physical evidence, not independent physical certification.

`joint_limit_lab.telemetry.decode()` is a pure recorded-frame decoder, reusing
X5 joint signs and the right-J5 offset. Protocol scales come from installed
`/usr/include/openarmx/robstride_motor/rs_motor_constants.hpp`; they are not test
limits. The [manufacturer protocol reference](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py)
identifies status feedback and separate filtered-current/voltage parameters.
No CAN/serial/camera transport exists in this package.

## Resource and scope controls

All nontrivial CLI commands run one contained worker, with a 3 GiB memory cap,
bounded duration and AF_CAN excluded. The Python entrypoints also reject CAN
sockets. Heavy work is serial. Trials, trace size and count are bounded. No
results/reference caching, hardware commands, package upgrades or existing
program modifications are needed.

Probes are limited to 60 seconds; synthetic output has a 256 MiB budget and
route validation a 100,000-sample budget. These are resource bounds, not physical
limits. Cancellation preserves partial data but never writes a successful
completion receipt. A zero subprocess exit alone is not treated as success.

Verification: `tests/test_joint_limit_lab.py` and
`diagnostics/joint_limit_testing_20261025/REPORT.md`. Fake traces and saved-scene
checks are not physical validation. The live runner, fresh acquisition,
controller-cap readback, supervised recovery integration and physical tests
remain the explicitly separate hardware stage.
