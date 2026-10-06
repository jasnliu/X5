# Standalone J7 strike experiment

**Record3-based cymbal setup with adjustable strike depth.**
The methods were previously calibrated on the **unloaded, centered** arm; those
results are preserved in [EXPERIMENT_RESULTS.md](EXPERIMENT_RESULTS.md). They are
not a calibration of the new recording-end posture or permanently attached stick.
This workflow change has offline verification only, not new physical testing.
`experiment.sh` is independent of `start_beat.sh`. Existing beat modes and source
are unchanged. The new program provides Tk controls and the same robot model in
RViz. There is no camera, microphone, detector, stick-loading step, or ESP32
connection. Preparation now plays `recordings/record3.json`. Default mode is a synthetic inertial simulation, not
physical control. ROS domain 93 keeps the visualization separate from beat's 92.

## Start the software

```bash
./experiment.sh                         # simulation, controls + RViz
./experiment.sh --no-rviz --degrees 3   # simulation, controls only, 3-degree strikes
./experiment.sh --list-methods
./experiment.sh --headless --fast --method all --repetitions 3
```

The normal window contains Close + Center + Record3, method selection, strike
depth in degrees, JSON parameter editing, repetition count, Run Selected,
Compare All Defaults, campaign stage,
Pause After Current, Finish Batch + Relax, and Stop + Relax. Escape stops the
session. Full-rate data are not inferred from the decimated live depth plot.
A normal manual batch holds at the anchor for the next request; Finish Batch +
Relax closes that session. `--auto-run --exit-after-batch` automates preparation,
the selected batch, relaxation and closing the UI/RViz.

### Preparation and manual hits

1. Press **1. CLOSE + CENTER + RECORD3**. The file is validated first, before
   hardware connection or motion. The gripper closes, the right arm moves to its
   usual custom center, then moves to record3's start and plays the recording.
2. **The gripper stays closed** during centering, playback, endpoint hold, and all
   strikes. Any finger/opening values stored in record3 are ignored for commands.
   There is no loading, camera alignment, sound detection, or automatic strike.
3. J1–J6 hold the recording endpoint. J7 enters MIT hold at the exact recorded J7
   endpoint, now the fixed zero-depth reference for the session. It does not return
   to center after playback and is not recaptured between strikes.
4. Enter **Strike depth (°)**, choose a method and press **RUN SELECTED METHOD**.
   A depth of 3 means 3° down from the held endpoint, then back to that endpoint.
   Change depth between batches without replaying record3. The entry is locked
   during preparation and a running batch; graph markers update for each goal.
5. The live graph, method controls, results table and saved full-rate traces work
   as before. **STOP / FINISH / window close still relax the arm**, including the
   gripper; “stays closed” describes the enabled preparation/playing/hold session.

The record3 source is not edited. Playback uses the existing validated, smoothed
recording trajectory and velocity-scaled timing (0.8 rad/s playback firmware speed,
then back to normal 0.4 rad/s); only J1–J7 recording targets are sent. The simulated
preparation follows the same recording, but strike dynamics remain synthetic.

### Physical use

```bash
./experiment.sh --hardware --degrees 3
# Default depth remains 10 if --degrees is omitted; the UI can change it.
# Only use a campaign once the setup and selected depth have been checked:
./experiment.sh --hardware --headless --degrees 3 --method all --campaign --config my_calibration.json
```

Without an explicit `--config`, hardware loads
`config/experiment_hardware_tuned.json` and defaults to `gravity`. Simulation
keeps its separate synthetic defaults and defaults to `powered`. An explicit
configuration replaces the preset rather than silently merging with it. The
configuration path and SHA-256 are recorded in new sessions. A tuned profile is
specific to the unloaded centered arm, not a drop-in calibration for another
posture, stick, or cymbal.

Hardware requires the explicit flag. `STRIKE_LAB_OFFLINE_ONLY=1` forbids creating
the hardware backend even if the flag is supplied. Physical initialization sends
an ntfy warning using the existing host notifier and waits ten seconds BEFORE
connecting/enabling/moving the robot. Failed notification prevents setup. There
is no required human acknowledgement. `--fast` is accepted only for headless
simulation. Never run simultaneously with another arm-control program.
An explicitly authorized operator can use `--no-motion-notification` to waive the
notification/countdown; this choice is saved in the session. It does not change
the motion boundaries. The October 2 hardware experiment used this waiver at the
user's request.

## Exact common target and score

Every method shares one **user-selected goal**, default **10.000 degrees**, measured
in the existing downward J7 direction (decreasing J7 angle) from record3's exact
endpoint. This common goal is not a per-method tuning parameter and the search
cannot change it. The reference stays fixed across strokes.

- Set depth in the UI or with `--degrees VALUE`; input is validated before use.
- Depth acceptance is ±`min(rules.acceptance_deg, 0.05 × goal)`; the default is
  **10 ± 0.2°**, or **3 ± 0.15°** for a 3-degree goal.
- The plotted/scored lower zone is **strictly deeper than 90% of the goal**:
  >9° for 10°, >2.7° for 3°, >0.9° for 1°.
- Depth must be at least 0.5°, and depth plus tolerance must remain below the
  unchanged hard downward limit (normally 12°), as well as inside J7's joint
  range. Large entries do not automatically enlarge the motion corridor.
- Changing depth scales trajectory distance and catch targets, not the saved
  gains, durations or load feedforward. A curve exceeding shared dynamic limits
  is rejected, not sped up or silently clipped to a different goal.
- Lower-zone time: sum of ALL intervals in that zone, downward and upward,
  including any reentries. Linear interpolation uses actual encoder timestamps.
- A crossing-time uncertainty bracket is recorded. Interpolation is not evidence
  of timing precision finer than the feedback sample spacing.
- Stale/missing or out-of-order feedback, partial crossings, a bad return,
  substantial reversal chatter, creeping/staging, very slow approaches, velocity
  spikes, excess command limiting, and depth misses remain failed attempts.
- Minimum entry speed is 0.20 × goal/10 rad/s by default, estimated from encoder
  positions around the crossing, not from the commanded speed. This and other rules require
  physical calibration; changes must be shared by every method.
- Return requires a fresh 40 ms position window near the same anchor.
- Depth-matched comparisons require batch median depths within 0.10 degrees.
  A shallow tolerance-edge stroke cannot automatically become the winner.
- All trial failures count. Results are never calculated from only a method's
  successful subset. Groups are separated by method, exact parameters, backend,
  rules, motion limits, campaign stage, **selected goal and zone**.
- Rankings show median and 95th-percentile zone time, depth spread, cycle time,
  overshoot, entry speed, effort and timing uncertainty. Ties within measurement
  uncertainty remain ties. A hardware winner additionally needs 100 successful
  validation trials and a successful endurance group of at least 100 trials.
  The comparative campaign must finish its entire requested plan; stopping after
  the first successful method cannot produce a winner.
  Simulation reports only provisional leaders, never a physical winner.

The zone remains an **encoder-based timing proxy**, not a contact sensor or tone
measurement. You set the goal to where the real cymbal is. The program cannot
detect its surface or know whether the stick is touching it. Posture, stick mass
and real contact change the dynamics; old no-stick scores do not transfer.

## Saved methods (eight families / nine modes)

| Mode | Family / behavior | Main tuning variables |
|---|---|---|
| `baseline` | Previous MIT-controller family with lab-only exact-target catch and fixed-load correction | damping, braking estimate, return acceleration, delay |
| `gravity` | Low-damping gravity descent + predictive catch/withdrawal | damping, braking, return duration, delay |
| `variable_damping` | Smoothly scheduled gravity-descent damping | damping levels, transition depth, catch timing |
| `powered` | Asymmetric quintic down/up trajectory | down/up durations, nonzero bottom curvature |
| `cosine` | Powered sin^4 pulse with smoothed endpoints | duration, gains, feedforward |
| `torque` | Rounded acceleration-derived torque pulses with weak stroke tracking | pulse timing/curvature, inertia, low gains |
| `hybrid` | Powered initial pulse → coast → predictive upward catch | pulse amplitude/duration, damping, catch |
| `impedance` | Moving virtual spring with smooth gain scheduling | approach/return stiffness/damping, timing |
| `optimized` | Adjustable approach-knot quintic spline | knot time, knot velocity shape, curvature, return time |

Each method is its own module under `strike_lab/methods/`, selected as a mode
of the same application. All share transport, readiness, limits, capture and
scoring. Search does not alter the target or pretend the existing method is best.
Gravity catch prediction is encoder-based motion planning, NOT impact detection.
The optimized method uses finite coordinate-neighborhood search, not unrestricted
RL or arbitrary torque exploration. The baseline's source dependencies are
included in each archived source snapshot.
The baseline wrapper was tuned too; it is not an untouched historical control.
The original `camera_playback/mit_strike.py` itself remains unchanged.

## Configuration and tuning

A JSON file may contain `rules`, `limits`, `plant`, `parameters`, and `plan`.
Unknown keys and invalid values are rejected before a session starts. Example:

```json
{
  "rules": {"acceptance_deg": 0.2, "minimum_entry_speed": 0.20},
  "limits": {"torque": 3.0, "velocity": 3.0, "acceleration": 80.0},
  "plant": {"inertia": 0.02, "gravity": 0.16, "delay": 0.004},
  "parameters": {"powered": {"down_time": 0.22, "up_time": 0.20}},
  "plan": {"screen_repeats": 3, "refine_repeats": 20,
           "refine_rounds": 2, "validate_repeats": 100,
           "endurance_seconds": 300, "seed": 20261001}
}
```

The example above and built-in synthetic defaults are **not calibrated hardware
profiles** or certified motor capabilities. The preserved hardware JSON profiles
record the settings actually tested; see the results document for their evidence.
`plant` only changes simulation. Physical calibration values belong in each
method's parameters. Acceptance/ranking rules never automatically widen to hide
failures. To change scoring after measurement, preserve the original and use:

```bash
./experiment.sh --rescore experiment_results/SESSION --config revised_rules.json
```

This creates a new analysis directory; original trace/score files stay unchanged.
The original session's limits/rules are inherited unless explicitly overridden.
Every trial is rescored using its own saved goal (legacy trials default to 10°);
`--degrees` does not rewrite historical goals.
Re-scoring cannot turn a physical abort or missing trial into a successful stroke.
Changes to motion limits require new matched trials; reanalysis alone does not
prove a trajectory was actually executed under the new limits.

Hardware tuning added fixed `load_torque` feedforward: a positive value is used
throughout a stroke, independently of the upper hold integral. Zero retains the
older automatic hold-bias scaling behavior. This separation prevents slowly
changing static-hold compensation from changing subsequent strikes. Every method
still targets the same user-selected depth. Gravity methods fit the sampled predictive catch
to that target, with continuous acceleration into withdrawal; they do not wait at
the bottom. Powered methods expose bottom curvature without adding a bottom pause.

### Exact finite job lists

`--jobs FILE` runs an archived, finite list of parameterized jobs through the same
controller and scorer. It requires `--headless`; it cannot be combined with
`--campaign`. Each job specifies `method`, `parameters`, `repetitions`, `stage`,
and either `interval` (rest after completion) or `intervals` (requested start-to-start
periods). Start intervals are scheduled from the preceding actual release; no
catch-up bursts are issued after lateness. Every requested and actual start is saved.

The hardware screen/refinement lists and frozen shuffled validation/endurance list
are preserved under `config/experiment_jobs/`. The main frozen comparison holds one
session anchor across methods, uses 100 validation and 100 mixed-interval trials
per mode, and does not change parameters mid-run. A whole completed job list with
both stages records campaign completion; partial/interrupted lists cannot declare
a winner.

## Finite campaign

`--campaign --method all` evaluates every mode. `--stage screen`, `refine`, or
`validate` stops after that phase; `all` also includes endurance.

1. Screen the default plus bounded one-parameter neighbors, same budget per mode.
2. Shuffle execution order with a recorded seed; no disappearing failures.
3. Refine promising valid configurations in smaller neighborhoods between trials.
   Accuracy around the selected goal takes priority until configurations are depth matched.
4. Freeze parameters and run independent validation repetitions.
5. Exercise validated modes at mixed requested start intervals (including short
   bursts and long pauses). Log actual release lateness; do not overlap strikes or
   hide missed intervals with catch-up commands.
6. Write JSON and Markdown comparisons. If nothing qualifies, report that instead
   of inventing a winner or retrying forever.

Defaults imply thousands of trials and potentially substantial raw data. Small
plans are useful for software checks, but cannot qualify as 100-trial validation.
The campaign is a reproducible initial search, not proof of a global optimum.

## Physical execution boundaries

A spawned process is the sole controller; Tk/ROS scheduling does not clock motion.
The physical backend reuses the confirmed drive-setup sequence and timestamped
private J7 CAN receiver. It closes the gripper, centers, plays record3 with the gripper closed, then
enters MIT once for J7 and keeps J1–J6 at the recording endpoint and the gripper
closed. Only J7 moves during subsequent strikes. Left-arm traffic remains
state-query-only. There are no per-strike mode changes or above-anchor backswings.

The common command layer bounds estimated TOTAL torque (PD plus feedforward),
not merely feedforward torque, and its slew. This is an estimate using the latest
feedback, not a force sensor or a guaranteed instantaneous firmware torque limit.
Commands stay in the downward corridor; reference curves are checked before
execution. Gross depth/speed excursions, missed active deadlines, drive faults,
held-joint drift, supervisor loss, and stale feedback terminate the batch. On a
motion fault, relaxation is requested BEFORE scoring or disk work. Disable
confirmation failures are reported, not hidden. Software cannot guarantee CAN
commands reach a disconnected drive.

The hardware comparison kept common limits of 3 Nm estimated total torque,
3 rad/s commanded velocity, 80 rad/s² reference acceleration, and 8000 rad/s³
reference jerk. These are the tested envelope, not measured maximum motor capability.
Its emergency corridor allows up to 12 degrees down and 0.6 degrees above the
anchor. At the historical 10-degree goal, a scored strike failed above
10.2 degrees down or 0.15 degrees of return overshoot. Current depth acceptance
follows the selected goal as described above. The wider emergency upper bound lets a small failed return
be measured rather than turning every quality failure into an abrupt shutdown.
MIT arming has a separate bounded preparation velocity allowance; active strikes
retain their tighter limit. Release waits for fresh feedback rather than counting
an old USB/CAN hold reply as a new stroke sample. Disable verification drains the
short transmit queue before state polling.

A data-invalid or depth-invalid completed stroke is saved as failed. A rejected
reference is saved without executing it. Neither is silently retried. Real
hardware faults stop the campaign; they do not trigger autonomous re-enabling.

## Permanent evidence

Each session in `experiment_results/` contains:

- `session.json`: backend, common rules/limits, synthetic plant (if applicable),
  initial goal, source manifest and content hash.
- `source/`: exact program/method source snapshot, supporting controller files, model/zone,
  and the exact record3 JSON used for that session.
- `events.jsonl`: requested batches/campaign plans, seed, and the held recording-end reference.
- One unique directory per trial: immutable metadata including that trial’s goal/zone, full-rate `trace.csv`, and
  `score.json`, including failed and rejected attempts.
- New uniquely named JSON/Markdown reports, rather than replacing older analyses.

Trace rows include actual sample timestamps, position/velocity/drive torque,
commanded position/velocity/gains/feedforward, estimated total torque, limiter
activity, method phase, other held-joint positions, and temperature when available.
An interrupted/incomplete trial retains metadata and any flushed trace; absence
of a final score is not a successful trial. CSV streaming occurs on a separate
writer thread; a hardware logging backlog stops instead of silently dropping data.

## Offline verification commands

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD:${PYTHONPATH-}"
STRIKE_LAB_OFFLINE_ONLY=1 python3 -m unittest discover -s tests -p 'test_strike_lab*.py' -v
STRIKE_LAB_OFFLINE_ONLY=1 python3 tests/strike_lab_ui_check.py
STRIKE_LAB_OFFLINE_ONLY=1 python3 tests/strike_lab_launch_check.py
STRIKE_LAB_OFFLINE_ONLY=1 ./experiment.sh --simulate --auto-run --exit-after-batch --method all --repetitions 1
```

The supervisor also checks for an unexpectedly exited physical worker. Only if
that worker had started physical setup and had not confirmed relaxation, a
separate right-only disable fallback is attempted after it has exited. It sends
no enable, mode-change, position, or strike commands. This path is mocked in
software tests and blocked by the offline-only environment setting.

A short, reproducible software campaign is available (not a physical calibration):

```bash
STRIKE_LAB_OFFLINE_ONLY=1 ./experiment.sh --headless --fast --method all --campaign --config config/experiment_software_smoke.json
```

See [the original software verification record](diagnostics/strike_lab_software_verification.md)
for the completed checks and preserved synthetic runs.

The record3/adjustable-depth update is covered separately in
[its software verification record](diagnostics/strike_lab_cymbal/verification.md).
