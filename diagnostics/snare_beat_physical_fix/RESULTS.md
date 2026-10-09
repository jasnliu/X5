# Physical snare/main-beat startup and return fix

## Physical acceptance: PASS

Final source was physically exercised with:

```bash
./start_beat.sh --hardware --camera /dev/video4 --verify-playback
```

This is the normal hardware app and real camera/audio/hi-hat readiness, not
`--test` or the older right-only `--recording-only` path. The opt-in verifier
invokes the real Run button and changes only the post-recording endpoint action:
skip alignment/search/beat and request the existing Center + Relax workflow.
It prohibits strike entry points. Centered left-J6 MIT preparation still runs.

Completed on October 7, 2026 (local time):
1. Both arms centered and grippers closed.
2. Left `left_recordings/record1.json` played completely (7.5914 s motion).
3. Right `recordings/record3.json` played completely (4.8 s motion).
4. Beat/search skipped; right returned to center; left reversed the same
   recording to its start, then followed a smooth center leg.
5. Each arm relaxed only after its own measured settled-center proof.
6. All 16 drives confirmed disabled; all support processes exited cleanly.

Final run: `/home/jason/Proyectos3/X5/playback_results/dual_playback_verification/20261008T035106-0a3ffd92`

- Application/control faults: **0**.
- Motor fault frames: **0** across **89,909** real feedback frames.
- Unproven/off-center active disable commands: **0** (independent passive CAN audit).
- Maximum measured center error at disable: left **0.0110°**,
  right **0.0215°**; existing .20°/.12° span/.6 s gates retained.
- Maximum observed motor temperature: **35.0 °C**.
- Snare/ride beat strikes executed: **0**.
- Independent post-exit state-only query: **all 16 disabled**, no fault bits,
  48 query frames and no motion/enable/disable commands.
- The final run needed no fault recovery, re-enable, or emergency stop.

The snare owner tolerated 5913 transient TX queue refusals by
recomputing current commands from fresh feedback, with a maximum successful
command gap of 11.91 ms. These were not drive faults;
persistent congestion and unchanged control/feedback deadline violations still fail.

## Fixes

- Dedicated J6 owner no longer aborts on a momentarily full CAN TX queue, nor
  queues stale strike commands behind retries. Successful-send timing is tracked.
- Left startup path validation follows strict settled-center/MIT preparation.
- Final left centering uses the existing bounded rest-to-rest quintic instead
  of a step into J4's zero boundary. Measured center-boundary handling uses the
  existing .20° center allowance only; command, recording and strike limits and
  measured TCP-zone checks are unchanged.
- Recording process startup re-primes real feedback after blocking spawn.
- A recording worker in endpoint/error teardown no longer claims it is feeding
  the GUI just because its process is alive. GUI queries resume if replies stop.
- Joining the snare worker invalidates the old center proof. Fresh replies and
  a new .6 s settled-center window precede the first left disable packet.
- A latched dual-arm fault no longer re-enters cleanup on every GUI tick.
- Added `--verify-playback`, read-only evidence/audit output and regression tests.

## Evidence

- `run6/app.log`: complete real application/launcher output.
- `run6/can.log`, `run6/can_audit.json`: independent passive wire evidence.
- `run6/summary.json`: acceptance assertions, measured motion ranges and center errors.
- `run6/final_query_disabled.json`: independent query-only final state and raw motor-8 feedback.
- `run6/source.sha256`: exact physically tested source; verified unchanged afterward.
- `run6/both_animations_complete.png`: actual live camera view after both recordings.
- Camera video: `/home/jason/Proyectos3/Y2/recordings/20261007_205108_413512.mp4`.
- GUI result and telemetry: the final-run directory above.
- `preservation_result.txt`: all **344** protected existing data, recordings,
  tuning and standalone snare files remain byte-identical.
- Final source-wide regression result: **774 passed in 239.16 s**
  (`verified_source_tests.log`, offline/fake transports only).

## Earlier failed trials (not counted as acceptance)

Retained in `run1` through `run5` with their logs and CAN audits:
- run1: transient CAN TX queue failure during left playback.
- run2: direct final-center J4 overshoot triggered the recording joint-limit check.
- run3: blocked spawn exposed stale GUI feedback; the step-to-center still overshot.
- run4: expired center proof after snare worker join caused a refused disable batch.
- run5: a live-but-closing recording worker stopped querying feedback while the GUI
  incorrectly continued suppressing its own queries.

All failed trials held or returned safely; no off-center active disable appears
in their independent CAN audits. Runs 1–3 required documented powered-center
recovery with fresh ntfy warnings and no off-center mode switch or relax. Runs
4–5 completed automatic centered recovery but were correctly recorded as failures.

Development-only test invocation/fixture errors are also retained in their logs;
they are not final test results. One overly broad pytest discovery imported a
preview launcher; its parser rejected `-q` before app initialization, with no
hardware flag or motor action. The final suite explicitly targets `tests/`.

## Scope boundary

This proves physical centering, both animations, centered snare preparation,
reverse return and centered relaxation. It does **not** validate a physical
snare/ride beat or acoustic timing. The +11° snare reference, curved strike and
unchanged .220 s strike return remain the same shared `snare.sh` implementation.
