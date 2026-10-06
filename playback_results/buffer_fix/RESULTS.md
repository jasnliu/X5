# start_beat CAN buffer fix: physical verification

Date: 2026-10-03. Workspace: `/home/jason/Proyectos3/X5`.

## Outcome

The actual `start_beat.sh` completed the entire record3 playback and a verified
center-before-relax return in **both hardware modes**. No pink-zone alignment or
strike ran; the cymbal was not hit. Both final launches and all their support
processes exited cleanly, without CAN/control errors.

| Final physical run | `--hardwaretest` (test3) | `--hardware` (test5) |
| --- | ---: | ---: |
| Scheduled recording duration | 4.800 s | 4.800 s |
| Trace first-to-final playback target | 4.798684 s | 4.803682 s |
| Original endpoint maximum joint error | 0.021979 degrees | 0.021979 degrees |
| Maximum center error at disable | 0.032969 degrees | 0.021481 degrees |
| Center stability history at disable | 0.649572 s | 0.648220 s |
| Center-gated disable packets audited | 24 | 24 |
| Full playback / original gains restored / relaxed feedback | PASS | PASS |

Record3's source duration is 3.841315 s. The existing near-original-speed 4.8 s
smooth trajectory was retained, not slowed further to hide the CAN problem.
Center-return motion takes about 11.6 s separately from the recording.

The 200 Hz setting is a scheduler ceiling. Both physical runs delivered 638
seven-joint target sets during playback (about 132.7 Hz effective rate under bus
pressure), with median intervals about 8.7 ms and maximum intervals under 14.1 ms.
The scheduler skips stale deadlines rather than extending playback. This matches
the delivery behavior of the previously tested standalone method; this repair
does not claim physical 200 Hz target delivery.

## Cause and repair

The GUI's feedback-query bursts and the independent recording worker competed
for a ten-frame SocketCAN transmit queue. The original GUI `Motors.poll()` state
queries bypassed the existing motor-command retry loop and could raise ENOBUFS.

The playback-local transport now:

1. Stops duplicate GUI state queries once the recording worker has established
   fresh feedback; the GUI still drains its independent receive socket.
2. Retries transient ENOBUFS/EAGAIN writes for at most 20 ms, without dropping a
   command. Fatal errors and sustained pressure still fail explicitly.
3. Preserves worker ownership through gain restoration and cancellation.

The physical runs exercised real transient queue pressure: the GUI recovered
43/54 retry events and the worker 4930/4873 retry events, respectively, without
aborting playback. Maximum individual write wait was below 7.4 ms.

The optional recording-only mode bypasses alignment/strikes and independently
gates every disable of an active drive on fresh measured center settling:
maximum error <=0.20 degrees, motion span <=0.12 degrees, for >=0.6 s.
MIT commands and strike entry points are rejected. Failed return verification
holds rather than relaxing at an elevated pose. No ESP32/hi-hat commands run.

## Reproduce the tested no-strike workflow

```bash
./start_beat.sh --hardwaretest --recording recordings/record3.json --recording-only
# Or:
./start_beat.sh --hardware --recording recordings/record3.json --recording-only
```

The usual Run and Continue buttons remain. The physical tests additionally used
`--auto-run`, which automatically continues after a five-second loading pause;
keep hands clear before invoking that option.

**Without `--recording-only`, the original later alignment and striking workflow
remains enabled.** Those later stages were intentionally NOT physically tested.

## Retained attempts and fixes discovered during testing

No attempted method, recording, code snapshot, log, or video was deleted.

- **test1:** arm was off; CAN startup failed before enabling. After the user
  turned it on, independent query-only preflight verified live motor feedback.
- **test2:** entire recording and endpoint verification succeeded. The new return
  helper's CPU-only geometry validation exceeded the feedback freshness window;
  the guard stopped before a return target and did not relax. Following another
  ntfy warning, the existing GUI center-return control recovered to measured
  center before relaxing. This attempt is **not** counted as a clean full run.
  The helper now reacquires fresh feedback after geometry checking and rejects
  measured drift before streaming the validated return.
- **test3:** clean complete hardwaretest run; all required physical checks passed.
- **test4:** normal-mode GUI construction exposed an optional-widget attribute
  error before enabling. Fixed by handling the absent hardwaretest-only widget;
  checked both GUI variants offline before retrying hardware.
- **test5:** clean complete normal hardware run; all required physical checks passed.

The successful method is the retained `paced_precise` trajectory with playback-local
CAN retry/query ownership. No alternative smoothing method was introduced here.
Earlier standalone method comparisons remain in `PLAYBACK_RESULTS.md`.

## Evidence and regression checks

- Clean physical logs and videos: `test3_hardwaretest.log`, `test3_video.mp4`,
  `test5_hardware.log`, `test5_video.mp4` in this directory.
- Hardwaretest center/disable evidence:
  `../start_beat_recording_only/20261003T191158-1011e719/`;
  worker trace: `../start_beat/20261003T191224-f25c4ebd/`.
- Hardware center/disable evidence:
  `../start_beat_recording_only/20261003T191653-75d52cf5/`;
  worker trace: `../start_beat/20261003T191719-b642f6c4/`.
- `postflight/`: read-only live verification of all sixteen motors relaxed,
  original position gains `[80,80,60,60,30,30,30]`, both CAN buses ERROR-ACTIVE,
  current TX/RX error counters zero. Historical interface error-warning counters
  from the arm-off attempt are not represented as zero.
- `ntfy_test*.json`: accepted pre-movement warnings; source snapshots in
  `test3_source/` and `test5_source/`; originals in `before/`.
- `regression_verified.log`: **385 offline tests passed**, real CAN prohibited.
- `ui_*.log`: **four real-Tk offline checks passed**, covering both modes with and
  without the no-strike option. These are not counted as physical evidence.
- `verification.json` and `evidence_audit.log`: assertions over the retained
  physical data, tests, protected-file hashes, and final source hashes.

Re-run the evidence audit without opening any hardware socket:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 playback_results/buffer_fix/verify_evidence.py
```

The current production code hashes match the final physically tested snapshot.
All three original recordings, `start_beat.sh`, `centering/motors.py`, the existing
smooth trajectory implementation, `playback.sh`, and all `smooth_playback/*.py`
files retain their pre-repair hashes. Only camera-playback integration, opt-in
no-strike testing, launcher flag plumbing, shutdown handling, tests, and their
documentation were changed.
