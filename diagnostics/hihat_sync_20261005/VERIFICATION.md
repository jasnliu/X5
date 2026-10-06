# Background hi-hat timing servo — implementation and physical verification

## Scope

Normal `start_beat.sh --hardware` now continuously adjusts one signed **hi-hat**
advance, starting at zero on every swing run. Positive sends earlier; negative
sends later. CLOSE targets beats 2/4 and OPEN targets 1/3. Both edges shift;
each close and its following open use the same latched advance. The existing
110-degree/zero targets, serial protocol, heartbeat and firmware are unchanged.

**No ride offset exists.** The ride grid, search/depth selection, impulse,
catch/return/partial-return controller, gains, limits and model are unchanged.
The read-only pickup bottom establishes beat 1; subsequent hi-hat targets stay
on that exact 100 BPM grid. No camera, gripper, recording, return/relax or other
mode workflow changes were made. `--test` and `--hardwaretest` retain their
previous contracts. Existing collectors retain their original controller API.

UI construction, labels and layout are unchanged, including the legacy
“visual only” hi-hat panel wording, as explicitly requested. The new feedback
path is nonvisual; its logs distinguish it from the old display. Startup console
text and README describe the new behavior. The launcher itself is untouched.

## New background logic

- `camera_playback/hihat_sync.py`: deterministic scheduling and acoustic servo.
- `camera_playback/hihat_sync_runtime.py`: 5 ms nonvisual Tk callback adapter,
  with no ride-command capability; cancels on stop/fault.
- Small integrations in `app.py`, `hybrid_workflow.py`, `sound_monitor.py`;
  explicit existing-target hi-hat edge method in `hihat.py`.
- Both models' ADC-derived **monotonic acoustic onsets** are placed on the same
  internal timeline. Receipt/notification delays never substitute for onsets.
- Match only isolated beat-2/4 closures: one ride event within ±140 ms, another
  neighboring ride-grid event, and one closure within 450 ms of its actual close
  command. Both streams must finalize the whole window. Duplicates and old,
  missing, ambiguous or unavailable feedback cannot update the advance.
- Historical estimate = actual advance used + hi-hat onset minus ride onset.
  This avoids repeatedly integrating obsolete errors from delayed detections.
  A five-estimate median, at least three accepted pairs, 15 ms deadband,
  10 ms maximum updates and ±250 ms limits bound adaptation. A skipped pair
  clears the estimate history; the last applied advance is retained.
- Already committed edges never jump. Every subsequent pair changes by at most
  10 ms; its scheduled closed duration stays 600 ms. Serial/Tk jitter still
  affects actual delivery time, which is recorded separately.
- An 80 ms missed deadline causes existing controlled stop, not catch-up.
  Normal STOP/fault still finishes the hybrid return and verifies arm center
  before relaxing. No new arm enable/disable or emergency logic was added.
- Every run logs `playback_results/hihat_sync/<session>/timeline.jsonl`: acoustic
  onsets, original beat targets, command times, advances, pair errors and skips.
  Logs/recorded WAVs are diagnostic evidence, not training labels.

## Physical run (one motion test)

The real ordinary launch graph was used with record3 and camera 0. The only test
wrapper change was extending the existing opt-in `BeatEvidence` duration from
5 to 40 seconds. Its old terminal phrase “five seconds” is not the duration;
result.json contains the measured 40.4099 seconds. No detector events, search
outcomes, encoder samples or motor commands were injected/substituted.

- State-query-only preflight: all 16 arm motors disabled, no faults. Query
  packets audited; no actuation in preflight.
- **ntfy successfully delivered before movement**, followed by ten seconds.
- The ordinary camera workflow made its own accepted alignment adjustment.
- Ride v2 independently selected the first heard depth at **9 degrees**; no
  depth was forced, widened or changed for this test.
- **40.4099 seconds of continuous physical swing; 102 encoder strokes; no faults.**
- 68 scheduled hi-hat edges / 34 closure commands, correctly phased OPEN1/3,
  CLOSE2/4. Maximum observed command lateness 49.7 ms, under the existing
  80 ms abort threshold. No duplicate/catch-up edges.
- **22 accepted acoustic timing pairs, 11 skipped pairs**; one final closure
  was still waiting for finalized audio when the finite test stopped.
- Advance estimate ended at +120 ms; last actual scheduled pair used +110 ms.
  Stop cancels remaining scheduled work; it does not apply the unused estimate.
- All close/following-open pairs had identical advances. Targets exactly match
  `beat1 + index * 0.6`, with zero measured grid arithmetic error.
- **Center verified before relax**: maximum error 0.0989 degrees, settled
  0.6336 seconds. Right motors subsequently verified disabled.
- Independent post-run query found **all 16 motors disabled and fault-free**.
  After relaxation J7 settles away from its powered center pose; the stored
  center evidence is the state BEFORE relaxation, not a claim that an unpowered
  joint remains at 1.4 radians.
- All temporary physical controller, camera, microphone and detector processes
  exited. No hardware was left powered for audio review.

Evidence paths:

- `run_20261005_134848/notification.json`, `launch.log`, `raw.wav`,
  `audio_blocks.jsonl`, `audio_result.json`, `exit.json`.
- `query_preflight.json`, `query_after_run1.json`.
- `../../playback_results/swing_verification/20261005T204914-6a572157/result.json`
  and `telemetry.jsonl`.
- `../../playback_results/hihat_sync/20261005T204941-72baa6d3/timeline.jsonl`.
- `run1_summary.json`, `waveform_review_1.png`, `waveform_review_2.png`.

## Acoustic evidence and model limitation

A third TONOR capture stream recorded the entire test as 16 kHz mono PCM WAV.
All three streams were confirmed on TONOR source 58. The recording has no
reported overflow/errors and zero clipped samples. Recorder ADC-clock offset
spread was 9.72 ms, so sub-millisecond acoustic accuracy is NOT claimed.

Offline waveform/spectrogram review confirmed that the dominant closure burst
actually shifted; this was not merely a changing number in the servo. A causal
high-frequency energy leading-edge estimate, visually checked on representative
panels, measured:

- Four baseline closures at zero advance: median **+86.1 ms** relative to the
  unchanged ride beat grid.
- Fifteen later closures at >=100 ms advance: median **-12.3 ms** relative to
  that same grid.
- Median command-to-closure burst delay: **83.3 ms**.

This confirms physical movement toward the fixed beat timing. It does NOT
independently isolate every weak ride onset once loud closures overlap it.
The detector-reported ride/hi-hat error medians were 68.0 ms for the first five
accepted pairs and 48.0 ms for the last five. Those values are **not acoustic
human ground truth**: the ride model sometimes omitted the expected event or
placed its onset on low-level pre-impact activity. Some unique but unreliable
model events can pass schedule gates. Model confidence is not proof of correct
onset timing; normality was not used as timing confidence.

Therefore: the complete signed servo is implemented, physically ran safely,
and moved closures closer to the fixed beat. **Precise sustained acoustic lock
is not demonstrated with the current ride detections during overlap.** Better
mixed-instrument/onset training may be needed. No model, threshold, config,
normality data, or training dataset was changed, and no training was performed.

## Offline verification

- First full suite: 523 tests passed (136.713 s).
- Final full suite: **524 tests passed** (136.673 s), including explicit serial
  OPEN/CLOSE edges; see `full_tests_final.log`.
- Servo tests cover positive and negative convergence, unequal detector delays,
  multi-second delayed feedback, historical advance accounting, immutable ride
  grid, paired-edge advances, ambiguity/missing data, duplicates, timeout,
  stale epoch, missed deadline/no catch-up, cancellation and controlled stop.
- Existing ride-depth/search isolation and other-mode regressions pass.
- Actual Tk offline panel test passed with all CAN/serial access forbidden;
  `ui_closure.png`, `ui_optional_error.png`, `ui_verification.json`.
- AST comparison confirms existing Tk widget constructors unchanged:
  `ui_unchanged.json`.
- Protected-file hash manifest covers 552 model/runtime/recording/config/data/
  firmware files plus the ride strike controller, bridge and launcher script.
  See `protected_verification.json`. Backups of edited files are under `before/`.
