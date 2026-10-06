# Simultaneous ride and visual-only hi-hat detection

## Scope

`start_beat.sh` still uses its existing launcher and workflow. The launcher now
runs two `camera_playback.audio_bridge` instances: default ride uses ST7 `v2`,
and `--instrument hihat` uses `hihat_v1`. Each selects that version's config,
checkpoint and normality artifact directly from the sibling ST7 checkout.
No model, threshold, training data, motor command, trajectory, beat schedule,
camera alignment or center-before-relax behavior was changed.

The hi-hat panel is display-only: green 600 ms detection flash, cumulative
closure count, last score and normality; readiness/errors are independent.
Its object has no arm/controller reference. Hi-hat errors never gate start or
call the arm's failure/recovery paths. The existing physical ESP32 hi-hat motor
controller is unrelated to this new audio panel and keeps its old safety gates.

## Isolation

- Separate Unix sockets and per-message instrument tags isolate hits,
  heartbeats, errors and finalized-audio watermarks.
- The ride bridge rejects hi-hat-labeled model events. The hi-hat bridge
  requires `instrument=hihat` and `label=hihat_close`.
- Untagged legacy events remain ride-only. Receivers reject another instrument.
- A final guard in `_process_sound_hit` rejects explicitly non-ride input.
- Hi-hat missing files or process exit only change its visual status.
- Pure `--test` remains microphone-/detector-free.

For minimal impact on ride behavior, this implementation uses independent
listeners rather than refactoring ST7 into a new shared-encoder runtime. Both
consume the same explicitly routed TONOR microphone through PipeWire. CPU
thread pools are capped at one per model process; checkpoints and inference
settings otherwise remain unchanged.

## Verification

- Real concurrent microphone test: both models reported ready for over **10 s**.
  PipeWire source-output inspection confirmed **both streams routed to TONOR**.
  Each produced 35 finalized-audio updates. Maximum update gap was 0.404 s and
  maximum captured-audio-to-bridge lag was 0.222 s. No overflow/error occurred.
  This was a quiet-room microphone test, **not a physical strike test**.
- Both actual models were run offline on existing hi-hat-only, mixed and
  ride-only WAVs. Hi-hat v1 produced two, one and zero closures respectively;
  ride v2 detected the ride-only example. No audio was played through speakers.
- Actual Tk integration verified the green flash, count/score display and an
  optional detector error while the ride readiness gate stayed true. This used
  an offline real-model event replay, no camera or motor access. Screenshots:
  `ui_closure.png`, `ui_optional_error.png`.
- Unit tests cover cross-instrument rejection, wrong-socket progress/status,
  unchanged ride depth selection, optional error isolation, fresh heartbeat
  timing, model asset pinning, optional missing-model behavior and launch modes.
- **509 tests passed** in the full offline suite: `full_tests_final.log`.
- Protected-model/recording/firmware integrity: `protected_verification.json`.

Evidence is in `live_verification.json`, `live_messages.json`,
`replay_verification.json`, `ui_verification.json` and the test logs.

## Observed model limitation, deliberately not changed

Ride v2 also emitted three detections on the **hi-hat-only** replay clip
`X5data/hihat/recording/r1.wav`. This is the ride model's own acoustic
cross-response, not hi-hat events being routed into the ride search. No hi-hat
veto, threshold change, retraining or classifier arbitration was introduced:
only ride-model events still control the search, exactly as requested.

The first live-test harness mistakenly sampled its readiness-check time before
polling, making fresh receipt timestamps look slightly future-dated. The
listeners were processing normally. The check and the new panel were corrected
to sample time after polling; the final simultaneous live test passed. First
attempt evidence is retained in `first_smoke_clock_check/`.

No robot or hi-hat actuation was performed for this change, and no physical
swing execution is claimed. Any temporary microphone test processes were
stopped after verification.
