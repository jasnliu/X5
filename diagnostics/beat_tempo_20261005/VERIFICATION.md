# Editable beat tempo — 2026-10-05

## Implemented
- A Beat BPM textbox in normal beat mode and pure simulation. Default 100 BPM;
  accepts finite fractional values from 20 through 180. Invalid entries disable
  Start/RUN and are independently revalidated by the start handler.
- The field locks immediately at Start/RUN, stays locked through preparation,
  searching and playing, and unlocks after returning to an idle/relaxed state.
- Each App owns its tempo. Both the powered simulation and hardware hybrid worker
  receive the same scaled six-event swing grid, including the opening pickup.
- The hi-hat receives the same quarter-note period. OPEN remains on beats 1/3;
  CLOSE remains on beats 2/4. Signed acoustic advance still moves close and its
  following open together; there is no ride offset.
- Faster-tempo advance and ride matching windows are bounded by the beat period;
  100 BPM behavior is unchanged. The hybrid grid validator now accepts the UI
  tempo range instead of rejecting every subdivision shorter than 200 ms.
- No changes to individual strike physics, depth/boost, motion parameters,
  encoder/rebound/deadline protections, startup calibration holds, model settings,
  recording playback rate, waveform display, or center-before-relax handling.
- Manual hardware tests and recording-only mode have no BPM control.

The practical upper estimate (~120 BPM) is an engineering estimate, NOT a new
physical certification. At 120 BPM the shortest swing interval is 167 ms; the
existing short rebound profile itself is 110 ms. Input acceptance up to 180 BPM
is not assurance that physical motors can meet the grid. Deadline and rebound
checks are retained, with no catch-up bursts or widened physical limits.

## Verification
- Full offline suite: **614 tests passed in 138.757 seconds** (`full_tests.log`).
- 14 new tempo tests: fractions/range/NaN validation, independent App schedules,
  start refusal, in-run locking, selected schedule transport, unchanged search,
  actual hybrid controller driven by the simulated plant at 60/87.5/120 BPM,
  stop/full return, opening/closing beat IDs and paired advances across 20–180
  BPM, pickup epoch scaling, and continuous delayed audio-feedback convergence
  at 60/120/180 BPM with both positive and negative corrections.
- Real Tk widgets in pure simulation: typed invalid input and then 87.5; completed
  center -> recorded playback -> fixed-depth swing -> Stop -> center -> relax;
  confirmed locked input while active, then unlocked it and selected 120.
  The physical CAN constructor was forbidden. `ui_tests.log`, `bpm_ready.png`,
  and `bpm_locked.png`; both screenshots visually reviewed.
- Normal hardware-mode App routing at 120 BPM: all arm/ESP32/perception I/O mocked,
  including a fresh post-RUN status reply. Successful one-RUN preparation,
  synthetic first-hit selection plus original 0.5-degree boost, scaled hybrid
  request and 0.5-second hi-hat period, then existing center-before-relax path.
  This is **NOT physical verification** (`hardware_ui_tests.log`).
- Launcher help succeeds (`launcher_help.log`).
- Compared 21,151 pre-existing files against the prior publication snapshot:
  only the eight intended existing source/document files changed. All 21,143
  remaining files, including datasets, motion recordings, firmware, experiment
  tuning and unrelated programs, are unchanged (`preservation.json`).

No physical arm or hi-hat movement, microphone capture, camera operation, model
training, or new GitHub push was performed for this change.

`before/`, `changes.patch`, and `source_hashes.json` record the local changes.
