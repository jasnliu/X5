# Main-beat left snare integration

## Implemented

- Normal `start_beat.sh` uses the selected left recording's endpoint for a
  fixed **positive 11 degree J6 snare strike**.
- The reference and powered tuning come directly from
  `snare_lab.runner.prepare_strike` / `load_left_tuning` and
  `snare_lab.motion.build_method`, not from the ride hybrid method.
- Reference descent: approximately **0.147463 s**, curved acceleration;
  unchanged return: **0.220 s**. No snare_lab source was changed.
- Each successive four-quarter-note measure independently selects a uniform
  random integer 1–4. Exactly one strike is scheduled in that measure. Pickup
  and ride triplet extras do not count as main beats.
- The existing right-worker pickup bottom establishes the shared absolute
  clock. Snare releases one reference descent early to put the reference
  bottom on the chosen beat. Actual acoustic contact timing is NOT verified.
- Normal stop cancels future snare strikes and waits for the full current
  stroke/return. Left J6 stays powered in MIT from its centered setup through
  recordings, all hits, reverse playback, and verified center.
- The left recording worker retains its 200 Hz path and sends J6 targets via
  shared memory instead of CSP position frames to the MIT-owned joint.
- Left active-drive disable is guarded by independent settled-center evidence
  (0.2 degree error, 0.12 degree range, 0.6 seconds), except explicit emergency.
  The snare writer joins before the centered left arm is disabled.
- Default remains 100 BPM. Snare-enabled hardware and simulation accept
  20–120 BPM, preserving a complete return for the worst 4 -> 1 boundary and
  allowing the opening pickup to establish the clock before beat-1 descent.
- `--test` visualizes the same reference/measure schedule with no devices.
  Ride rhythm/search, hi-hat angle/calibration, hardware-test/recording-only
  modes, standalone snare.sh, collector, saved audio and motion data are not
  repurposed. No sound-model training or model changes were made.

## Verification (offline ONLY)

- **755 unit/integration tests passed**: `final_tests.log`.
- Actual normal Tk/ROS UI construction/preflight, injected fake motor bus and
  serial controller: `normal_ui.log`. No real CAN/serial/camera/mic access.
- Actual main Tk/ROS `--test` sequence: **8 measures / 8 snare strikes**, choices
  4,1,2,3,4,1,2,3. Stopped during the eighth snare; completed its return, reversed
  the left recording, and independently centered/relaxed both simulated arms.
  `simulation_ui.log`, `simulation_ui_trace.json`.
- Production worker-loop tests use quantized fake J6 MIT packets; confirm no
  CSP/disable packets, clock cancellation/full return, and heartbeat lockout.
- Spawned recording workers tested both forward and reverse with fake J6 MIT
  feedback driven via shared memory. No J6 CSP position frame permitted.
- Source syntax/launcher shell syntax passed.
- **344 existing files preserved byte-for-byte** (audio/labels, recordings,
  configuration and snare_lab): `preservation.sha256`, `preservation_result.txt`.
- Tested source hashes: `source.sha256`; source diff: `changes.diff`; original
  modified-file snapshots: `before/`.

Initial development logs are retained. The first broad run included cache
invalidation caused by concurrent source edits, plus three compatibility
failures subsequently corrected (minimal test fixtures missing attributes and
an existing error-message contract). The final full run used unchanged source
and passed all 755 tests.

## Physical boundary

**No robot movement or physical full-band verification was performed for this
integration.** The ideal reference UI and fake-CAN checks do not establish real
CAN scheduling under concurrent strike load, tracking, loudness, contact timing,
or a fault-free physical run. Prior collector/standalone physical evidence is
not presented as verification of this new main-beat integration.
