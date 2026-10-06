# Standalone hybrid-hit audio collection

This is a separate program. It does not change `start_beat.sh`, `experiment.sh`,
the saved recordings, or either detection model. No training is performed.

## Commands

From `/home/jason/Proyectos3/X5`:

```bash
# Offline geometry/plan check only; no CAN, microphone, or motion:
./collect_x5data.sh --check

# PHYSICAL MOTION: run only when the arm setup is ready and people are clear.
# This collects a NEW batch, not labels an existing one.
./collect_x5data.sh
```

The physical command uses the connected TONOR TD510 microphone, checks its
actual PipeWire routing, sends the ntfy motion warning, and waits ten seconds
before enabling. Failed notification delivery prevents enabling. Both arms
must initially report disabled and fault-free. No stick-loading pause, camera,
or ST7 inference is involved.

## One finite powered batch

1. Center the right arm and close its gripper; verify center.
2. Change J7 to MIT mode **at center**, not in the air. J7 then follows the
   recorded path under powered reference control, while J1–J6 use CSP.
3. Play **`recordings/record3.json`**, using the existing 4.8-second smooth
   trajectory, and verify its endpoint. The source recording is not modified.
4. Keep that endpoint as the fixed J7 displacement anchor. Use the experiment
   hybrid controller and hardware tuning at target displacements **10, 10.5,
   11, 11.5, and 12 degrees**. These are downward displacements, not absolute
   joint angles, and not cumulative displacements after partial returns.
5. Collect four isolated examples and two double examples at each depth:
   **20 isolated + 10 doubles = 40 strikes**. Depth order is shuffled within
   each repetition. Isolated examples precede doubles. Pairs target 200 ms
   spacing with the existing 4-degree-rebound partial-return behavior; the
   second stroke completes its full return. The finite controller prevents a
   third release. Wait six seconds after each block's completed return for
   ringdown before the next block.
6. Finish the hybrid return, bring the whole right arm to its customized
   center, verify stationary center, restore/read back original position gains,
   relax, and verify all 16 motors disabled and fault-free. Then stop capture.

The completed batch used about **3 minutes 53 seconds powered**, excluding the
unpowered notification countdown and all subsequent labeling. The hard measured
J7 corridor remains 12 degrees; commanded depth and actual peak are recorded
separately because a hybrid stroke can undershoot its target.

Ctrl-C requests a controlled center-before-relax shutdown, not an immediate
midair disable. If feedback or a return path cannot be verified, the program
reports `NOT RELAXED - ATTENTION REQUIRED` rather than blindly disabling. This
requires operator attention; software cannot guarantee a safe recovery from
every hardware fault. Do not cut power to a suspended arm as a normal stop.
The program's narrowly bounded inward startup/near-center recovery paths do
not widen the strike corridor. Do not bypass the safety checks.

## Separate offline labeling/export

Capture writes a timestamped session under
`diagnostics/x5data_collection/<session>/`, including `raw.wav`, motor feedback,
attempt metadata, audio block clocks, notification/routing proof, and
center/disable verification. The capture process itself creates no labels.

Only after verified center and relaxation, review `raw.wav` and prepare an
explicit `review.json` with accepted blocks and audio onset times. **Motor
release timestamps are not sound labels.** Keep ambiguous/incomplete blocks in
the diagnostic archive instead of exporting them as confidently labeled data.

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m x5_collection.export \
  diagnostics/x5data_collection/SESSION \
  diagnostics/x5data_collection/SESSION/review.json
```

This is offline and opens no CAN connection. It refuses export without recorded
center/disable proof, rejects capture errors and incomplete pair annotations,
and refuses existing filenames rather than overwriting data. The completed
session is already exported; do not rerun its export to the same destination.

Files go to `X5data/recordings/*.wav` and `X5data/timestamps/*.csv`. WAVs are
16 kHz, mono, signed 16-bit PCM. CSV columns match existing ST7 data:

```csv
recording_file,hit_time_seconds,label
r_SESSION_b01.wav,1.500000,cymbal_hit
```

Each row marks a hit onset relative to the beginning of its paired WAV; a
double has two rows. Clips retain unmodified captured PCM, variable background
lead-in, and ringing tails. `X5data/manifest.json` records hashes, provenance,
commanded/measured depths, and review status. All examples from one physical
session share a group and have no assigned training split.

See [the dataset README](X5data/README.md) for the completed batch and its
label-quality limitation, and [verification](diagnostics/x5data_collection/VERIFICATION.md)
for the recorded physical and offline evidence.

## Offline tests

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 \
  PLAYBACK_OFFLINE_ONLY=1 STRIKE_LAB_OFFLINE_ONLY=1 \
  /usr/bin/python3 -m unittest discover -s tests -p 'test_x5_collection*.py' -v
```

The collector uses X5's existing robot code and system Python. Its capture-only
subprocess uses sibling `../st7/.venv/bin/python` for `sounddevice`, without
loading the model. It expects PipeWire/pactl and the existing ntfy helper.
