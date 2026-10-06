# Finite hi-hat closing-sound collection

This is separate from the swing program and does not train or change a model.
Use `./collect_hihat_data.sh --check` for offline geometry/plan preparation only.
`./collect_hihat_data.sh` is the **physical collection command**: use it only
with the normal open/rest hi-hat setup, secured stick, clear workspace, and
both arms initially disabled. Do not run competing serial/microphone/arm owners.

## Fixed batch

- 20 hi-hat-only clips: 16 singles and 4 two-closure examples (24 closures).
- 10 real mixtures: one closure plus one hybrid ride attempt per clip.
- 10 six-second background clips with no closure commands.
- After capture, copy 20 existing ride-only WAVs as hi-hat negatives. Create new
  **header-only** CSVs for those negatives, not copies of the ride-positive CSVs.

Thus the intended export is 60 clips: 30 positives, 30 negatives, 34 closure
onsets. Actual counts depend on completed capture and offline review; failed or
ambiguous material must not be silently labeled as a verified positive.

Hi-hat operation reuses `camera_playback.hihat.HiHatController` and existing
firmware: motor 2 `C` now closes to 100 degrees, `O` returns to zero, heartbeat every
100 ms. No firmware flashing, motor-1 commands, force/gain changes, or new travel
limits. Each closure opens after 0.6 seconds; pairs repeat after 1.2 seconds,
matching 100 BPM swing command timing. The user-confirmed starting rest position
is the open reference; the firmware does not independently home the mechanism.

## Physical sequence and shutdown

Verify state-only arm feedback and actual TONOR capture routing before motion.
Send ntfy, wait ten seconds, then record hi-hat-only/background clips with the
arm disabled. A separate ntfy warning/countdown precedes arm enabling.

For mixtures, reuse the ride collector's physically tested center + close
gripper, **record3**, fixed-anchor hybrid control, and verified center-before-
relax sequence. No camera is used. Each of 10, 10.5, 11, 11.5, and 12 degrees of
J7 displacement is requested twice. The measured hard corridor remains 12
degrees. Ride-command offsets relative to the closure command span -0.4 to
+0.4 seconds; measured acoustic offsets, not those command offsets, determine
the annotations.

The hi-hat has its own finite serial worker so arm feedback work cannot starve
its heartbeat. Only `C`, `O`, `H`, and `S` are permitted. An abort cancels later
closures. Normal shutdown sends `O`, services the existing controller for 1.5
seconds, then sends `S` and requires the firmware release acknowledgement.
**The firmware does not supply an encoder-position readback**, so this is
command/acknowledgement evidence, not independent proof of hi-hat position.
The arm separately requires measured stationary center before disabling.
Accessory shutdown is requested asynchronously before the arm's return, so
the parent can continue servicing arm feedback without starving the accessory.
A small J7 center overshoot can enter the existing bounded near-center recovery
region during the return: J1-J6 within 0.2 degrees and J7 within 1 degree.
This does not change the stricter measured/settled center requirement for relax,
or the geometry limits during playback and strikes.

Ctrl-C requests controlled recovery; do not forcibly kill a suspended-arm
controller. Major hardware faults can prevent verified recovery and require
operator attention. Existing watchdog/emergency behavior is not bypassed.

## Offline annotation/export

Only after the batch ends and shutdown evidence is saved:

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 label_hihat_data.py \
  diagnostics/hihat_collection/SESSION \
  diagnostics/hihat_collection/SESSION/review.json
```

`review.json` supplies explicit accepted raw-WAV onset times and clip boundaries
or a source `ride_id` for copied negatives. Motor command times only locate
review windows; they are not ground-truth acoustic onsets. Mixed closures must
remain positive. Uncertain overlaps require review, not fabricated precision.

Files use the user's requested folders:

```
X5data/hihat/recording/r1.wav
X5data/hihat/timestamp/t1.csv
```

WAVs are 16 kHz, mono, signed PCM16. CSV columns are
`recording_file,hit_time_seconds,label`; positive rows use `hihat_close`, and
negative CSVs contain the header only. Times are seconds from the paired WAV's
start. Source PCM is not normalized/filtered in export. Each physical session
retains its group; copied ride negatives retain their original ride-session
group. Splits remain unassigned. Existing data is never overwritten.

Assistant waveform/spectrogram annotations are marked as visual/signal-review
drafts, not auditory/human-verified ground truth. No training or live detector
predictions are used to create the annotations.

## Collected batch: 2026-10-05

This existing batch was captured with the earlier **110-degree** closure.
The later runtime change to 100 degrees applies to future collection only;
existing audio, labels and saved capture metadata remain unchanged.

The first batch is exported: **60 WAV/CSV pairs, 30 positive clips, 30 negative
clips, 34 closure onsets**. See [dataset details](X5data/hihat/README.md) and
[verification report](diagnostics/hihat_collection/VERIFICATION.md).

All capture blocks completed, but the original automatic shutdown was **not
fault-free**. A near-center J7 overshoot tripped the geometry limit; independently,
the accessory's parent-heartbeat watchdog fired during the arm return. The
hi-hat performed its normal O/wait/S shutdown. After another ntfy warning, a
bounded recovery verified arm center before relaxing; all 16 motors were then
confirmed disabled and fault-free. The original unsuccessful result is retained.

The two cleanup fixes described above have offline regression coverage, not a
second physical end-to-end validation. No additional batch was run just to test
them. Export of this completed capture required a separate recovery record,
strict center proof, post-capture-only watchdog timing, clean audio, release ACK,
and final disabled readback; it did not turn the original result into success.
