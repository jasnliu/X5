# Three-instrument snare collection — results

Completed 2026-10-07 America/Los_Angeles (2026-10-08 UTC).

## Delivered

The existing `collect_hihat_data.sh` was renamed to **`collect_data.sh`**, and its
`hihat_collection` implementation was adapted in place. The offline exporter is
now `label_data.py`. The old source is retained in `before/`, not another active
collector. Defaults: snare **11 degrees**, hi-hat **90 degrees**, original ride
collector targets **10, 10.5, 11, 11.5, 12 degrees**. No sound calibration or model
training was added. Existing `start_beat.sh`, `snare.sh` strike curves, firmware,
saved arm recordings, ride/hi-hat audio and CSVs are preserved.

The new `X5data/snare` contains `recordings/r1.wav`–`r60.wav` and
`timestamps/t1.csv`–`t60.csv`. There are 40 positives, 20 negatives, and 45
`snare_hit` labels. Category counts are exactly 20 snare-only (15 singles and 5
doubles), 8 snare+ride, 8 snare+hi-hat, 4 snare+both, 6 ride-only, 6 hi-hat-only,
4 ride+hi-hat, and 4 background. See `dataset_verification.json`.

## Physical run and shutdown evidence

`physical2/program.log` and `physical2/exit.json` record a real, unmocked
`./collect_data.sh` execution with exit code **0**:

- Development: `20261008T005622-development-4ebb64e9`, 48 clips / 36 snare hits.
- Test: `20261008T010321-test-40bddeb3`, 12 clips / 9 snare hits.
- Both sessions report success, no recovery errors, and verified relaxation.
- All **45 snare commands were 11 degrees**; measured peaks were
  **10.7259–10.9457 degrees**. The measured snare cap remains 14 degrees.
- Ride measured peaks: **10.2424–11.7150 degrees**, inside its 12-degree corridor.
- Both hi-hat workers acknowledged angle 90 degrees / 134 encoder target counts;
  22 closures total, followed by open-return and release acknowledgments.
- Passive CAN audit: **524,091 feedback frames, 0 motor-fault frames**, maximum
  recorded temperature **36 C**.
- Every active-motor disable in the full CAN trace was preceded by fresh
  feedback showing its arm at center: **0 unproven/off-center disable commands**.
- Maximum final center-history errors: left **0.033 degrees**, right
  **0.0445 degrees**; both passed the 0.20-degree error / 0.12-degree span /
  0.6-second stable-history gates.
- Each session's independent final state-query audit confirmed **all 16 motors
  disabled and fault-free**. These observations do not claim a relaxed arm must
  remain exactly at its powered center after release.
- `physical2/physical_video.avi` contains 17,951 readable frames. Extracted snare,
  ride and final centered frames were visually checked.

Both strike joints change mode only at center and stay powered in MIT through
path movement, strikes and return. The final dual-arm transport adds a left
center-proof gate to the existing right gate. No new emergency-disable bypass
was introduced. A stale inherited ride-session ownership key was also fixed so
stopping its worker correctly relinquishes J7 control before powered return.

### Retained initial attempt

`physical1` stopped during initial centering before any strikes or clips. The
old native center target was still moving when a stationary validation began;
geometry then blocked that recovery attempt. It **did not relax either arm**.
After the existing target settled, a separately notified recovery verified both
centers before release (`startup_center_recovery/`). All motors were then
independently confirmed disabled and fault-free. The startup was fixed to hold
right drives at their live pose during setup, then use only the validated
synchronized center stream. The subsequent complete physical run above verifies
that corrected controller. No motor-fault frames occurred in the first attempt
and no active-motor disable was sent there. Its unsuccessful result is retained.

## Audio review and export

Audio capture reported no overflows/clock/stream errors or PCM clipping in either
successful session. All 60 blocks were visually reviewed as waveform/spectrogram
pages, with enlarged attack windows for all 45 snare hits. Blocks 26, 50 and 52
had interference-related onset proposals refined against the first 14 ms of an
isolated snare attack (correlations 0.959–0.970), then visually rechecked in
`corrected_mixture_onsets.png`.

Motor commands only located broad review windows. Labels were placed on the
actual recorded waveform, not on command or encoder timestamps. ADC-to-monotonic
offset spread over each recording is documented in `clock_summary.json`; that
mapping is not treated as physical contact ground truth. The labels are marked
**draft_visual_signal_review**, with no auditory or independent human review.
No measured model accuracy or millisecond annotation guarantee is claimed.

One new ride-only example (block 41) had no clearly verifiable cymbal attack.
Instead of calling that a strong ride example, exported **r41.wav** is a
byte-identical copy of existing `X5data/ride/recordings/r1.wav` (an 11.5-degree
ride example), with a new header-only **t41.csv**. Its original physical-session
group and hash are retained. Thus the final set is **59 new clips + 1 existing
ride-only negative**, still exactly 60 examples with the requested category
counts. The weak new attempt is preserved in the raw capture. See
`replacement_ride_review.png` and the per-session `review.json` files.

Development and test are separate physical capture sessions, not different
rooms/days/microphone placements. Test files r49–r60 are all newly recorded.
There are no cross-split duplicate WAV hashes or session groups. Keep the copied
ride negative with its original group if combining datasets in future.

## Checks

- **66 focused tests pass**: collection/serial, export, center gates, original
  ride collection, standalone snare handoff, and shared centering.
- Offline geometry/plan check and an 11-degree snare simulation pass.
- All existing protected data/motion/strike-config SHA-256 checks pass.
- All output WAV/CSV pairs, sequential numbers, category totals, 45 labels,
  label bounds, PCM preservation, hashes and split separation pass.
- The offline exporter rejects unsafe shutdown evidence, incomplete reviews,
  duplicate sessions, overwrites and positive/test substitutions with ride audio.

Evidence: `final_tests.log`, `export_tests.log`, `check.log`,
`snare_simulation.log`, `preservation_result.txt`, `dataset_verification.json`,
`physical2/can_audit.json`, both source sessions' center/disabled proofs, and the
source snapshot in `verified_source/`. No additional physical motion is needed.
