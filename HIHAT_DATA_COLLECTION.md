# Snare dataset collection (adapted existing collector)

The former `collect_hihat_data.sh` is now **`collect_data.sh`**. It uses the same
`hihat_collection` package, extended for both arms and all three instruments.
It does not change `start_beat.sh`, `snare.sh`, saved motions, firmware, or models.
The previous collector source is archived under
`diagnostics/snare_data_collection/before/`; existing ride/hi-hat datasets remain intact.

## Commands

```bash
./collect_data.sh --check                 # offline geometry/plan only
./collect_data.sh                         # PHYSICAL: 48 development + 12 test clips
./collect_data.sh --session development   # PHYSICAL: only the 48-clip session
./collect_data.sh --session test          # PHYSICAL: only the 12-clip session
```

Physical commands require secured sticks, a clear workspace, normal open hi-hat
reference, and both arms initially disabled. Do not run another controller.
A delivered ntfy warning and ten-second countdown precede each session.
No learned audio detector decides whether a hit becomes a label.

## Fixed dataset

| Category | Clips | Snare attacks |
|---|---:|---:|
| Snare alone: 15 singles, 5 doubles | 20 | 25 |
| Snare + ride | 8 | 8 |
| Snare + hi-hat | 8 | 8 |
| Snare + both | 4 | 4 |
| Ride only | 6 | 0 |
| Hi-hat only | 6 | 0 |
| Ride + hi-hat | 4 | 0 |
| Background/motor hold noise | 4 | 0 |
| **Total** | **60** | **45** |

Every snare strike commands **11 degrees**, with a separate measured maximum
excursion guard at 14 degrees. It reuses the snare's curved powered method and
unchanged return curve. Ride uses record3 and the existing hybrid collector's
**10, 10.5, 11, 11.5 and 12-degree** targets, not sound calibration. Hi-hat uses
an acknowledged **90-degree** collection-local angle (`A90`, then `B`/`O`), not
the legacy `C` command's 100-degree default. No firmware flashing is performed.
The snare pose comes from `left_recordings/record1.json`.

Mixed timing offsets vary around the snare command. They are capture scheduling
metadata, NOT ground-truth acoustic timestamps. Clips include lead-in and decay.
The two sessions have distinct raw captures and full center/relax cycles; the
12 test clips are held out from fitting. They are still the same kit/room/day,
not a guarantee of broad unseen-day or microphone generalization.

## Powered return and relaxation gate

Strike-mode entry is allowed only at measured center. J6 stays powered in MIT
through left playback, strikes and return; no off-center CSP handoff is used.
Right J7 retains the original collector's powered MIT return. Normal relaxation
requires BOTH arms within 0.20 degrees of center, with at most 0.12-degree motion
span for at least 0.6 seconds and fresh feedback. Disable writes are guarded.
A failed/unconfirmed recovery does not blindly relax an airborne arm; it reports
`NOT RELAXED - ATTENTION REQUIRED`. Use the physical emergency cutoff for a
severe hardware emergency. No new automatic emergency bypass is introduced.

## Review and export, only after verified shutdown

Capture sessions live in `diagnostics/snare_data_collection/` with raw WAV,
audio ADC timestamps, block descriptions, feedback and center/disable evidence.
After checking audio attacks, provide reviewed clip boundaries and onset times:

```bash
python3 label_data.py diagnostics/snare_data_collection/SESSION \
  diagnostics/snare_data_collection/SESSION/review.json
```

Outputs are `X5data/snare/recordings/r1.wav`, `r2.wav`, ... and
`X5data/snare/timestamps/t1.csv`, `t2.csv`, ... . Export appends subsequent session
numbers and refuses existing files or duplicate sessions. Existing ride/hi-hat
files and folder names are not changed. WAV is 16 kHz mono PCM16; CSV columns:

```csv
recording_file,hit_time_seconds,label
r1.wav,2.012500,snare_hit
```

Each actual snare attack gets one row. No-snare clips get header-only CSVs.
Ambiguous clips must be reviewed or recollected; commands are never substituted
for audible onsets. Signal/visual review is identified explicitly in the manifest;
it must not be represented as independent human/auditory ground truth.

## Verified delivered batch

The complete two-session physical run exited 0, with no motor faults. Both arms
were verified centered before relaxation. The exported set contains all 60
numbered pairs and 45 snare labels. One acoustically weak ride-only attempt was
replaced at offline export with an existing verified ride recording, retaining
its original source group/hash. This preserves the exact category counts without
calling an unverified ride attempt a successful acoustic example. See
[dataset notes](X5data/snare/README.md) and [verification](diagnostics/snare_data_collection/RESULTS.md).

For an explicitly reviewed **development ride-only negative**, `review.json` may
supply `source_ride_id`; the exporter verifies its hash and creates a fresh empty
snare CSV. This is forbidden for snare-positive clips and held-out test clips.
