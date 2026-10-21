# Bass data collection (no training)

`./collect_bass_data.sh --check` validates the 60-clip plan and both arm paths
without opening hardware. `--execute --session train|validation|test` is physical
motion; use only with a clear workspace, secured sticks and accessible emergency
power cutoff. Optional `--blocks 1,2` selects planned blocks for staged collection.

Plan: 12 bass-only (6 single hits, 6 four-quarter sequences), 12 bass/hi-hat,
12 bass/ride, 8 bass/snare, 8 bass/ride/hi-hat, 8 all four. Context hits occur on
beats 2 and 4. Tempos 80/100/120 BPM; nominal context offsets 0 and +/-75 ms.
Every bass strike uses existing firmware `K`: clockwise PWM 180/255, 20 kHz,
50 ms, then coast. No bass calibration, force changes, firmware flashing or models.

The 36/12/12 split is a future-use assignment only, captured in separate physical
sessions. Sessions do not imply different days, rooms or microphone placements.
Raw continuous audio, ADC timing, motor commands and safety evidence stay under
`diagnostics/bass_data_collection/`. Final reviewed clips go to
`X5data/bass/recordings/r1.wav` through `r60.wav`, paired with
`X5data/bass/timestamps/t1.csv` through `t60.csv`.

Normal shutdown keeps both arms powered. After left playback, the LEFT arm first
replays the recording backward to its verified start, then centers. Interrupted
playback reverses only its played prefix; a failed reverse must NOT fall back to
direct centering or off-center relaxation. The right return is unchanged. Existing
strict checks require <=0.2 degree error, <=0.12 degree movement span and >=0.6 s
settled history before relaxation. Ordinary capture/feedback errors do not
permit off-center relaxation. Failed normal recovery enters supervised powered
hold and writes `attention.json`; **do not kill that process**. After diagnosing
the cause, a supervisor can create `retry_center` inside that session directory
to request another validated center attempt. Observed motor fault, overtemperature
or explicit motor-stall errors are the separately logged hardware-emergency
exception. If software cannot stop dangerous motion, use physical power cutoff.

Acoustic onsets must be reviewed from the WAV; schedules only locate broad
windows. `python3 -m bass_collection.export SESSION --review REVIEW.json`
refuses unreviewed/failed sessions, clipping, missing expected hits and overwrites.
CSV header: `recording_file,hit_time_seconds,label`; bass uses `bass_hit`.
Only bass impacts are labeled in these instrument-specific CSVs. Accompaniment
types are retained in the manifest; their timestamps are NOT exhaustively labeled.
Do not treat the missing ride/hi-hat/snare labels as negatives for those models
when merging in the future. Unverified context proposals remain diagnostic-only.
`python3 -m bass_collection.export --verify` audits exactly 60 numeric pairs,
labels, hashes, format, split grouping and preservation of the fixed bass pulse.

Completed collection evidence is recorded in `X5data/bass/README.md` and
`diagnostics/bass_data_collection/RESULTS.md`. Offline tests are not physical
validation; the saved session results and passive audits document the actual run.
