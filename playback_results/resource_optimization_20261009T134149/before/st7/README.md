# Cymbal hit detector

This project detects the **onset** of the target cymbal strike. It uses a frozen
1.1-million-parameter frame-MobileNetV3 audio encoder, direct log-mel attack
features, and a small causal TCN. The output decoder uses a peak latch and hysteresis so the ringing
tail does not create repeated `HIT` events.

A separate post-detection normality scorer compares each detected strike with
the training set's normal strikes. It does not feed back into the hit detector
and cannot add, remove, or retime detections.

The MobileNet is derived from EfficientSED at commit
`e3449bfd1f4922b81d82410ed842653f462bfca8` and retains its MIT license under
`cymbal_detector/vendor/efficientsed/LICENSE`.

## Run

```bash
cd /home/jason/Proyectos3/st7

# Detect a WAV file.
.venv/bin/python cymbal.py file data/ride/recordings/r32.wav

# List microphone devices, then listen. The configured default is the connected
# "TONOR TD510 Dynamic Mic". Use --device to override it when needed.
.venv/bin/python cymbal.py devices
./run_cymbal.sh
./run_cymbal.sh --device 3
```

Each live event is one JSON line such as:

```json
{"event":"HIT","time_since_start":2.04,"score":0.97,"normality_score":89.4}
```

`score` is the existing onset-model output. `normality_score` is a separate
0–100 acoustic-similarity value: higher means closer to the reference normal
hits. It is gain-normalized and based on the first 350 ms of the strike. It is
not a probability or a measure of musical quality. With the current one-session
dataset it should be treated as a preliminary relative number.

Stop live listening with `Ctrl+C`.

On Linux, selecting the named **TONOR** microphone (including the model configs'
default) automatically uses its specific PipeWire capture source. This converts
the microphone's native rate to the models' **16 kHz**, supports shared capture,
and also works when running `cymbal.py` directly rather than a shell launcher.
The desktop's default source is not changed. Startup JSON reports `device:
"pipewire"`, the requested device, and the selected capture source after opening
the stream. PipeWire and `pactl` must be available; missing/ambiguous TONOR sources
fail explicitly rather than silently recording a different microphone.
Explicit numeric/other device selections and `--device pipewire` remain unchanged.

## Reproduce training and evaluation

```bash
.venv/bin/python cymbal.py train
.venv/bin/python cymbal.py fit-normality
.venv/bin/python cymbal.py evaluate --split all
```

The selected checkpoint and complete event-level report are written to
`runs/cymbal-fmn-tcn/best.pt` and `runs/cymbal-fmn-tcn/report.json`.
Measured results and limitations are summarized in `TRAINING_REPORT.md`.

The separate normality artifact and its complete fit report are written to
`runs/cymbal-normality/reference.npz` and `runs/cymbal-normality/report.json`.

The original WAV and timestamp CSV files are read-only inputs. Feature caches,
checkpoints, and reports are separate derived artifacts.

## Versioned models

The original model is preserved in `models/v1/best.pt`. The updated dataset's
model is trained separately into `models/v2/best.pt`. Each version has its own
`config.yaml`, `manifest.json`, evaluation report, normality reference, and
source snapshot. Both use the unchanged `assets/fmn06_strong.pt` encoder and
read-only WAV/CSV files in `data/`; a manifest is not a backup of those files.
The existing `./run_cymbal.sh` remains on the original checkpoint.

```bash
# Select a version explicitly for live listening:
./models/v1/run.sh
./models/v2/run.sh

# Offline detection/evaluation using a version's settings:
.venv/bin/python cymbal.py --config models/v2/config.yaml file data/ride/recordings/r56.wav
.venv/bin/python cymbal.py --config models/v2/config.yaml evaluate --split test
```

Version 2 preserves the original 39/8/8 recording train/validation/test split
and adds all 30 new recordings (`r56`–`r85`, one physical session) to training:
69 training recordings / 78 hits, 8 validation recordings / 8 hits, and 8 test
recordings / 9 hits. New-session performance is **training-set performance**;
there is no independent held-out new-session test yet.

For future training, use a new config with a new `training.output_dir`,
`cache.directory`, `data.manifest`, `runtime.checkpoint`, and normality artifact
and report paths. Run `cymbal.py --config <new-config> train`, then
`cymbal.py --config <new-config> fit-normality`. Training refuses to overwrite
existing checkpoints, including the legacy `runs/cymbal-fmn-tcn` location.
The legacy commands in the reproduction section are not an overwrite route.

## Instrument data folders

```text
data/
├── ride/
│   ├── recordings/     # Existing r<number>.wav ride recordings
│   ├── timestamps/     # Corresponding t<number>.csv labels
│   └── manifest.json
├── hihat/
│   ├── recording/      # Hi-hat r<number>.wav (user-provided singular name)
│   └── timestamp/      # Hi-hat t<number>.csv; target label hihat_close
├── snare/
│   ├── recordings/     # Local independent copy of the X5 snare dataset
│   ├── timestamps/     # Existing snare_hit annotations
│   └── manifest.json
└── merged/
    ├── recordings/     # 184 unique r<number>.wav files from all three datasets
    ├── timestamps/     # t<number>.csv containing the union of existing labels
    ├── manifest.json   # Full provenance, coverage, hashes, and grouping
    ├── source_map.csv  # All 205 source recordings mapped to merged IDs
    └── annotation_review.csv
```

The merged dataset retains the established labels `cymbal_hit` (ride),
`hihat_close`, and `snare_hit`. Twenty-one duplicate WAV copies are omitted from
the merged output only; original folders and models are not changed. Audio and
numeric onset times are preserved. The merged data contain 174 existing labels.

**This merge does not invent the missing cross-instrument timestamps.**
Thirty-five recordings have expected instruments without onset annotations;
other unreviewed labels are explicitly marked unknown, not negative. See
[`data/merged/README.md`](data/merged/README.md) and `annotation_review.csv` before
retraining. All merged splits remain unassigned, with source splits and original
capture-group constraints retained. Existing model configs still use the original
instrument folders. New reviewed target-specific manifests/configs are required
for training from the merged data.

To reproduce into a fresh folder (existing outputs are never overwritten):

```bash
.venv/bin/python merge_datasets.py --output data/merged_next
```

The recorder currently saves **ride** data into `data/ride/recordings` and
`data/ride/timestamps`. The checker defaults its folder dialogs to those
locations but can still open folders you select elsewhere:

```bash
./data_checker.py --wav-dir data/ride/recordings --csv-dir data/ride/timestamps
```

Both existing model versions now read the ride data through updated configs
and manifests. Audio, CSV contents, and model weights are unchanged. Archived
source/config snapshots and historical reports retain their original paths;
use each version's current `config.yaml` and `manifest.json` to run it.
The reorganization itself did not create hi-hat data; the user subsequently
added recordings and timestamps in the singular folder names shown above.


## Hi-hat closure detector

`models/hihat_v1` is a separate binary onset detector for the **open-to-closed
hi-hat sound** (`hihat_close`), not for all stick hits on the hi-hat. It shares
the unchanged frozen encoder file with the ride models, but has its own trained
TCN checkpoint, feature cache, config, manifest, decoder, and normality reference.
The default ride launcher and v1/v2 weights remain unchanged. This is a standalone
hi-hat runtime; simultaneous ride/hi-hat inference in one shared stream is not
yet integrated.

```bash
# Live hi-hat listening; Ctrl+C stops cleanly.
./models/hihat_v1/run.sh

# Detect closures in a WAV (times are onset positions in the file, not print times).
.venv/bin/python cymbal.py --config models/hihat_v1/config.yaml file data/hihat/recording/r1.wav

# Evaluate only the held-out hi-hat test split.
.venv/bin/python cymbal.py --config models/hihat_v1/config.yaml evaluate --split test
```

Hi-hat events explicitly contain `instrument: "hihat"` and `label: "hihat_close"`.
Live events retain `event: "HIT"` and `time_since_start`, while offline events
use `time` in seconds from the start of the WAV. Notification time is distinct
from estimated onset time. Normality is similarity to hi-hat closure reference
sounds, not detection confidence or beat correctness.

The 60 supplied recording/CSV pairs contain 34 closures in 30 positive clips
and 30 negative clips. Twenty negatives are byte-identical to ride recordings
and intentionally have no hi-hat labels; they are not added a second time.
The version manifest fixes a stratified recording-level split: 42 train clips
(23 closures), 8 validation clips (5 closures), 10 test clips (6 closures).
Each recording belongs to one split, with no duplicate WAV hashes across splits.
No session metadata was supplied, so these are **not session-independent metrics**.
The detector is selected on validation with a 50 ms event-matching tolerance;
see `models/hihat_v1/TRAINING_REPORT.md` for final detection and timestamp results.

The label-aware training loader selects only `data.target_label` annotations
and rejects unknown labels. Legacy ride configs retain their original behavior.
A checkpoint/config instrument mismatch is rejected instead of silently labeling
ride detections as hi-hat detections. Source WAVs and CSVs are not modified.

## Snare-hit detector

`models/snare_v1` is an independent binary onset detector for `snare_hit`.
It uses the same frozen audio encoder but a separately trained TCN checkpoint,
decoder, and normality reference. It does not replace or merge the ride and
hi-hat models, and the default `run_cymbal.sh` is unchanged.

```bash
cd /home/jason/Proyectos3/st7

# Listen for snare hits using the configured TONOR microphone; Ctrl+C stops.
./models/snare_v1/run.sh
# Override the microphone when needed:
./models/snare_v1/run.sh --device 3

# Detect snare hits in a WAV, with onset timestamps in seconds:
.venv/bin/python cymbal.py --config models/snare_v1/config.yaml file \
  data/snare/recordings/r49.wav

# Reproduce held-out evaluation and detailed runtime/preservation verification:
.venv/bin/python cymbal.py --config models/snare_v1/config.yaml evaluate --split test
.venv/bin/python models/snare_v1/verify_model.py
```

Live JSON events carry `instrument: "snare"`, `label: "snare_hit"`,
`time_since_start`, `score`, and `normality_score`. File events use `time`.
Normality is post-detection similarity, not an acceptance gate or a musical
quality rating. Allow approximately two seconds of startup audio before playing;
notification time is later than the estimated acoustic onset.

Training and evaluation now read the local, independent copy at
`data/snare/{recordings,timestamps}`. The config, prepared model manifest, and
preparation script resolve these paths from the ST7 project root. The original
`X5/X5data/snare` folder remains untouched; all 123 copied files are byte-identical.
The copied collection manifest/README, archived training audits, and checkpoint
metadata retain their original provenance. Historical preservation verification
still checks the original source files; it does not use them as training inputs.
The 60 clips contain 45 labeled snare hits and 20 no-snare
clips, including ride/hi-hat mixtures and background/motor noise. Development
data are split into 38 training clips (29 hits) and 10 validation clips (7 hits).
The original separate test capture, `r49`–`r60`, remains held out: 12 clips,
9 hits, 4 no-snare clips. Checkpoint/threshold selection uses validation only.

The test capture shares the same day, room, kit, and microphone placement with
development. Train and validation share a capture. Source annotations are draft
visual signal reviews, not independently human/auditory audited labels. These
are initial-detector results, not a guarantee under new acoustic conditions.
See [the snare training and verification report](models/snare_v1/TRAINING_REPORT.md).
