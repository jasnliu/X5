# Separate instrument datasets

- `ride/recordings/*.wav` and `ride/timestamps/*.csv`: the original 30 ride
  examples, moved with their manifest. Ride audio and timestamps are unchanged.
- `hihat/recording/*.wav` and `hihat/timestamp/*.csv`: separate hi-hat closing
  dataset: 60 pairs, 30 positive clips and 30 negative clips, 34 closure labels.
  Singular directory names here are intentional. See its [review notes](hihat/README.md).

Each instrument has its own manifest and independent `rN.wav` / `tN.csv`
numbering. `ride` targets `cymbal_hit`; `hihat` targets `hihat_close`. A ride-only
sound is a negative for hi-hat detection, not for ride detection. A mixture
containing a closure remains positive for hi-hat detection.

Do not merge these labels into the existing ride model. ST7's current data and
models remain separate and unchanged. No model training is part of collection.

See [ride collection](../X5_DATA_COLLECTION.md) and
[hi-hat collection](../HIHAT_DATA_COLLECTION.md).
