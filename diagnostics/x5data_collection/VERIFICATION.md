# Hybrid audio collection verification

## Physical success: `20261005T060856-f745b243`

Evidence is in that session directory and `physical_collection_6.log`.

| Check | Recorded result |
| --- | --- |
| Required recording | `recordings/record3.json`; original hash retained |
| Notification | ntfy accepted before enabling; ten-second countdown |
| Microphone | TONOR TD510 route verified |
| Camera / ST7 inference / training | None |
| Blocks / strokes | 30 / 40 (20 singles, 10 pairs) |
| Command depths | 10, 10.5, 11, 11.5, 12 degrees from fixed endpoint |
| Measured peaks | 10.000599–11.407276 degrees |
| Record3 endpoint | max error 0.043959 degrees; stable 0.629213 seconds |
| Powered time | 232.721 seconds |
| Highest reported motor temperature | 36 degrees C |
| Final center settling | max error 0.044457 degrees; stable 0.601860 seconds |
| Center check guarding disable | max error 0.066436 degrees; stable 0.647447 seconds |
| Final motor readback | all 16 disabled and fault-free |
| Audio capture | 243.24 seconds, no reported capture errors |

`result.json`, `events.jsonl`, `query_final_disabled.json`, `microphone.json`,
`notification.json`, `strikes.jsonl`, and the encoder logs support these checks.
This is physical evidence for the completed finite collection batch, not a
claim that all possible fault recoveries have been physically tested.

## Earlier setup attempts

The complete development campaign was **not** fault-free. Five earlier launches
produced no dataset strikes. Their logs are retained:

1. `20261005T055458-f0027524`: initial relaxed pose just outside the normal TCP
   buffer; stopped before enabling. Added narrowly bounded, inward-only startup
   recovery, not an expanded strike zone.
2. `20261005T055721-e3daafae`: CAN queue pressure at stationary hybrid handoff;
   returned to center and relaxed. Added bounded stationary-setup retries.
3. `20261005T055903-cf564d76`: powered CSP-to-MIT mode transfer not accepted;
   returned to center and relaxed. Moved the mode change to verified center and
   kept J7 in powered MIT reference control through record3.
4. `20261005T060425-bd45b3c0`: settling drift during geometry validation;
   returned to center and relaxed. Added settling and feedback servicing during
   expensive path checks.
5. `20261005T060610-05cbc3d6`: small near-center J7 overshoot blocked the ordinary
   recovery geometry check. The arm was **not** blindly relaxed. A separately
   notified bounded return verified center and disabled all motors; see
   `recovery_5.log` and the session's `recovery_verified.json`. Removed the
   inappropriate initial CSP torque bias and added a tightly bounded
   near-center recovery path.

Only the sixth, successful session contributes to `X5data`.

## Offline dataset verification

`verification.json` and `st7_loader_check.json` record:

- 30 WAVs and 30 matching CSVs, with 40 onset rows total.
- 16 kHz mono PCM16 throughout; zero clipped samples.
- Approximately 182.091 seconds of exported PCM, copied from the raw capture.
- All manifest hashes, label bounds, pair counts, and ST7 loader checks passed.
- Measured double-onset intervals about 202–215 ms.
- All 181 protected input/code/model/data hashes unchanged.
- All examples share a physical-session group; no train/test split or training.

The labels remain **draft visual/signal-analysis annotations**. Waveform and
spectrogram review found distinct attacks consistent with the ST7 references,
but no auditory or human review was performed and no labeling-accuracy metric
was measured. The absence of extra attack-review flags is only a heuristic
check, not proof that every sound has been semantically classified correctly.

Offline tests: 13 collection/export tests and 14 existing hybrid controller
tests passed. See `final_collection_tests.log`, `hybrid_regression.log`, and
`final_offline_preflight.log`. Offline test success is separate from the
physical evidence above. No further arm access was used for labeling/export.
