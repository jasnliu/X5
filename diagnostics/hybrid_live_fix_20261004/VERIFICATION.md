# Hardware hybrid swing verification — 2026-10-04

> Historical report. Its independent waveform review is not current acceptance
> of ST7 operation. See the subsequent [ST7-only physical verification](../st7_handoff_20261004/VERIFICATION.md)
> and corrected depth-search / powered-center-return behavior.

## Outcome

The normal one-button workflow physically completed a **100 BPM swing for
5.021710574 seconds**, with **14 encoder-observed and recorded-audio-confirmed
strikes**, six partial returns, and **zero faults** on the successful run.
The first ST7-confirmed search depth was 11 degrees. The largest absolute
arrival error was 28.944 ms; the existing 80 ms limit was not widened.

Before moving, an ntfy warning was delivered (`notification_6.json`). The
opt-in verifier invoked the normal RUN button, then requested Center + Relax
after five seconds. The customized center was encoder-verified for 0.628 s
(maximum error 0.121 degrees) **before** disabling. A final independent,
audited query-only check confirmed all 16 motors disabled with no fault bits
(`query_final_disabled.json`). No arm controller was left running.

## What was corrected

- Audio hits now carry actual ADC-based onset timestamps; stdout is fully
  drained instead of leaving buffered events behind. The search waits for
  the detector's finalized-audio watermark before declaring a miss, with a
  bounded backlog fault rather than silently advancing past a real hit.
- Hybrid deadline prediction accounts for measured coast dynamics. Fast
  pairs use a braked partial return, instead of releasing while still moving
  rapidly upward. Experiment kick/catch tuning and safety limits remain intact.
- Fresh encoder feedback is acquired after playback-worker startup, avoiding
  a false stale-feedback fault caused by process creation blocking the GUI.
- Ordinary Stop, faults, relaxation requests, and application close follow
  center-before-relax. Explicit major stall/thermal faults retain the
  emergency relaxation exception requested by the user.

## Evidence and limitation

`physical_acceptance.json` records the independent physical acceptance.
The raw TONOR WAV contains 14 distinct high-frequency attacks aligned with
the 14 encoder bottoms. Each attack rises at least 10.51 dB; each metallic
decay has spectral cosine similarity at least 0.965 to the first ST7-confirmed
swing hit. `analyze_physical_audio.py` reproduces this offline analysis.
Recorder-start timing is approximate, not an ADC calibration claim.

ST7 itself reported only two hits during sustained ringing. Consequently,
the original automatic result in
`../../playback_results/swing_verification/20261005T031425-a625f9f5/result.json`
still correctly records failure of its stricter repeated-ST7-detection check.
That historical result was not rewritten. Raw recorded audio plus encoder
telemetry establishes the requested five-second physical beat; this work
does not claim to fix ST7's under-counting of sustained ringing.

Evidence: `physical_run_6.log`, `physical_run_6.wav`,
`physical_run_6_audio.png`, and the adjacent `physical_acceptance.json`.

## Software verification

- Full X5 suite: **460 passed** (`final_unit_tests.log`).
- Final focused suite, including additional center-first safety tests:
  **60 passed** (`final_focused_tests.log`). Counts overlap the full suite.
- ST7 pytest suite: **9 passed** (`st7_tests.log`).
- Actual Tk hardware-workflow routing with substituted transports passed
  (`final_hardware_ui.log`); this check was offline, separate from run 6.
- Recording files 1, 2, and 3 and experiment hardware tuning match their
  original SHA-256 hashes (`protected_sha256.txt`).

## Normal operation

For the camera and recording used in this physical test:

```bash
./start_beat.sh --hardware --camera 0 --recording recordings/record3.json
```

Press RUN once. Normal operation does not auto-stop after five seconds.
The explicit `--verify-swing` flag used for this test invokes RUN automatically
and requests a safe return after five seconds; use it only with authorization,
a cleared setup, and a pre-motion warning.
