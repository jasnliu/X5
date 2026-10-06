# Visual-only rolling TONOR waveform

## Implemented

A separate optional process/window opens alongside the live detectors in `start_beat.sh` (not in microphone-free `--test`). It displays the last 10 seconds of TONOR audio on a fixed-width scrolling timeline. Ride detections are red solid vertical lines with Ride labels; hi-hat detections are blue dashed vertical lines with Hi-hat labels. Full instrument names appear in the legend. Separate label rows and compact vertical text make fast/overlapping hits readable.

Each detection is copied after its original control delivery, using the exact existing `event_at` timestamp. Rendering uses that original onset, not the delayed arrival time. The waveform's independent read-only capture uses the same PortAudio ADC-to-host-monotonic mapping as ST7, so both detector streams and the waveform share one clock. The graph is live; delayed markers are inserted back over the appropriate earlier audio and scroll with it. The capture/display does not detect or relabel sounds. Visual gain only affects drawing.

The audio bridge has only optional best-effort, nonblocking event/status mirroring added. The graph consumes separate sockets and has no control protocol, motor imports, launch-wide shutdown handler or readiness role. Closing, failure, or a full visual queue cannot stop/reclassify original event delivery. The original app, audio protocol, calibration, hi-hat synchronization, ride controller, models, thresholds, ST7 source, firmware and datasets remain byte-identical. No countdown/notification was added. No microphone audio files are saved; sample envelopes and markers are bounded in memory.

## Verification

- **600 full-suite tests passed** in 136.241 s, including 18 new tests for event fan-out isolation, original payload equality, closed/broken/full viewer sockets, instrument identity, bounded memory, onset-vs-arrival placement, delayed/out-of-order markers, gap-preserving capture, read-only source routing and launch/mode separation (`full_tests.log`, `unit.log`).
- **Actual live TONOR microphone: 15.18 seconds**, **zero capture gaps**. Both unmodified ST7 models ran simultaneously and became ready on both original and mirrored feeds. PipeWire routing confirmed all three streams on the same TONOR source (`live_routes.json`).
- Closed the real viewer during this live test; both detector processes and original control feeds continued and supplied new ready heartbeats. No motor/serial/CAN control was run.
- The live check observed no model HIT events in that interval; it validates real capture/routing/readiness/isolation, not new physical cymbal-hit recognition. Marker geometry was verified independently with saved WAV waveforms and explicit test events, including late insertion, coincident red/blue markers, exact scroll distance and resize (`ui_verification.json`). No speaker playback or assisted detector events were used in the live check.
- Actual viewer CLI launched, captured, exited cleanly on SIGINT and removed its own sockets (`cli_verification.json`). Closing the parent launcher is also handled by this SIGINT path.
- Reviewed screenshots: `waveform_before.png`, `waveform_scrolled.png`, `waveform_overlap.png`, `live_tonor.png`.
- **562 protected files verified unchanged**, including all original control and detector/model files (`protected_verification.json`).

The first live-check attempt finished capture successfully but its diagnostic routing assertion assumed an application PID property that ALSA/PipeWire does not expose. The corrected check uses the explicit application names plus actual source IDs; no production routing change or assumption-based microphone fallback was made.

No arm or hi-hat motor moved for this visual-only change. All test processes were stopped. `before/` and `changes.patch` preserve the exact code change.
