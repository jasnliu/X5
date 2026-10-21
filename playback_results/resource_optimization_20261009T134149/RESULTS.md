# Shared audio resource optimization

## Implementation
- Normal beat launch now uses one shared ST7 process (one capture/encoder, the three original independent heads/decoders/normality scorers) and one multiplexing bridge.
- Instrument control sockets, graph copies, ADC timestamps, cadence, windows, thresholds and 0.4-second lookahead retained.
- Shared-backend exit notifies all three control receivers. Missing optional model bundles retain legacy isolated-launch behavior.
- Before controller/perception startup, Linux MemAvailable must be at least 2 GiB. This never kills or relaxes an active controller; it cannot prevent other applications exhausting RAM later.
- Camera thread reductions were rejected because they slowed inference. Camera resolution, model, frame selection, recording and all camera source bytes are unchanged.
- Robot motion, snare strike, 80 BPM calibration, swing, recentering and safety source bytes are unchanged. No arm/ESP32/CAN operations were performed.

## Detection equivalence
- 27 WAVs: union of the three held-out test manifests.
- 81 model/WAV streaming comparisons against the saved pre-change listener.
- Exact equality of HIT events, confidence, normality, onset/ADC timestamps and progress watermarks, including synthetic-padding detections. This demonstrates non-regression, not perfect classification accuracy.
- Unchanged weights/configs/references verified by hashes (570 protected ST7/Y2 model assets).

## Performance on this machine
Two sequential bounded microphone-only runs, after warmup, 30 seconds measured each:
- Audio CPU: 24.94% of one core aggregate -> 7.83% (68.6% less).
- Aggregate proportional resident memory (PSS): 926.49 MiB -> 371.27 MiB (59.9% less; 555.22 MiB saved).
- Audio processes including bridges: 6 -> 2.
- Capture-to-receiver progress latency: previous medians 167-172 ms / p95 219-228 ms; shared median 153 ms / p95 212 ms. These are progress-delivery measurements, not acoustic HIT latency.
- Offline 6-second-window CPU-work median: 44.27 ms for three independent evaluations -> 18.43 ms shared.
- Camera benchmark: 8 threads ~38 ms/frame; 4 threads ~47 ms; 2 threads ~69 ms. Same detections at all tested thread counts, but slower settings were not deployed.
- No full physical beat stress test or universal guarantee against external system-wide memory exhaustion is claimed.

## Verification
- ST7 current suite: 91 passed plus 4 subtests.
- X5 full offline suite: 865 passed in 235.48 seconds.
- Live audio-only clean shutdown passed for separate and shared backends.
- Fault injection: missing graph sockets did not gate readiness; unexpectedly stopping the shared listener caused all three control receivers to report error and the bridge to exit nonzero.
- Initial camera benchmark encountered an existing unfinished MP4; rerun used two intact saved videos. No videos were changed.
- Initial X5 full suite had one new test-harness Mock-type filtering failure (864 passed); test fixed, final full suite passed. Failed-attempt logs retained.

See audio_resources.json, audio_equivalence.json, camera_benchmark.json, fault_verification.json, preservation.json and final test logs for detailed evidence.
