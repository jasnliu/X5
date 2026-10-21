# Visual-only snare model integration

## Behavior
- Live perception starts the existing sibling `st7/models/snare_v1` config/checkpoint/normality bundle alongside unchanged ride `v2` and `hihat_v1`.
- Snare detections use the graph-only socket, purple dotted onset markers, and the exact label `snare`.
- All three instrument markers retain their original audio onset timestamps and occupy separate label rows when coincident.
- No snare detections or status enter the robot app, motion/calibration gates, hi-hat timing tuning, ride-depth selection, or snare scheduling. No snare exit handler stops the robot.
- Missing/failed snare is visual-only. Closing the graph does not stop listeners. Microphone-free `--test` still launches no listeners or graph.
- Restart the normal beat launcher to load the new listener. No ST7 model weights/configs or robot workflows were modified.

## Verification
- Full offline regression: **834 passed in 241.59 s** (`full_tests.log`).
- Focused audio/launch/model-selection/isolation checks: **47 passed in 0.51 s** (`targeted_tests.log`).
- Initial full run found one existing test still expecting two bridges; its expected preflight calls, listener count, unique sockets, and controller isolation assertions were updated for the additional visual-only bridge. The complete suite then passed.
- Actual snare model file inference on preserved `X5data/snare/recordings/r1.wav`: one `snare_hit` at 2.15 s (label file 2.149875 s). This is a single saved example, not an independent accuracy benchmark.
- Real Tk/Unix-datagram graph test replayed that model output over the recorded waveform. Ride/hi-hat overlap probes were synthetic and explicitly identified as such. Original onset, purple `snare` label, all three colors, scrolling, and minimum-window resizing passed. Screenshots inspected.
- Live TONOR smoke: all three actual models maintained ready status together; 17.28 s waveform capture, zero capture gaps, bounded history, all four streams (three detectors plus waveform) routed to TONOR. No audio recording saved.
- Stopping only snare left ride/hi-hat heartbeats live. Closing the viewer also left both control feeds live. All test children were then cleaned up.
- SHA-256 preservation: **120/120 protected files unchanged**, including controller workflows, centering/strikes, beat/snare launchers, relevant recordings, all three model bundles, and ST7 detector source.

## Evidence boundary
No robot controller was launched and no robot was moved. No deliberate physical drum hits were made; the live test verifies microphone routing, actual model readiness, and failure isolation, while saved-audio replay verifies detection-to-graph rendering.

## Artifacts
`verification.json`, `preservation.json`, `source_hashes.json`, `changes.patch`, `live_verification.json`, `live_routes.json`, `ui_verification.json`, per-model logs, and graph screenshots are in this directory. `before/` contains exact pre-change copies of edited existing files.
