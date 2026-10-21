# Sequential 80 BPM timing calibrations

Implemented ride + hi-hat timing at 80 BPM, followed by ride + snare timing at 80 BPM, then swing at the user-selected tempo. Both require two consecutive finalized acoustic pairs within 15 ms at the current applied offset. The offsets are independent and fixed during swing; ride remains the unshifted anchor.

Snare uses the preflighted snare.sh 11-degree reference, unchanged return/settling and motor limits. The existing worker owns all J6 writes; shared-memory timing adds calibration mode and offset, and release evidence goes back to the acoustic calibrator. Step corrections are bounded by 200 ms and full-return spacing. Already scheduled releases never move. When the learned advance makes a first-bar note impossible before the ride pickup publishes its clock, the random planner excludes that note for the first bar rather than hitting late. Later bars retain all eight positions.

Hi-hat stays idle/open during snare calibration. Both arms finish their current return, with fresh settled feedback, before the next stage. Stop, missing/stale detectors, worker faults, or 45-second per-stage timeout use the existing controlled return/center path, not off-center relaxation or catch-up hits. Missing snare model assets fail normal hardware launch before any process/motion.

The snare detector has a dedicated control socket and best-effort graph mirror. Its receiver is drained from normal audio polling even before calibration to avoid stale startup messages filling the socket. Graph closure/failure never controls motion. Detector/model versions and trained assets were not changed.

Verification:
- 857 offline regressions passed (full_tests_final.log).
- Focused tests include both real timing runtimes plus the unchanged snare controller with an ideal servo and delayed synthetic audio, positive/negative offsets, historical offset rejection, ambiguous/missing pairs, timeouts, stale input, full-return spacing and fixed-offset swing.
- Fake-channel worker IPC test covers actual worker loop, 80 BPM, latched offsets, quantized feedback and CAN queue retries without opening CAN.
- Real Tk widget check uses mocked devices and verifies both stages at 80 BPM, two acoustic matches each, transition to selected 110 BPM, and controlled Stop.
- Ride worker executable AST is unchanged; only its imported calibration period changed.

No physical robot, microphone, camera, or serial hardware was exercised for this change. Acoustic/physical synchronization is NOT claimed verified. Full source snapshots are under before/; change_audit.json and after_sha256.json record scope.

Initial test failures were obsolete 100 BPM/visual-only/socket/IPC expectations and recording-cache invalidation while source edits were in progress. Final full suite ran against stable sources and passed without weakening the cache or hardware checks.
