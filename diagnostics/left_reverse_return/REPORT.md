# Left reverse-recording center return

## Behavior

After normal left-first/right-second playback, Center/Stop now starts these independent returns:

- **Right:** the existing center controller, path, targets, feedback/query cadence and independent center/disable verification.
- **Left:** hold/verify the selected recording endpoint, play the same validated curve backward, verify arrival at the original start, move from that start to the left center `[0, 0, 0, 0, -50, 0, 0]` degrees, then disable only after center settling and confirmation.

The left gripper stays closed at its existing target. Startup centering before playback stays unchanged. If stopped during left playback, only the played prefix is reversed; it does not advance to the end first. If stopped during the initial approach, it reaches the recording start before centering.

The recording JSON files, selected recordings, zones, reference `start_beatTest.sh` and its runtime were not edited. The reverse view evaluates the cached preflighted curve at `T - t`, rather than refitting, changing timestamps on disk, or weakening the existing checks. New measured approach/start-to-center connections retain the existing geometry checks. Hardware reversal uses the existing isolated 200 Hz worker, left J1-J7 send allowlist, mode confirmation, tracking/feedback/gripper checks, endpoint verification and gain restoration.

## Simulation and offline verification

- `focused.txt`: 28 existing dual return, right-path restoration and fault/emergency tests passed.
- `reverse_tests.txt`: 6 additional tests passed, covering full/prefix/zero-duration mathematical reversal, simulated interrupted return, unchanged startup centering, independent right disable ownership, preservation of the left writer during right recovery, and an actual spawned reverse worker using fake motor sockets. The worker reached the original first pose, restored gains, and issued only left J1-J7 commands.
- `ui_check.txt` and `ui_trace.json`: actual Tk/ROS App in pure simulation completed left/right forward playback, reverse-left → original start → center → relax, and independent right center/relax.
- `right_comparison.json` and `matrix/*.json`: **12/12** A/B cases exactly matched the untouched reference right-arm positions, center command bytes, command timestamps relative to first center dispatch, and right disable timing. Normal MIT/CSP, transient feedback/send faults, delayed callbacks, post-strike pose, hybrid-deadline recovery, and all three right recordings were covered. All current runs also completed the new left reverse return.
- The detailed right-path harness uses the real right hardware-control code and finite-speed simulated feedback. The concurrent left worker is represented by a finite-speed fake; the real spawned worker is tested separately with fake sockets.
- Typical deterministic record3/MIT case: right disable at 4.96875 s, both returns complete at 11.75 s. These are simulation measurements, not physical timing promises.

All test processes ran in a private network namespace with a private `/dev`, and read-only production/reference files, with only diagnostics and temporary files writable. No physical arm movement or CAN/device access was performed. Simulation agreement does not establish physical clearance, torque dynamics or real bus timing.

Full regression result and final hashes: `verification.json` and `full_tests.txt`.
