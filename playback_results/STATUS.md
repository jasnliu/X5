# Complete: physically verified standalone playback

Selected default: `paced_precise`. Run `./playback.sh`.

- 4.8-second recording movement; exact recorded endpoint commanded.
- Four successful selected-method physical runs, all endpoint errors <=0.02198°.
- All-run medians vs physical old-GUI timing replay: position ripple 11.5% lower,
  motor-reported velocity ripple 17.8% lower. See detailed comparison limits.
- Two deliberate live SIGINT checks passed, including interruption at endpoint.
- Every successful cycle recentered before relaxation. Original gains restored.
- Final plain-command run: `20261003T054749-paced_precise-c1515a`.
- Query-only postflight: all 16 motors relaxed; gains [80,80,60,60,30,30,30];
  both CAN buses healthy with zero current error counters.
- 24 new offline tests and all 358 existing tests passed.
- All 265 protected existing files and record3.json remain unchanged.

Final documentation: `../PLAYBACK.md` and `../PLAYBACK_RESULTS.md`.
Data: `comparison.json`, `comparison.csv`, `comparison.png`, `selection.json`,
`final_verification.json`, `measured_path_proximity.json`, and all run folders.
All methods, prototypes, source snapshots, failed attempts, and raw logs remain.
The former blocked status is preserved as `STATUS_blocked_20261003T0403.md`.
