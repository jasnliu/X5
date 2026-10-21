# Separate pre-beat timing calibration

Implemented in normal hardware beat mode only:
- Existing hi-hat angle calibration and ride depth search complete first.
- Fixed 100 BPM paired ride strikes / hi-hat closures; half-beat reopen; snare idle.
- Immediate absolute estimates from finalized acoustic onsets, 200 ms next-pair
  slew bound, unchanged +/-250 ms total bound, no integration of delayed error.
- Two consecutive actual onset errors within 15 ms at the current offset lock it.
- Final open and fresh settled ride return precede the original selected-tempo
  swing; exact learned offset is fixed from the first swing closure onward.
- Existing controlled Stop/center-before-relax paths handle failure/cancellation.
  A 45-second nonconvergence timeout prevents endless calibration.

## Verification
- `STRIKE_LAB_OFFLINE_ONLY=1 python3 -m pytest -q tests`: **825 passed**, 240.12 s.
  ROS environment sourced first. Test discovery must target `tests`, not root
  launch scripts (a root-wide attempt imported a non-hardware ROS launcher).
- Focused regressions: **104 passed** (`targeted_tests.log`).
- Real hybrid controller and spawned synthetic worker cover shared-deadline
  100 BPM calibration, complete returns, ordinary swing, and finish.
- End-to-end virtual plant + actual timing engine/runtime/workflow verifies
  learning, locked-offset handoff to 80 BPM, snare gating, and controlled Stop.
- Real Tk hardware-mode widgets with mocked devices pass (`ui_test.log`);
  a device audit forbids physical CAN/serial/video/audio. Post-search endpoint
  is injected; this is not full physical startup/center verification.
- Positive, negative and delayed detector simulations all converge (see
  `convergence.json`); these are synthetic timing results, not physical claims.
- 15 protected files match the prior published workspace byte for byte
  (`preservation.json`): snare strike/return and scheduling, hi-hat angle/serial
  code, ride tuning, recordings, launcher, cache and independent camera search.

**No physical motion verification was performed for this change.**
