# Ride/hi-hat calibration: consecutive paired detections

Changed only the hi-hat pre-swing timing-calibration streak semantics:
- Finalized windows missing ride, hi-hat, or both do not increment or reset the
  good-pair count and cannot adjust the offset.
- Two satisfactory paired detections at the same currently applied/target
  offset now lock calibration even across intervening missed detections.
- Bad paired detections, ambiguous pairs containing both instruments,
  out-of-bound estimates and detector timeouts still reset the streak.
- 15 ms tolerance, 45-second nonconvergence timeout, detector-health checks,
  command deadlines, final reopen/full return and safe handoff are unchanged.
- Legacy swing-adaptation and unused legacy snare-calibration semantics are
  unchanged. Bass checkbox/pulse, other instruments and arm return paths untouched.

Updated console/UI waiting text and calibration documentation.

Validation: 172 offline tests passed (tests.log), including the real timing
scheduler with only every 16th ride or hi-hat detection present; both cases lock
on the second paired detection after at least 15 ignored windows. Covered zero,
positive and negative offsets, no false progress before the first pair, bad and
ambiguous pair resets, stale detector reset, offset-change revalidation, prior
bass-checkbox behavior, legacy snare rules and existing simulated handoff.
No physical CAN, camera, microphone or motor operation was performed.

All 532 protected existing files are unchanged (preservation.json). No data
collection or training. Syntax checks passed. A nearby documentation-only stale
100 BPM/600 ms reference was corrected to the existing 80 BPM/750 ms behavior;
calibration tempo was not changed.
