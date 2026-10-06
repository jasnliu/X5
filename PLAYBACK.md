# Standalone record3 playback

This is the new, isolated player. It does not modify `start_beat.sh`, the old
controllers, the URDF/zone, or any saved recording. `recordings/record3.json`
is read only; all derived trajectories are constructed in memory.

**Selected default: `paced_precise`.** The recording movement takes **4.8 s**,
versus **3.841315 s** in the source recording (25% longer), and 7.09575 s in the
old playback trajectory. This does not obtain smoothness by stretching the
movement to the 13–34 seconds used by the rejected slow experiments.

It combines nearby quintic path smoothing, locally redistributed timing,
200 Hz commands and a temporary position-gain cap of 10. Only the outer
position gains are reduced; speed/current ceilings and inner-loop gains are
not increased. After a soft endpoint hold, original gains are restored with a
guarded 0.5-second ramp for accurate final positioning. There is approximately
two seconds of endpoint settling/restoration/verification after the 4.8-second
movement, separate from approach and recentering. See [physical results](PLAYBACK_RESULTS.md).

## Run

```bash
cd /home/jason/Proyectos3/X5
./playback.sh
```

Keep the working area clear. Both arms must initially be relaxed and the CAN
interfaces must be healthy at 1 Mbit/s. No ROS launch is required. The player
uses the installed system Python, NumPy/SciPy and the existing native motor and
geometry helpers. OpenCV is used only to save optional camera evidence.
That evidence now defaults to **Orbbec Gemini 2 RGB**, using the same stable USB
identity selection as the other X5 camera programs. Close OrbbecViewer first;
`--no-video` still omits capture. See [shared camera defaults](CAMERA_DEFAULT.md).

The cycle is:

1. Validate the recording, trajectory, joint limits, zone and return paths.
2. Send an ntfy motion warning; a failed notification blocks motion. Count
   down 10 seconds before enabling anything.
3. Enable the right arm at its live pose, close the right gripper, and move to
   the existing customized center: J1–J6 = 0, J7 = 1.4 rad.
4. Move smoothly to the recording start and play the selected trajectory.
5. Hold the **exact last joint target from record3**. With the default method,
   soft-settle, smoothly restore original gains, then verify every joint is
   within **0.05 degrees** for at least 0.6 s. Do not relax here.
6. Return smoothly to center and verify settling. Restore any gains still
   modified (such as on interruption or other methods), then recheck center.
   Independently read back the original gains at center before relaxing.
7. Only now send relax commands and confirm all 16 motors report disabled.

The left arm is state-query-only. Centering and approach/return moves are
conservative rest-to-rest quintics; their time is separate from playback time.

## Stop and failure behavior

**Ctrl+C requests a controlled return to center, gain restoration and relax.**
Do not force-kill the program during motion. A second Ctrl+C does not bypass
centering. With lost feedback, a drive fault, or an unsafe return path, it may
be impossible to recenter: in that case it explicitly reports
`RECOVERY FAILED - NOT RELAXED` and does not blindly disable the arm. Drives may
still hold their last targets; inspect the arm and resolve the fault before
another operation. Do not assume that an exited process means relaxed motors.
Power loss and force-killing cannot be made to follow a software return path.

Center/ordinary settling requires every arm joint to stay within 0.20 degrees of its
target, with no more than 0.12 degrees span over at least 0.6 seconds. Runtime
checks include fresh feedback, motor operating state, temperature/faults,
8-degree tracking-error limit, zone membership and missed command deadlines.
Up to 1 degree of passive sag beyond a modeled stop is accepted only for an
inward startup recovery; normal commanded joint limits are not expanded.
The default's final endpoint check is tighter: 0.05-degree error and
0.066-degree span. Its gain ramp stops if the measured pose leaves a
0.25-degree envelope around the exact endpoint target. Gain changes are
volatile, read-back-verified, no higher than saved originals and no lower than
12.5% of them. No firmware save, motor-zero or calibration command is sent.

These are software checks against the existing model/TCP zone, **not whole-arm
collision certification**. Keep people and unexpected objects clear.

## Options and evidence

```bash
./playback.sh --check                 # offline validation only; no CAN
./playback.sh --help
./playback.sh --no-video              # still saves motor feedback/commands
./playback.sh --gripper untouched     # does not enable or command gripper
./playback.sh --method paced_kp20      # retained fast gain-cap candidate
```

Every physical cycle gets a new directory under `playback_results/` containing
its source snapshot/hashes, trajectory metadata, CAN status, full outgoing
control packet log, actual motor feedback, command/pose trace, event ordering,
endpoint/center evidence, gain readbacks and optional camera video. A requested
camera that fails does not invalidate motor-side evidence; its error is saved
in `summary.json`.

All experimental methods are retained in `smooth_playback/trajectory.py` and
available through `--method`; slow variants are labeled as experiment-only.
The comparison protocol and all early/failed trials are also preserved. The
camera points at the cymbal, not the whole arm; saved video is not independent
whole-arm vibration validation.

Offline tests and reproducible physical-data analysis:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest smooth_playback.test_playback
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m smooth_playback.report
```

`python3 -m smooth_playback.campaign` only lists the retained balanced test
order. Adding `--hardware` deliberately runs nine physical cycles, each with
its own notification and safe return. It is not needed for normal playback.
