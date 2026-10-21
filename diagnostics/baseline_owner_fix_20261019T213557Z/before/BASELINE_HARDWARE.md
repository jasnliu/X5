# Baseline-only supervised right-arm mode

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh baseline
```

**Starting the program does not enable, move, center or relax either arm.** Both
arms must already be relaxed, stationary, powered for telemetry, and the right
arm must be at a physically supported resting pose. Close other CAN controllers.
The left arm is query-only throughout; it is never centered or enabled.

At startup, a pure CAN "recovered to error-active" status notice restarts the
read-only measurement batch (at most three retries). It is logged, not treated
as encoder feedback. Every accepted batch still requires all 16 fresh, disabled,
fault-free motor responses. Warnings, error-passive/bus-off, missing feedback,
motor faults and mixed error/status frames still stop capture; no CAN reset,
motor enable or automatic relaxation is performed.

## Workflow

1. Startup reads the actual disabled encoders and five native Gemini depth
   frames, releases the camera, and computes only **Baseline**: the same
   multiple-candidate weighted route selector as the experiment. Progress appears
   in the launching terminal. Only record3's final joint position supplies a
   goal; no recorded route is used. All new plans retain the exact original
   zone3 right-gripper TCP rule and full-arm/custom-stick collision checking.
   Worker logs are in the session's `plan/` directory; the new planner artifacts
   are in `plan/result/plan.json` and its adjacent request/model files. Keeping
   these directories separate prevents a fresh-start directory collision without
   permitting an existing trajectory to be overwritten.
2. **Play animation [P]** is optional and only moves the rendered model. The
   solid robot overlay is a preview, not live joint feedback. The camera mesh is
   frozen, not continuously updated. Hardware progress/hold status is shown in a
   separate, explicitly labeled status line.
3. **Total movement speed** is 0.25–1.00 times the validated baseline timing.
   This scales outward and normal return movement, not the two-second goal hold.
   1.00 never exceeds the existing 0.6 rad/s, 1.2 rad/s², 6 rad/s³ limits. Speed
   locks before preparation/arming. There is no unchecked faster-than-validated
   setting, live speed jump, or physical loop playback.
4. **START REAL ARM** authorizes one cycle with a single click. There is no
   physical-setup checklist, written review or typed startup confirmation. It
   automatically captures fresh disabled encoders/depth and independently
   rechecks the exact previewed curve before starting. The status line says
   **START REQUESTED** while these checks run; **EMERGENCY HOLD** cancels a
   pending start. A changed arm pose requires replanning, not an automatic move
   to the old start. The displayed mesh is replaced once with the checked scene.
5. **Check readiness (no motion)** optionally runs those same checks without
   authorizing movement. Clicking Start afterward uses them if still fresh;
   checks older than 30 seconds are refreshed automatically. Failed, cancelled
   or expired in-progress checks cancel the Start request; they never retry
   motion automatically. Final file hashes, feedback, gripper, joint limits and
   start agreement are still checked before enable. Opening the program or
   completing a readiness-only check never authorizes movement.
6. Normal motion is original start → record3 endpoint → settle → **two-second
   hold** → exact reverse → settled original start → relax
   right joints and gripper. The custom-closed gripper target is the only allowed
   gripper target during motion. No center maneuver or gripper opening exists.

The original resting pose must still be physically supported, and the tool,
camera alignment, left stick and work area must match the setup. Removing the
manual review does not make these facts measurable by software. Return checks
verify joint position and settling, **not physical support**. No review document
or fabricated "physically verified" flags are generated. Old preparation files
must be refreshed; saved plans and recordings are unchanged.

## Stop and recover

- **EMERGENCY HOLD [Space]** freezes trajectory progress. If fresh, healthy
  feedback permits, it replaces the pending target with the measured right-arm
  pose. It does not disable any motor, automatically reverse, or keep advancing
  toward an obstructed goal. Software/servo/inertia latency still exists.
- Ordinary tracking errors, scheduling gaps, GUI disappearance or faults latch a
  hold rather than returning or relaxing automatically. The error remains visible.
- If feedback was unavailable when Hold was requested, use **Recheck hold after
  fault** once feedback is restored. This requires running, fault-free motors;
  it does not reset faults, re-enable stopped drives or clear hardware faults.
  It never repeatedly follows a drifting arm by changing the hold anchor.
- **Reverse traveled path → rest → relax** starts recovery with one click, with
  no popup or typed confirmation. Use it after clearing the obstruction, with
  the return corridor and original support unchanged. It requires fresh
  stationary feedback and a new collision/zone
  check. It follows only the already-traveled original path prefix backward,
  not a newly invented route or a center shortcut. A small tracking-offset
  connector (maximum total joint deviation 0.01 rad) is independently checked;
  a large deviation is refused. Recovery starts/ends at rest, with conservative
  velocity/acceleration/jerk-bounded timing, at most 0.5× nominal rate.
- Hold works again during reverse. A reverse fault returns to a held/error state
  and **does not automatically relax**. Automatic end-of-return relaxation still
  requires the original start pose, settled feedback and disabled-drive feedback.
- **Relax right arm now (torque OFF)** immediately sends disable to the seven
  right joints and the right gripper, wherever they are. **One click, no typing,
  no confirmation popup, no return or centering first. An unsupported arm can
  fall.** This is an explicit manual override, never called by a fault, Hold,
  disconnect, timeout or window close. It cancels pending motion/recovery and
  does not open the gripper or command the left arm. The display reports
  `RELAXED` only after fresh feedback confirms all eight right motors disabled.
  If communication fails, `RELAX_UNCONFIRMED` means torque off is not confirmed;
  it never retries, resumes, re-enables or claims a hold automatically. You can
  click Relax again after restoring communication.
- **Exit controller (no motor change)** exits the stopped controller with one
  click. It does **not** send relax/disable; use the Relax button for that. This
  remains available after an unconfirmed relaxation or a held fault.
- **Close window [Q]** closes the window. If motion was active, the independent
  controller holds and remains available through `--resume`; closing never
  silently releases the arm or kills the holding service.

**A software hold is not a hardware emergency stop.** Power loss, CAN loss,
thermal protection, stalls or disabled drives can make holding torque impossible.
Do not repeatedly drive a stalled motor or assume the UI can prevent a fall after
loss of power. Physically support the arm and use the hardware stop when needed.
An active drive fault or unsafe/unknown recovery prevents software reverse; the
program cannot promise recovery from every fault without physical intervention.

## Independent controller and reconnect

The motor owner runs in a separate systemd user service, not in the renderer or
VS Code cgroup. A lost UI connection/heartbeat requests Hold. It keeps ownership
until return/release or an explicit controller exit. The terminal prints:

```bash
./plan_drums.sh baseline --resume /absolute/path/to/session/controller-XXXXXXXX
```

Reconnecting never automatically enables, resumes outward travel or relaxes.
It restores the controls so you can inspect the error and request checked return.
Do not stop/kill the motor-owner service to clear an ordinary error.
An owner already running before a software update keeps its loaded code. Older
owners can still use the one-click Reverse/Exit controls, but cannot acquire the
new manual Relax command through a UI restart. The UI labels this case; the new
Relax command is available in newly started controller sessions. No update kills
or replaces an active holding owner.

Planner/check workers are memory-capped at 3 GiB, with 128 MiB swap and bounded
runtimes. Only the separate query-only acquisition worker is permitted CAN
sockets; its wire allow-list permits state requests only. Check/planning workers
are denied CAN. The small persistent motor service has a 512 MiB cap and no
execution timeout that could kill a long hold. Raw CAN audit history rotates at
four 8 MiB files; long holds do not accumulate unbounded in-memory data.

## Collision and scene limits

The camera is frozen between explicit captures. Missing surfaces, people entering,
objects moved after capture, and unmodeled left attachments are not made safe by
choosing Baseline or by zone3. The user must verify the return corridor after a
contact; the program does not automatically claim that reversing is unobstructed.

Execution preparation and checked-prefix recovery use **the same collision
margins as Baseline planning**, not the older executor's stricter hardware
margins. Normally this is 20 mm scene padding plus the unchanged native voxel/
edge allowances (9 + 5 mm), with no additional minimum self-clearance gap.
**Self-collisions are still checked**, as are the complete arm, custom gripper,
stick, zone3 and joint limits. Any explicitly requested larger Baseline margins
are preserved. The execution proof must match the checked request's margins.

Fresh depth, a changed pose or an obstruction can still cause preparation or
recovery to be refused. This does not certify physical tracking accuracy; there
is no extra hardware-specific clearance reserve. The older `live --execute`
mode is unchanged and still uses its separate 50/40 mm requirements.

## Offline rehearsal (no CAN, no camera)

```bash
./plan_drums.sh baseline --simulate /absolute/path/to/plan.json
```

This uses the same controller state machine and independent service with a
fake motor bus. The window is labeled MOCK. It includes mock contact/CAN-fault
buttons to exercise hold, blocked recovery, restored feedback and reverse.
Simulation recovery explicitly does not claim a physical collision proof. Even
a real-scene experiment plan is accepted only for this **mock** rehearsal, never
as an executable hardware artifact.

Tests of this mode do not establish real tracking accuracy, stop latency, torque,
physical support, or obstacle clearance. Initial physical trials need separate
operator authorization and supervision.
