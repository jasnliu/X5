# Baseline: three-window arm workflow

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh baseline
```

Opening the program **does not calculate a trajectory or enable, move, center,
or relax either arm**. It reads disabled encoders and captures an initial camera
snapshot for display. Camera or encoder startup failures leave the UI open;
Calculate retries acquisition. Close other applications using the same CAN buses.

## The three windows

1. **Planned trajectory — disconnected preview.** Shows the frozen camera scene
   and the calculated path. Play/Pause, the timestamp slider and speed adjustment
   affect only this preview. Moving the real arm never changes its displayed
   timestamp/pose. Before the first calculation it retains its first measured
   pose, explicitly not a continuously measured display.
2. **Live arm — measured status.** Shows both arms at measured encoder positions
   over the same frozen camera scene. No planned animation is drawn here.
   Feedback older than 0.5 seconds is clearly marked STALE / last known pose.
   The custom right claw/stick remain fixed attachments in the calibrated model.
3. **Control panel.** Five always-clickable buttons, a status message, and a small
   bounded history. No setup checklist, typing confirmations, readiness button,
   mixed test controls, or manual torque-off button. The fifth button, **Cancel
   calculation / pending start**, complements the four motion/planning actions.

Close a preview window to hide it; the control panel's **Windows** menu brings
that same window back. There are no extra renderer windows. Closing the control
panel closes the workspace. An active independent owner is told to HOLD, never
relax, and remains reconnectable.

## Buttons and normal workflow

### 1. Calculate trajectory

Place the right arm in its physically supported, relaxed starting pose, with
both arms disabled and the custom right gripper closed. Click **Calculate**.
Keep the RIGHT arm still from this click through planning and waiting to Start.
The unpowered left arm may sway; its position/opening does not invalidate a
calculation, stop playback, or prevent return.

The program anchors the current encoder pose, takes five full-FOV native depth
frames, and runs the existing Baseline multi-candidate weighted route selection.
Only `record3.json`'s endpoint is used, never its recorded route. zone3's original
right-gripper point rule and full arm/custom stick collision checks are unchanged.
Baseline collision margins are unchanged. Progress streams into the terminal.

The exclusive query observer stays active **throughout calculation**, not just
while capturing the camera. Every RIGHT-arm encoder batch is checked against
the click's anchor. Right movement is latched even if it returns to the original
pose before planning completes. The existing 0.01 rad summed-joint and
0.35-degree per-joint stationary tolerances apply only to the right arm. Left
angles/opening still update the live display and the measured collision snapshot,
but are not stillness gates. Missing/faulted monitoring still invalidates the plan.
The UI does not claim to detect right movement below those limits.

Calculate also performs the dense exact-curve collision/zone validation against
this same capture and saves its execution packet. No extra camera check is deferred
to Start, and waiting to preview does not expire the scene after 30 seconds.

After calculation, the background reader continues this guard until Start.
Right-arm movement causes a visible error and discards execution eligibility. The previous
preview can still be inspected, but **Calculate stays available**. Restore the
setup and click it again; restarting the program is unnecessary. Calculate during
another read-only calculation cancels/restarts it. Calculate during actual motion
or a held fault explains that Hold/Reverse should be used first; it never steals
an active motor owner or attempts a new path while the arm is in the air.

### Preview and speed

Preview playback is optional. Speed is 0.25–1.00 times the validated baseline
movement timing, not a faster-than-validated override. The two-second goal hold
is not scaled. Speed remains editable: a change during actual motion changes the
preview and **next** Start, never the already-running arm's speed. The Start click
captures the requested speed for that one run.

### 2. Start trajectory on arm

Click **START** to authorize one cycle on the already checked **same previewed
curve and frozen camera scene**. It does not recapture the camera, replan, or
perform a second expensive scene check. Plan/model/calibration integrity and
fresh RIGHT-arm start feedback are still verified before enable. No buttons
are disabled. A click during ongoing calculation is not queued for later motion.

Keep obstacles/camera as they were when Calculate captured them. If the kit or
camera moves, click Calculate again. This deliberately uses a static scene, not
live obstacle avoidance; elapsed time alone is not an error.

Normal motion is:

**measured start → record3 endpoint → settle → hold two seconds → reverse →
settled original start → relax right joints and gripper.**

No center maneuver, gripper opening, or left-arm command is added. The original
resting pose still needs physical support; encoders cannot verify that fact.
Final release retains the existing 0.2-degree settling requirement.

A completed or rejected owner is detached automatically. Calculate is immediately
usable for the next attempt/cycle in the **same workspace**, including after a
pre-enable Start rejection. There is no one-use controller left blocking buttons.

### Emergency Hold [Space]

Stops trajectory progress and requests a measured-pose torque hold. It does not
relax or automatically reverse. During read-only calculation/start checks, it
cancels the pending work/authorization. Space works from any of the three windows.

### Reverse to rest + relax

One click requests the whole recovery sequence. If moving, it first requests
Hold. If already held/fault-held, it rechecks healthy holding feedback. After the
existing stationary interval, it requests the independently collision-checked
traveled-prefix return and only releases torque after verified rest.

A failed hold/return request clears the pending recovery intent; fixing a fault
never triggers surprise motion. Click Reverse again to retry. The button stays
clickable with stale feedback, unconfirmed hold or a lost connection. A known
owner reconnects automatically. An explicit Reverse can also discover an existing
baseline owner using the CAN locks and reconnect in-place rather than requiring a
CLI restart. It does not steal an unrelated application's locks.

The return check uses the left arm/gripper's current measured pose. That arm
stays query-only, and passive sway during/after checking is not a return veto.
The small right tracking-offset connector is still limited to
0.01 rad total and independently checked. No arbitrary new return path or center
shortcut is introduced. Hold is available during return too.

### Cancel calculation / pending start

Cancels only disposable read-only planning/acquisition/check work or an unsent
Start authorization. It never disables motors and is **not** a stop for an already
running trajectory; use Emergency Hold for that. Click Calculate to try again.

## Errors do not lock the interface

All five action buttons stay enabled. Requests that cannot be performed report
why and the appropriate next action. Calculation failures clear their worker and
pending Start; right-arm movement invalidates a plan; failed Start/finished cycles release
the old controller session. Connection errors retry without losing the known
owner. There is no automatic retry of physical motion.

This does not make missing power/CAN feedback, stalled/disabled drives or an
actually obstructed return path executable. Such faults still need their physical
cause corrected. The UI remains usable and reports the blocker rather than
hiding the recovery action. No software UI can guarantee holding torque after a
power/CAN/drive failure. No fault, cancellation or window close automatically
relaxes the arm in midair.

Runtime event/CAN-audit/status-file write failures are warnings, not motion
faults. They do not stop healthy playback or block operator recovery. Required
plan/proof files must still be readable and valid.

See [BASELINE_ERROR_REVIEW.md](BASELINE_ERROR_REVIEW.md) for the removed blockers
and the remaining error categories.

## Rendering and resources

Both windows use the same immutable camera scene; snapshots are replaced only
by explicit Calculate acquisition, never Start. They are not live obstacle avoidance.
Each renderer retains its robot/depth geometry; only actor transforms change.
The connected view updates at up to 20 Hz, while a paused preview does not redraw.
No mesh quality reduction was introduced. Camera-only startup uses no CAN.

Idle telemetry uses the exact existing state-query allowlist and exclusive CAN
locks, with one latest pose in RAM and bounded rotating wire logs. Calculation
owns those locks until planning completes and publishes a latest-only feedback
file for the live window. The motor owner publishes its own measured feedback
during motion/hold, with no competing reader. Disposable workers retain the
existing memory/time caps. GUI histories and renderer timing samples are bounded.

Hardware playback still uses the existing `PlaybackFollower` and validated curve,
not a new control algorithm. Sustained following/stall faults hold rather than
relax; original motor limits, gripper target and tracking/settling checks remain.

## Files and recovery

Each launch creates a new session under `planner_runs/`:

- `scene-*/scene.json`: initial display-only frozen scene.
- `calculate-*/capture/`: measured state, camera scene and query logs.
- `calculate-*/plan/result/plan.json`: calculated Baseline path and model.
- `calculate-*/feedback.json`: latest guarded feedback/error, not a growing queue.
- `calculate-*/checked/prepared.json`: exact-route proof on the planning capture.
- `controller-*/`: independent owner, motor audit, status and return proofs.
- `live-encoders/`: bounded idle query audit.
- `three_window_report.json`: UI/session report on close.

Reconnect after intentionally closing a workspace that left a held controller:

```bash
./plan_drums.sh baseline --resume planner_runs/SESSION/controller-ID
```

Reconnecting does not re-enable or relax the arm. The live owner is authoritative.

Offline rehearsal uses the same three windows and controls, but fake motors:

```bash
./plan_drums.sh baseline --simulate planner_runs/SESSION/plan/result/plan.json
./plan_drums.sh test
```

In rehearsal, Calculate explicitly loads the supplied saved fixture instead of
accessing a camera or planning from real motors. Start/Hold/Reverse exercise the
independent simulated controller. No fault-injection/test buttons are mixed into
the production UI. The older single-window module remains only for historical
diagnostic scripts/tests; `plan_drums.sh baseline` no longer launches it.
