# Workspace curve: three-window, selected-arm hardware workflow

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh --hardware         # Right arm (default), destinations/right
./plan_drums.sh --hardware --left  # Left arm, destinations/left
```

Choose one arm per workspace. The opposite arm stays relaxed and query-only;
no enable, hold, opening, or disable commands are sent to it. Omitting the arm
flag defaults to right. `singlearc` and `baseline` remain compatibility aliases; both now use Workspace curve.
The three windows identify the selected arm; it cannot change mid-session.

Saved-input/native UI and simulated-controller verification for this integration:
[`diagnostics/hardware_workspace_20261022/REPORT.md`](diagnostics/hardware_workspace_20261022/REPORT.md).
No physical motion was performed in that verification.

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
   The selected custom claw/stick remain fixed attachments in the calibrated model.
3. **Control panel.** Destination/checkpoint selectors, five always-clickable action buttons, a status message, and a small
   bounded history. No setup checklist, typing confirmations, readiness button,
   mixed test controls, or manual torque-off button. The fifth button, **Cancel
   calculation / pending start**, complements the four motion/planning actions.

Close a preview window to hide it; the control panel's **Windows** menu brings
that same window back. There are no extra renderer windows. Closing the control
panel closes the workspace. An active independent owner is told to HOLD, never
relax, and remains reconnectable.

## Buttons and normal workflow

### 1. Calculate trajectory

Place the selected arm in its physically supported, relaxed starting pose, with
both arms disabled and the selected custom gripper closed. Choose a saved destination
(the first saved destination is initially selected), optionally **Add checkpoint**
for up to three total destinations, then click **Calculate**. Each dropdown excludes
the preceding destination; A → B → A is allowed. Remove last / Refresh destinations
remain available. Selecting, adding, removing or refreshing does **not** plan.
Changing a selection cancels stale calculation/eligibility; Calculate starts the new
selection. An active motor run retains its fixed selection until it returns to rest.
Keep the selected arm still from this click through planning and waiting to Start.
The unpowered opposite arm may sway; its position/opening does not invalidate a
calculation, stop playback, or prevent return.

The program anchors the current encoder pose, takes five full-FOV native depth
frames, and runs **Workspace curve**, using the same independent Cartesian-first
broad-curve proposals, geometric ranking, 120-second per-leg search budget and
settings as the experiment mode. It uses only destination endpoints—not recorded
interior motion. The selected arm's approved timing dispatcher is unchanged:
right adaptive fixed-path timing; left bounded C3 corner smoothing when needed.
The experiment-only Vulkan GPU optimizer does not change this hardware timing
policy or the separate `--hardware-sparse` fork.
Each successive checkpoint leg begins at the exact previous endpoint. A multi-
checkpoint mission also plans a separate final-checkpoint → acquired-start return;
it does not retrace earlier checkpoints on the normal return. Every leg uses the
same original captured scene/self-mask, static passive-arm snapshot, geometry,
collision margins and zone (right zone3 / left zone1, original gripper point).
Full arm/custom claw/stick checking remains. The accepted left traditional-grip
model and closed gripper targets are unchanged (+14.16° raw / -14.16° internal
left; +7° internal right). Neither mode opens the gripper.

Timing refinement and exact-curve validation follow the search budget. The bounded
worker permits the same timing overhead per leg as experiments, including the return,
without lowering refinement quality or increasing the memory cap. Progress streams
to the terminal. No saved experiment plan is promoted to hardware eligibility:
Calculate supplies current measured joints and the current captured scene.

The exclusive query observer stays active **throughout calculation**, not just
while capturing the camera. Planning, retiming and final curve validation run
in a separate, device-free process so long native optimizer calls cannot block
the encoder reader's Python thread. The processes share the existing 3 GiB
calculation memory limit; Cancel stops the whole calculation group. No encoder
timeout, freshness check or motion limit is loosened. Every selected-arm encoder batch is checked against
the click's anchor. Selected-arm movement is latched even if it returns to the original
pose before planning completes. The existing 0.01 rad summed-joint and
0.35-degree per-joint stationary tolerances apply only to the selected arm. Passive-arm
angles/opening still update the live display and the measured collision snapshot,
but are not stillness gates. Missing/faulted monitoring still invalidates the plan.
The UI does not claim to detect selected-arm movement below those limits.

Calculate also performs the dense exact-curve collision/zone validation against
this same capture and saves its execution packet. No extra camera check is deferred
to Start, and waiting to preview does not expire the scene after 30 seconds.

After calculation, the background reader continues this guard until Start.
Selected-arm movement causes a visible error and discards execution eligibility. The previous
preview can still be inspected, but **Calculate stays available**. Restore the
setup and click it again; restarting the program is unnecessary. Calculate during
another read-only calculation cancels/restarts it. Calculate during actual motion
or a held fault explains that Hold/Reverse should be used first; it never steals
an active motor owner or attempts a new path while the arm is in the air.

### Preview and speed

Preview playback is optional. Speed is 0.25–1.00 times the validated baseline
movement timing, not a faster-than-validated override. The two-second hold at each destination/checkpoint
is not scaled. Both preview and hardware evaluate the same analytic timing law,
not a new curve through sampled points. Legacy plans remain readable for recovery. Speed remains editable: a change during actual motion changes the
preview and **next** Start, never the already-running arm's speed. The Start click
captures the requested speed for that one run.

### 2. Start trajectory on arm

Click **START** to authorize one cycle on the already checked **same previewed
curve and frozen camera scene**. It does not recapture the camera, replan, or
perform a second expensive scene check. Plan/model/calibration integrity and
fresh selected-arm start feedback are still verified before enable. Exact-law root
checks are preloaded before the owner accepts IPC, with no motor transport open;
Start rechecks the bound file hashes and reuses that validated curve. This keeps
checkpoint validation from blocking controller heartbeats or feedback. No buttons
are disabled. A click during ongoing calculation is not queued for later motion.

Keep obstacles/camera as they were when Calculate captured them. If the kit or
camera moves, click Calculate again. This deliberately uses a static scene, not
live obstacle avoidance; elapsed time alone is not an error.

Normal motion is:

**One destination:** measured start → endpoint → settle / hold two seconds →
reverse → settled original start → relax selected joints and gripper.

**Multiple checkpoints:** measured start → each selected checkpoint in order
(settle / hold two seconds each) → independently planned return from the final
checkpoint → settled original start → relax selected joints and gripper.

Emergency recovery is different from this normal return: Hold freezes the arm;
Reverse retraces only the already-traveled prefix, including any traversed portion
of the normal return, before settling at the original start and relaxing. No fault
silently sends the arm onward or releases it in midair.

No center maneuver, gripper opening, or opposite-arm command is added. The original
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
Workspace curve owner using the CAN locks and reconnect in-place rather than requiring a
CLI restart. It does not steal an unrelated application's locks.

The return check uses the passive arm's current measured pose. That arm
stays query-only, and passive sway during/after checking is not a return veto.
The small selected-arm tracking-offset connector is still limited to
0.01 rad total and independently checked. No arbitrary new return path or center
shortcut is introduced. Hold is available during return too.

### Cancel calculation / pending start

Cancels only disposable read-only planning/acquisition/check work or an unsent
Start authorization. It never disables motors and is **not** a stop for an already
running trajectory; use Emergency Hold for that. Click Calculate to try again.

## Errors do not lock the interface

All five action buttons stay enabled. Requests that cannot be performed report
why and the appropriate next action. Calculation failures clear their worker and
pending Start; selected-arm movement invalidates a plan; failed Start/finished cycles release
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
- `calculate-*/plan/result/plan.json`: calculated Workspace curve path and model.
- `calculate-*/feedback.json`: latest guarded feedback/error, not a growing queue.
- `calculate-*/checked/prepared.json`: exact-route proof on the planning capture.
- `controller-*/`: independent owner, motor audit, status and return proofs.
- `live-encoders/`: bounded idle query audit.
- `three_window_report.json`: UI/session report on close.

Reconnect after intentionally closing a workspace that left a held controller:

```bash
./plan_drums.sh --hardware --resume planner_runs/SESSION/controller-ID
```

Resume restores the owner's saved arm (old sessions default to right). An explicitly
conflicting `--right`/`--left` is rejected before any device access. Reconnecting
does not re-enable or relax the arm. The live owner is authoritative.

Offline rehearsal uses the same three windows and controls, but fake motors:

```bash
./plan_drums.sh --hardware --right --simulate planner_runs/SESSION/plan/result/plan.json
./plan_drums.sh test
```

For a saved left plan, use `--hardware --left --simulate LEFT_PLAN`.
The selected mode and plan must match.

In rehearsal, Calculate explicitly loads the supplied saved fixture instead of
accessing a camera or planning from real motors. Start/Hold/Reverse exercise the
independent simulated controller. No fault-injection/test buttons are mixed into
the production UI. The older single-window module remains only for historical
diagnostic scripts/tests; `plan_drums.sh --hardware` does not launches it.

## Single arc integration verification (2026-10-21)

`diagnostics/baseline_single_arc_20261021/REPORT.md` records the offline backend,
exact-clock playback, native three-window UI and simulated motor-owner tests.
The no-plan display model now explicitly supplies its right-arm joint/stick
metadata, fixing the missing `DisplayModel.stick_link` startup error.
No physical movement was performed for this integration. Hardware tracking and
physical clearance are not established by software/simulated tests.

The subsequent continuous-arc selection correction is documented in
`diagnostics/single_arc_priority_20261021/REPORT.md`. Restart an already-open
program and click Calculate to use the updated selection; archived plans are
not rewritten. The UI, acquisition and hold/reverse/relax workflow are unchanged.

The encoder-monitor isolation correction is documented in
`diagnostics/feedback_isolation_20261021/REPORT.md`: 904 software tests, a full
263-second saved-scene calculation with simulated encoders and no feedback
errors, and 21 native UI workflow checks with simulated playback/recovery.
Restart the program to load the fix. No physical device was accessed for those
tests; the planner, timing policy and encoder freshness limits are unchanged.

## Occupied-volume clearance correction (2026-10-21)

New calculations count each full 10 mm occupied camera voxel once. The old
checker also added a 9 mm voxel-radius allowance to the robot, double-counting
that uncertainty and sometimes rejecting a genuinely clear resting stick.
The requested **20 mm clearance** and **5 mm edge-sampling allowance** remain;
no obstacles are removed and the stick/self-filter geometry is unchanged.
The `occupied_volume_v1` policy is shared by search, final checks and recovery,
recorded in native reports/prepared packets, and included in cache keys. Archived
requests without this field keep their original margin interpretation.

`diagnostics/start_collision_20261021/REPORT.md` documents the exact failed
capture, corrected 4.037-second route, unchanged input bytes, and software tests.
Restart the program and click Calculate to generate a new-policy plan; simply
reopening an old failed plan does not recompute it. No physical motion was used
for this verification.

## Selected-arm Single arc integration (2026-10-21)

Use `./plan_drums.sh --hardware --right` for ride or `--left` for snare. Each mode
uses its matching experiment's model, endpoint and timing policy, with current
encoder/camera acquisition and the same three-window control workflow. The
opposite arm is query-only. `baseline` remains an alias for existing commands.

`diagnostics/singlearc_arms_20261021/REPORT.md` records full saved-scene calculations,
virtual-wire isolation tests, native UI playback/recovery/reconnect tests for
both arms, and the regression suite. No physical movement/device access was
performed; these checks do not establish real-arm tracking or physical clearance.

## 5 mm camera occupancy (2026-10-22)

New calculations use 5 mm cells instead of 10 mm, without changing Workspace
curve, timing, zones, robot/tool geometry or the 20 mm + 5 mm clearance contract.
Both arms use the same map resolution. New hardware captures already contain
5 mm cells; new saved-input experiments/simulated recalculations reconstruct them
from the original full-resolution depth image in a separate `scene_5mm` folder.
Archived scenes/plans retain their recorded resolution and are never rewritten.
The frozen depth preview mesh and motor controls are unchanged. No camera,
encoder or physical motion was used for this change's verification. See
`diagnostics/voxel_5mm_20261022/REPORT.md` for results and limitations.
## Calculation performance (2026-10-22)

Workspace curve and timing quality are unchanged. For multiple destinations,
all geometric routes are chosen serially first, then their timing is calculated
with at most two workers inside the existing aggregate memory-limited job.
The current camera snapshot is still checked independently. Exact timing
mathematics can be reused across snapshots only for an identical complete curve,
original clock, motion limits, solver policy and numerical environment. No old
collision proof or motor authorization is transferred. Same UI, Calculate/Cancel,
preview, holds, independent final return and emergency recovery behavior.

## Optional sparse timing fork

`./plan_drums.sh --hardware-sparse --right` (or `--left`) runs the same hardware
workflow with sparse IPOPT timing. The original `--hardware`, `singlearc`, and
`baseline` modes retain their existing timing. See [SPARSE_HARDWARE.md](SPARSE_HARDWARE.md).
