# Standalone right-arm cymbal destination

```bash
cd /home/jason/Proyectos3/X5
./plan_cymbal_destination.sh
```

This is **not a new plan_drums mode**. It produces one simulated right-arm
**prime destination**, not a recording, movement trajectory, timing law or J7
strike. DT1, Y2, recordings, saved destinations, calibration and motor firmware
are unchanged. The shared generated planner model now has the local right-J2
operating bound described below; the vendor/shared URDF is not edited.

## What the launch does

1. Queries both arms using X5's existing strictly read-only encoder observer.
   Motors must already be disabled. It cannot enable, disable, hold, center,
   open/close a gripper, or move either arm. An unavailable/enabled/faulted motor
   stops acquisition with an error; there is no implicit saved-pose fallback.
2. Captures **one paired native-depth/RGB snapshot** with DT1's camera bridge,
   then closes the camera before detection/IK. The right-arm pose is selected
   from disabled-motor encoder batches bracketing the camera's host receive
   timestamp. This is not hardware-synchronized measurement. Passive left-arm
   sway is not a stillness veto; its measured pose is frozen for collision checks.
3. Runs the same Y2 detector and DT1 rightmost-cymbal selection, grid mapping,
   connected-surface fit and green-dot calculation. RGB/depth mapping uses the
   actual intrinsics, distortion and factory extrinsics, not pixel scaling.
   Detection is on this single image, without a temporal bounding-box history.
4. Finds and checks candidate destinations, including an internal bounded
   geometric approach check for promising candidates. Those approach curves are
   **discarded**. No velocity/acceleration optimization or strike is performed.
5. Displays one static pose in the frozen actual camera scene. The other arm
   remains at its captured position. The optional start/destination selector
   changes only this static simulation; there is no Play, Start or Execute button.

The paired-camera/inference child uses `../dt1/.venv`; IK/collision/UI use the
existing X5 experiment environment. This avoids mixing DT1's NumPy 2/YOLO stack
with the planner's numerical environment. No dependencies are installed/upgraded.

## Target and orientation rules

For the selected cymbal, DT1 supplies red surface point `P` and upward unit
normal `n` of its inferred flat disc. The **unaltered** green target is:

```
G = P + 0.0762 * n            # three inches, along the disc normal
```

The modeled stick tip must coincide with `G`. With `d` pointing from the grip
toward the tip and `a7` the **actual world-frame J7 rotation axis**:

- Define shaft-above elevation as `alpha = asin((-d)·n)`, in degrees. Here
  `-d` points **from the tip back toward the gripper**. The physical stick axis,
  its tip/gripper labels and the upward disc normal are not reversed.
- At `alpha = 0`, the stick is parallel to the disc. At `alpha > 0`, the shaft
  toward the gripper lies above a disc-parallel plane **through the tip**. This
  is the approved R1/screenshot convention, independent of camera viewing angle.
- Prefer near-parallel postures while balancing approach quality. There is **no
  arbitrary elevation maximum**. Negative shaft-above elevation is rejected.
- Require `a7·n = 0`: J7's swing plane is perpendicular to the disc plane.
  This uses the complete calibrated joint chain, not a guess that J5 alone
  supplies the roll.
- Check that the existing **negative-J7** strike direction points toward the
  disc, as used by the code launched through `start_beat.sh`. This is only a
  static direction check: it does **not** simulate, validate or generate a strike.

### Inward aiming: hard ±25° range

The fitted disc center `C` (second red dot) now supplies an additional **endpoint
orientation constraint**, not a new tip target. Project both the stick and the
red-target-to-center direction onto the fitted disc's plane:

```
u = normalize((C - P) - n * dot(n, C - P))
v = cross(n, u)
yaw = degrees(atan2(dot(d, v), dot(d, u)))
```

Require `-25 <= yaw <= +25` and a strictly **inward grip-to-tip direction**.
This is a directed angle, not `abs(dot)` or an acute angle between undirected
lines: a reversed stick at 155° or 180° is rejected even if its line looks close
to parallel. Top-down here means perpendicular to the fitted disc, not world-Z.
The two inequalities used during IK/polishing are
`tan(25°)*dot(d,u) - dot(d,v) >= 0` and
`tan(25°)*dot(d,u) + dot(d,v) >= 0`.
Final acceptance independently recomputes the signed angle and inward cosine.

The rule applies to the **destination**, not the captured start or every pose on
an approach. The green target, normal offset and existing above-parallel/J7
constraints are unchanged. If the red target projects exactly onto the center,
or the stick has no in-plane direction, aiming is undefined and rejected rather
than inventing a radial direction. No automatic range widening or target shift.

Model-space tolerances: tip error <= 10 micrometres; swing-plane error <= 0.1
 degree; 0.000001 degree signed-angle roundoff tolerance. These are **numerical
checks, not claims about camera accuracy or physical tip positioning**.

Candidate search uses deterministic multi-start IK: exact parallel first, then
continuous above-parallel search. The inward cone is enforced during fitting,
elevation minimization, posture polishing and final verification. Accurate raw
alternatives are retained in case a refined posture collides.

A bounded shortlist is compared using checked geometric approaches. Selection
favors less joint travel (proximal joints weighted more), the existing workspace
shape score and a small near-parallel preference. The sideways edge preference
is **symmetric and secondary**: it changes the approach score by at most 2%,
favoring ±25° over 0° only among otherwise comparable options. Exact ±25° is
not required, and a meaningfully better route may win at an interior angle.

For reproducibility, the final primary score is weighted joint travel in radians
plus `0.25 * workspace_geometry_score`. The preference-adjusted score is
`primary * (1 + 0.02 * (1 - abs(yaw)/20)) + elevation_degrees/50`.
Shortlisting uses `0.3 * posture_cost` instead of the uncomputed approach score.
These are geometric heuristics, **not movement duration or proof of a globally
shortest route**. Approaches are checked internally and discarded, as before.

The calibrated joint-position bounds are retained except for the measured-coordinate
right-J2 lower operating bound of **0°**. Right zone, custom tools, static left
arm, observed-obstacle collision policy and original collision padding are
retained. Promising approaches get continuous polynomial joint-bound checks,
>=2,001 sampled states and native edge checks at 0.003 rad joint-L1 resolution.
Camera self-masking always uses the acquisition pose, not the simulated goal.

### Generated-goal joint-position reserve

Every newly generated goal keeps **3° at all joint bounds except J2's lower
bound, which keeps 1° above its new 0° operating floor** (goal J2 >= +1°).
The operator reported a physical J2 stop near -1°; 0° is a provisional operating
floor, not an independently certified physical limit. A blanket extra 3° on top
of that floor unnecessarily rejected an earlier actual-camera target; the fixed
1° J2 endpoint reserve preserves additional clearance without stacking that much
restriction. J2 stays free above the floor, not locked at zero. The same shared solver serves this standalone
program, experiment's dt1, and normal `--hardware --right` dt1.

IK fitting, elevation/posture polishing, final acceptance and export enforce the
reserve. The solver re-solves at the **same green target** with the same inward
±25°, above-parallel and J7-plane constraints; it never clips a finished goal.
A slightly more elevated stick is allowed instead of sitting on a joint bound.
If no reserved endpoint/approach is found, no goal is saved; there is no automatic
reserve reduction, target shift or limit widening.

The **additional reserve is endpoint-only**; the **0° J2 operating floor applies
to ordinary goals and the path**, except for an exact acquired rest inside the
margin. An original rest strictly above the reported -1° stop may be retained
below zero; departure/return there must be monotone and never lower than that
rest. This is checked using continuous polynomial extrema and signs. New v7
goals keep +1° regardless of the reference rest (no reserve subtraction).
It is applied after calibration in the same encoder coordinates as start_beat's
degree display. All other bounds, model geometry/calibration, collision padding,
timing profiles and motor settings remain unchanged. Only optimizer starting guesses are projected into the inner
IK interval; captured joints are not changed. For example, the fault capture's
starting J4 remains only 0.248° from its original lower bound and is not rejected
by this new goal rule.

The reserve is a design buffer, **not a measured hardware overshoot guarantee**.
It does not certify physical tracking. The J2 policy excludes commands at the reported stop; its narrow original-rest
transition is not general permission for negative-angle planning. Saved destination metadata includes the policy, operating bounds and
per-joint distances; these are rechecked when opening/exporting and when a new
hardware DT1 goal is validated.

## Output and controls

Each launch gets a unique folder under `cymbal_destination_runs/`:

- `acquisition/`: frozen pair, detected target/disc, actual scene, pose and audit.
- `model/`, `backend_request.json`: exact calibrated geometry/checking inputs.
- `result.json`: status, numerical checks, internal approach summary and provenance.
- `destination.json`: **only on success**, one seven-joint posture. It uses X5's
  destination-compatible JSON container with one sample and zero duration, not
  a playable multi-frame recording.

The preview shows RGB, the depth mesh (selected cymbal blue), inferred green
flat disc, red surface point, an additional red dot at the fitted disc's center,
green tip target, normal URDF/tool visuals and a
purple J7 axis. Mouse navigation, Fit, Camera view and overlay toggles are local
view controls. Save view writes a screenshot. **Export destination** writes only
that single checked posture and refuses to overwrite an existing file. Nothing
is automatically installed into `destinations/right` or merged into plan_drums.
The center marker remains a visual overlay. Its already fitted center coordinate
now also defines the inward aiming reference; neither dot is moved. The tip still
targets green, and neither marker is added to collision geometry.

An infeasible target shows the captured start and an explanation, **not** a
compromised goal. Export is disabled. Missing/invalid target depth stops before
IK. Input hashes bind reopened previews to their scene, state, target, calibration
and model; changed files are rejected. Retry acquisition by launching a new run.

Version 8 stores the per-joint reserve, J2 operating policy, both orientation conventions and
the ±25° requirement. Old version-1/2/3/4/5/6/7 standalone results (including failed searches) are preserved but cannot be reopened
as current constrained previews or exported. Recalculate into a new folder using their saved
pair and pose, as below. Do not edit old result labels to force acceptance.

## Explicit offline and saved-input operations

```bash
# Reopen a completed run: no devices or recalculation.
./plan_cymbal_destination.sh --open /path/to/run

# Replay a saved DT1 pair using an explicitly supplied BOTH-arm state: no devices.
./plan_cymbal_destination.sh --replay /path/to/pair.npz --state /path/to/measured_start.json

# Fresh camera but explicitly saved arm geometry: no CAN queries.
# The display labels this pose as unsynchronized; it is not fresh live evidence.
./plan_cymbal_destination.sh --state /path/to/measured_start.json

# Calculation/report only; choose a NEW output directory.
./plan_cymbal_destination.sh --headless --output /path/to/new-run
```

`--seconds N` closes the preview after N seconds. There are deliberately no
`--execute`, `--left`, timing, strike, target-shift or limit-widening options.
Exit codes: 0 = ready; 2 = no feasible destination; 1 = acquisition/setup/error;
130 = cancelled. Jobs are serialized and contained to 3 GiB RAM/128 MiB swap.
Cancel/terminal interruption cleans up only owned workers; planning/preview jobs
cannot access AF_CAN. There is no background live camera tracking or result cache.

## Current inward-aiming result

Version 8 restores the operator-approved **±25°** range after the 2026-10-28
startup failure. The saved failing target needed more than 20° of inward yaw
with the existing joint reserves. This is not a motor-limit increase: the exact
green dot, J2 goal minimum +1°, other 3° reserves, above-parallel/J7 alignment,
full collision checks and timing limits are unchanged.

The optimizer, final validation, preview and new result/cache identities share
this same bound. Version 4–7 missions remain readable under their original
20° metadata for existing recovery; new destination generation uses version 8.
Old outputs are preserved, never relabeled as newly calculated destinations.

Verification: `diagnostics/dt1_ik_recovery_20261028/APPROVED_REPORT.md` and
`cymbal_destination/VERIFICATION.md`.

The earlier above-parallel sign correction remains in place. R1 still classifies
as above parallel; it is an angle reference, not an exact generated green-dot
solution. Historical pre-aiming results are preserved in the dated diagnostics,
not relabeled as satisfying the new rule.

Single-frame depth can contain noise and holes; the fitted disc is inferred.
Unseen space, absolute camera/model accuracy, dynamic reachability, joint effort,
strike clearance and physical execution are not certified by this destination
preview. A later integration must reacquire/recheck the real execution context.

Verification details: [cymbal_destination/VERIFICATION.md](cymbal_destination/VERIFICATION.md).
