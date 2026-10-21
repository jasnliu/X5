# Efficient Git updates for X5

This directory is now a permanent checkout of `jasnliu/X5`, on `main`, with
`origin/main` as its upstream. Do not create another temporary full clone to
publish ordinary changes.

```bash
cd /home/jason/Proyectos3/X5
git status --short
python3 scripts/git_artifacts.py --check-index
git add <files-you-intend-to-publish>
git commit -m "Describe the change"
git push
```

Review the staged paths with `git diff --cached --name-status` before committing.
No Git command above moves the robot or opens the camera. Normal pushes transfer
new Git objects rather than re-uploading the entire workspace.

## What stays in Git

- Application code, launchers, tests, documentation and vendor source.
- Robot/gripper geometry, zones, existing recordings and the labeled `X5data` dataset.
- Saved calibration parameters and collected calibration images/session data.
- The exact default experiment resting pose, frozen camera scene and all of its
  hash-bound depth files.
- Saved plans/models needed by offline regression tests and the baseline comparison.
- Selected human-readable reports, small linked attachments, and final study summaries/plots.

The explicit list of retained files **inside output directories** is
[`config/git_retained_artifacts.json`](config/git_retained_artifacts.json).
The rest of the source tree is not restricted to this list.

## What stays local

Routine generated files under `diagnostics/`, `planner_runs/`, `playback_results/`,
`experiment_results/`, `calibration_sessions/`, `output/` and `tmp/` are ignored
unless explicitly retained. This includes repeated candidate pools, raw CAN
telemetry, videos, intermediate checks, source snapshots and routine run logs.
Build/install/log output, Python environments and caches are also ignored.

**Stopping Git tracking does not delete these local files.** This cleanup did not
run `git clean`, reset the working tree, alter a trajectory, or rewrite history.
Never use `git clean -fdx` here: it would delete ignored local captures and runs.
Ignored files are not backed up by future Git pushes; use a separate backup for
valuable new capture data.

The pre-cleanup snapshot remains in history at
[`756e8af7`](https://github.com/jasnliu/X5/tree/756e8af725628bcaac14cf11042e28812af4fb8e).
It contains the previously uploaded raw study runs and source snapshots. A retained
report may refer to that archived raw evidence. The nine oversized files omitted
from that original upload exist only locally unless separately backed up.

## Promoting a new saved input or report

1. Choose only the required files. For a camera scene include `scene.json` and
   **every file in `files_sha256`**, plus any preview assets needed by the UI.
   A saved plan also needs its request and referenced model/scene/state files.
2. Add each path and a reason to `config/git_retained_artifacts.json`.
3. Run `python3 scripts/git_artifacts.py --write-ignore`.
4. Stage the selected files, the manifest and `.gitignore`, then run
   `python3 scripts/git_artifacts.py --check-index` and the relevant tests.

New files next to an allowed input remain ignored; allowing one fixture does not
allow an entire run folder. The script updates only its marked `.gitignore` block;
it never deletes files, stages changes, or accesses hardware.

## Historical data and partial clone

This checkout retains the full commit history metadata, but was initialized with
Git's `blob:none` partial-clone filter. Current retained file contents are cached
locally. Reading old, uncached historical file contents may download them on demand.
This avoids fetching gigabytes of old logs just to update code.

For another machine, use:

```bash
git clone --filter=blob:none git@github.com:jasnliu/X5.git
```

Old large blobs still exist in GitHub history; a conventional full clone can remain
large. Removing those historical blobs would be a separate history-rewriting task,
not part of this cleanup. Existing saved robot/scene records retain their original
absolute paths and hashes; this Git change does not make the application relocatable.

References: [Git partial clone](https://git-scm.com/docs/partial-clone),
[Git ignore semantics](https://git-scm.com/docs/gitignore).
