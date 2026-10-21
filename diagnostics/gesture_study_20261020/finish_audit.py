"""Assemble final evidence from completed offline tests; no device operations."""
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

P = Path(__file__).resolve().parent
ROOT = P.parents[1]
sys.path.insert(0, str(ROOT))
from drum_planner import planning
from drum_planner.common import DEFAULT_CALIBRATION
from drum_planner.experiment.config import METHODS, no_can, saved_inputs

no_can()


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(name, data):
    (P / name).write_text(json.dumps(data, indent=2) + "\n")


before = read(P / "before_hashes.json")
protected = {k: v for k, v in before.items()
             if not k.startswith("drum_planner/experiment/")
             and k != "drum_planner/cli.py"}
changed = [k for k, v in protected.items() if sha(ROOT / k) != v]
assert not changed, changed
frozen = read(P / "frozen_before_holdout.json")
post_freeze = [k for k, v in frozen.items() if sha(ROOT / k) != v]
assert set(post_freeze) == {
    "drum_planner/experiment/app.py", "drum_planner/experiment/search.py"
}, post_freeze
precision = read(P / "precision_fix.json")
assert precision["all_33_generator_seed_choices_unchanged"]
assert len(precision["changes"]) == 2
state_path, scene_path = saved_inputs()
scene = planning.scene(scene_path)
inputs = read(P / "inputs.json")
original = read(inputs["baseline"])
assert original["calibration_sha256"] == sha(DEFAULT_CALIBRATION)
assert state_path == Path(inputs["state"]) and scene_path == Path(inputs["scene"])
assert before["recordings/record3.json"] == sha(ROOT / "recordings/record3.json")
preservation = dict(
    passed=True, protected_files=len(protected), changed_protected_files=changed,
    core_changed_after_holdout_freeze=["drum_planner/experiment/search.py"],
    post_freeze_changes={
        "drum_planner/experiment/search.py":
            "Eight-decimal numeric score tie handling only; all 33 generator seed "
            "choices unchanged; two selector results changed; all pools reevaluated "
            "and selected candidates reverified. See precision_fix.json.",
        "drum_planner/experiment/app.py":
            "UI only: wrap long status paths and bound method combo size hint.",
    },
    input_hashes_match={"state": True, "scene": True},
    calibration_matches_original_baseline=True,
    all_saved_scene_data_hashes_verified=True,
    final_frozen_source_hashes={k: sha(ROOT / k) for k in frozen},
)
write("preservation_audit.json", preservation)

verification = {}
trials = 0
for stage, expected in (("refine", 15), ("heldout", 18)):
    summary = read(P / f"verification_{stage}/verification.json")
    assert summary["passed"]
    results = list((P / stage).glob("*/result.json"))
    assert len(results) == expected
    trials += len(results)
    actual_selected = set()
    for path in results:
        result = read(path)
        assert all(result["selected"].get(m) for m in METHODS)
        for m in METHODS:
            actual_selected.add((result["case"]["id"], result["seed"],
                                 result["selected"][m]["index"]))
    checked_selected = set()
    for item in summary["results"]:
        assert item["passed"] and item["unchanged_production_backend"]
        assert item["original_acquisition_self_filter"]
        assert item["cycle"]["motor_commands"] == 0
        key = (item["case"], item["seed"], item["index"])
        checked_selected.add(key)
        case_dir = P / stage / f'{item["case"]}-seed{item["seed"]}'
        result = read(case_dir / "result.json")
        row = next(r for r in result["rows"] if r["index"] == item["index"])
        # The verifier can reuse previous evidence. Bind every reused proof to
        # the CURRENT candidate file, not just its numeric candidate index.
        assert item["candidate_sha256"] == sha(case_dir / row["file"])
    assert checked_selected == actual_selected
    verification[stage] = {
        "passed": True,
        "unique_trajectories": summary["unique_trajectories"],
        "samples": summary["samples"],
        "current_candidate_hashes_match_verified_proofs": True,
        "evidence": f"verification_{stage}/verification.json",
    }

regression = (P / "regression_precision_final.log").read_text()
assert "518 passed" in regression and "FAILED" not in regression
ui = read(P / "ui_test/ui_full.json")
assert ui["passed"] and not ui["renderer"]["render_errors"]
assert ui["renderer"]["qt_offscreen"] is False
assert ui["motor_commands"] == 0 and ui["can_access"] is False
played = {x["method"] for x in ui["feature_checks"]
          if x.get("played_entire_cycle")}
assert played == set(METHODS)
assert ui["renderer"]["surface_triangles"] == ui["mesh"]["triangles"] == 20184
assert all(ui["method_states"][m] == "ready" for m in METHODS)
for name in ("cli_final_job", "ui_final_job", "plot_final_job"):
    job = read(P / name / "job.json")
    assert job["exit_code"] == 0 and not job["cancelled"]
    assert job["containment"]["Result"] == "success"
    assert "AF_CAN" not in job["containment"]["RestrictAddressFamilies"]

planning_jobs = sorted((P / "refine_jobs").glob("*/job.json"))
planning_jobs += sorted((P / "heldout_jobs").glob("*/job.json"))
assert len(planning_jobs) == 33


def peak(data):
    c = data.get("containment", {})
    return max(int(c.get(k, 0)) for k in ("MemoryPeak", "observed_peak_bytes")
               if str(c.get(k, 0)).isdigit())


for path in planning_jobs:
    job = read(path)
    assert job["exit_code"] == 0 and not job["cancelled"]
    c = job["containment"]
    assert c["Result"] == "success"
    assert c["MemoryMax"] == "3221225472" and c["MemorySwapMax"] == "134217728"
    assert "AF_CAN" not in c["RestrictAddressFamilies"]
all_jobs = [(f, read(f)) for f in P.rglob("job.json")]
oom_jobs = [str(f.relative_to(P)) for f, d in all_jobs
            if "oom" in d.get("containment", {}).get("Result", "").lower()]
assert not oom_jobs, oom_jobs
active = subprocess.run(
    ["systemctl", "--user", "list-units", "--state=running,activating,deactivating",
     "--no-legend", "--no-pager", "x5-experiment-*"],
    check=True, text=True, capture_output=True).stdout.strip()
assert not active, active
for image in ("main_comparison.png", "heldout_tradeoffs.png", "ui_test/gesture_ui.png"):
    assert (P / image).is_file() and (P / image).stat().st_size > 10000

verification.update(
    passed=True, created_utc=datetime.now(timezone.utc).isoformat(),
    unit_tests_passed=518, final_trial_count=trials,
    public_methods=list(METHODS),
    independently_verified_curves=sum(verification[s]["unique_trajectories"]
                                      for s in ("refine", "heldout")),
    dense_samples=sum(verification[s]["samples"] for s in ("refine", "heldout")),
    peak_worker_mib=max(peak(read(f)) for f in planning_jobs) / 2**20,
    peak_all_logged_jobs_mib=max(peak(d) for _, d in all_jobs) / 2**20,
    memory_scope="peak_worker is final 33 study planning jobs; all_logged includes "
                 "initial compilation/setup and prototype failures, not just planning",
    oom_jobs=oom_jobs, active_experiment_services=[],
    cli={"passed": True, "evidence": "cli_final_job/job.json"},
    native_ui={"passed": True, "methods_with_full_cycle": sorted(played),
               "full_mesh_triangles": 20184, "render_errors": [],
               "evidence": "ui_test/ui_full.json"},
    live_camera_opened=False, physical_arm_access=False, motor_commands=0,
    recapture_test="Explicit offline filesystem fixture; real camera not exercised",
    preservation_audit="preservation_audit.json",
    numerical_tie_correction="precision_fix.json",
    regression_evidence="regression_precision_final.log",
    images_visually_reviewed=True,
    limits="One frozen scene and nearby tasks; no physical safety/speed claim; "
           "recording has a different start; no universal humanlike winner",
)
write("verification.json", verification)
print(json.dumps({k: verification[k] for k in (
    "passed", "unit_tests_passed", "final_trial_count", "independently_verified_curves",
    "dense_samples", "peak_worker_mib", "peak_all_logged_jobs_mib")}, indent=2))
