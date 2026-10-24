"""Summarize measured, saved-input evidence without conflating phase/runtime."""
import hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];D=Path(__file__).parent
load=lambda p:json.loads(Path(p).read_text())
bench=load(D/'benchmark-2/report.json');first=load(D/'benchmark-1/report.json')
assert len(bench['rows'])==8 and all(r['status']=='pass' and r['ratio']<=1.01 for r in bench['rows'])
assert all(r['duration_s']==first['rows'][i]['duration_s'] for i,r in enumerate(bench['rows']))
checkpoint=load(D/'checkpoint/verification.json');assert checkpoint['passed']
assert all(r['identical_to_saved_geometry'] for r in checkpoint['rows'])
parity=load(D/'parity-final/verification.json');assert parity['passed']
cancel=load(D/'runtime-final/cancel/verification.json');assert cancel['reaped']
for side in ('right','left'):
 assert load(D/'runtime-final'/side/'result.json')['meta']['actual_backend']=='cpu'
uis={}
for side,session in [('right','gpu-20261024-ui-right-final'),('left','gpu-20261024-ui-left')]:
 ui=load(ROOT/'planner_runs'/session/'destination_ui_verification.json')
 assert len(ui['tested_destinations'])==5 and all(r['status']=='planned' for r in ui['tested_destinations'])
 assert not ui['native_ui_errors'] and ui['cancel_resume'] and ui['startup_unselected']
 uis[side]=str(ROOT/'planner_runs'/session)
pytest=(D/'pytest-final.log').read_text();assert ' failed' not in pytest and ' passed' in pytest
summary=re.findall(r'\d+ passed[^\n]*',pytest)[-1]
audit=load(D/'audit.json');assert audit['passed']
text='''GPU-native experiment timing — verified 2026-10-24 UTC
====================================================
Scope: ./plan_drums.sh experiment --right / --left only.
All five path generators/ranking remain. Hardware and hardware-sparse retain their
original CPU implementations; no physical, encoder, camera or CAN tests occurred.

IMPLEMENTATION
FP64 projected L-BFGS inside an augmented Lagrangian; parallel line search,
constraint/gradient evaluation and optimizer state/updates run on the Radeon GPU.
32 iterations per submission; local cubic support; one shared GPU solve slot.
Python controls refinement/penalty scheduling and basis construction. Independent
continuous polynomial extrema and full scene/zone verification remain on CPU.
Not GPU-offloaded SLSQP callbacks. Not a claim of a globally optimal trajectory.
Accepted movement duration <=1.01 times the exact-curve fully refined CPU reference,
with no added detected internal stops/deep slowdowns. Original limits and approved
left C3 geometry selection are retained. Failed numerical/quality checks reject
rather than silently publish an inferior trajectory. No extra hardware authority.

IMPORTANT PERFORMANCE LIMITATION
An unseen curve still needs the old CPU reference solve once to enforce the literal
1% guarantee. This qualification runs BEFORE the GPU phase and is separately logged.
Therefore a completely cold first Calculate is not accelerated end-to-end by this
strict version: it includes the old CPU solve plus GPU work and validation.
Existing exact-input reference caches avoid repeating that qualification; GPU
math itself can also be cached. Even a cache hit gets fresh scene verification.
The original CPU implementation also benefits from caches: warm total times are
not evidence of GPU acceleration. Removing cold qualification would require a
changed guarantee or a new independently certified bound, not a hidden omission.

DEVICE / CONTROLLED BENCHMARK
'''
text+=bench['device']['identity']+'; FP64.\n'
text+='Eight exact saved curves, two runs; accepted durations identical across runs.\n'
text+='Final table uses run 2; existing compiled backend/shader cache.\n'
text+='GPU phase includes device setup, optimizer and continuous checking, NOT CPU\nreference qualification, path generation, queue waiting, GUI startup or compilation.\n'
text+='Final scene column includes dense independent collision/zone checks.\n\n'
text+='Case                 GPU phase  Scene check   CPU movement  GPU movement  Longer\n'
for r in bench['rows']:
 text+=f"{r['case']:21s}{r['gpu_and_continuous_check_s']:8.2f}s{r['final_verification_s']:11.2f}s{r['reference_duration_s']:13.6f}s{r['duration_s']:13.6f}s{100*(r['ratio']-1):8.3f}%\n"
text+='''
For R1 -> R4, the earlier controlled original-CPU study measured 173.64–176.92 s
for timing (plus about 1.09 s final scene checks), versus 13.26 s + 1.08 s here.
Source: diagnostics/vulkan_feasibility_20261023/SUMMARY.json. This is a comparison
of timing stages using DIFFERENT optimizers and a new <=1% stopping rule, not
proof that GPU hardware alone caused the entire speedup. Cold reference costs
remain as above. This earlier callback-only study is preserved unchanged.

INTEGRATION AND FAILURE CHECKS
'''
text+='- '+summary+' (fresh regression run; source/software tests, no motion).\n'
text+='- Actual Qt/VTK windows: both arms, all five methods planned; controls, unselected\n  startup, rapid changes, cancellation/retry and unavailable-route display tested.\n'
text+='- Right progress/sweep use the retained exact analytic CPU branch (no nonlinear\n  timing optimizer needed); label explicitly says analytic CPU, not GPU.\n'
text+='- Saved R1 -> R4 -> independently planned rest: three GPU laws, exact same curve\n  coefficients/endpoints as saved CPU plans, each within 1%; two timing children\n  under one 3 GiB group, GPU work serialized. Entire contained run 45.16 s with\n  cached CPU references. Preview evaluates the exact laws and returns to rest.\n'
text+='- Saved-input hardware-vs-experiment comparison passed for Workspace curve on\n  both arms: identical geometry/selection, GPU duration within 1%; no hardware UI\n  acquisition or execution was launched. Worker identity invalidates old pools.\n'
text+='- Forced actual Vulkan loader to llvmpipe-only: both arms used original CPU\n  timing and prominently logged GPU UNAVAILABLE — CPU TIMING FALLBACK; software\n  Vulkan was not falsely counted as a GPU. No global driver settings changed.\n'
text+=f"- Cancellation during real GPU iterations reaped the contained job in {cancel['cancel_elapsed_s']:.3f} s;\n  GPU was usable immediately afterward.\n"
text+='- All six generated SPIR-V shaders passed spirv-val (Vulkan 1.2). Independent\n  NumPy FP64 cost/gradient checks match real GPU optimizer snapshots.\n'
text+='- GPU cache hits still recheck current scenes; collision/quality failure tests\n  reject the result. Left C3 reconstruction tested on four retained rounded paths.\n'
text+='- Benchmark job peak observed about 248 MB; checkpoint group peak about 571 MB\n  against 3 GiB cap. These are cgroup observations, not full-system VRAM accounting.\n'
text+=f"- Preservation audit: {audit['unchanged_files']}/{audit['preexisting_files']} pre-existing files unchanged.\n"
text+='  Changed files are only explicit experiment timing wiring, timeout, relevant\n  tests/docs and artifact allowlist. All baseline/actuator code, original CPU\n  solvers, path generators, model, recordings, destinations and calibration intact.\n'
text+='''
Evidence: benchmark-2/{report.json,CASE/source.json,reference.json,law.json,proof.json};
checkpoint/verification.json; parity-final/verification.json; runtime-final/;
pytest-final.log; audit.json. Native UI reports/screenshots are at:
'''
text+='\n'.join(uis.values())+'\n'
text+='''
REPRODUCE GPU BENCHMARK (software-only, new output folder required)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \\
 .venv-planner-experiment/bin/python diagnostics/gpu_timing_20261024/run_bounded.py \\
 --task benchmark --output diagnostics/gpu_timing_20261024/repeat-new

replay_inputs/ retains byte-identical models/scene data (deduplicated against
already curated assets); request path substitutions are recorded with SHA256.
Prototype code/results are retained locally for investigation, not installed
as alternate visible modes or confused with the validated production optimizer.
'''
(D/'REPORT.txt').write_text(text)
files=list((ROOT/'drum_planner/timing/vulkan').glob('*'))+[ROOT/'drum_planner/timing/gpu_integration.py',ROOT/'drum_planner/timing/dispatch.py',ROOT/'drum_planner/experiment/worker.py',ROOT/'drum_planner/experiment/app.py']
(D/'implementation_sha256.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},indent=2)+'\n')
print(summary)
