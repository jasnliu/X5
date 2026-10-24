"""Validate and summarize the complete cold-session wall-clock comparison."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
load=lambda p:json.loads(Path(p).read_text())
summary=load(D/'comparison.json');protocol=load(D/'protocol.json')
versions={b:load(D/b/'summary.json') for b in ('gpu','cpu')}

def stamp(t):
 m,s=divmod(t,60);return f'{int(m)}m {s:04.1f}s'

def request(folder):
 r=load(folder/'backend_request.json')
 for key in ('urdf','srdf'):r[key]=hashlib.sha256(Path(r[key]).read_bytes()).hexdigest()
 return r

validation=[]
for method in protocol['method_order']:
 g=next(r for r in versions['gpu']['methods'] if r['method']==method)
 c=next(r for r in versions['cpu']['methods'] if r['method']==method)
 if g['status']!='planned' or c['status']!='planned':
  validation.append(dict(method=method,complete=False,gpu_status=g['status'],cpu_status=c['status']));continue
 for leg in ('checkpoint_1','checkpoint_2','return_to_start'):
  gd=D/'gpu/plans'/method/leg;cd=D/'cpu/plans'/method/leg
  gp,cp=load(gd/'plan.json'),load(cd/'plan.json')
  gm=gp['experiment']['retiming'];cm=cp['experiment']['retiming']
  assert request(gd)==request(cd),(method,leg,'request inputs differ')
  assert gm.get('actual_backend') in ('vulkan','analytic_cpu'),(method,leg,'unexpected GPU fallback')
  assert 'gpu_used' not in cm and cm['method']=='adaptive',(method,leg,'not original CPU')
  same=gp['path_waypoints']==cp['path_waypoints'] and gp['source_timing']==cp['source_timing']
  ratio=gp['duration_outward_s']/cp['duration_outward_s']
  validation.append(dict(method=method,leg=leg,complete=True,identical_inputs=True,
      same_source_curve_and_clock=same,same_selection=gp['experiment']['report']['selected']==cp['experiment']['report']['selected'],
      movement_ratio=ratio,gpu_backend=gm.get('actual_backend'),gpu_math_cached=gm.get('cached'),
      cpu_reference_cached=gm.get('cpu_reference_cached'),cpu_cached=cm.get('cached'),
      gpu_reference_s=gm.get('cpu_reference_elapsed_s'),gpu_phase_s=gm.get('gpu_phase_elapsed_s'),
      gpu_solver_s=gp['timing'].get('details',{}).get('calculation_seconds'),
      cpu_retiming_s=cm.get('elapsed_s')))
  if same:assert ratio<=1.01,(method,leg,ratio)
(D/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
lines=['R1 + R4: full five-method cold-session comparison','================================================',
 'Route: saved rest -> R1 -> R4 -> independently planned saved rest.',
 'Both tests calculate all five methods, in identical default GUI order.',
 'Original CPU worker is byte-identical to the pre-GPU snapshot; CPU timing,',
 'path generators, limits, scene, endpoints, seed and settings are unchanged.',
 'No current program was edited. No CAN, camera, encoder or motor access.', '',
 'Version                         All-five wall time',
 f'Current GPU version             {stamp(summary["totals_s"]["gpu"])} ({summary["totals_s"]["gpu"]:.3f} s)',
 f'Original CPU-only version       {stamp(summary["totals_s"]["cpu"])} ({summary["totals_s"]["cpu"]:.3f} s)',
 f'GPU minus CPU: {summary["difference_s"]:+.3f} s ({summary["percent_difference"]:+.2f}%).','',
 'Method                   GPU version     CPU-only       Difference']
for r in summary['methods']:
 lines.append(f'{r["method"]:23s}{stamp(r["gpu_s"]):>13s}{stamp(r["cpu_s"]):>15s}{r["difference_s"]:12.2f} s')
lines+=['','Timing interpretation:',
 '- Wall-clock calculation time, NOT movement duration. Includes path generation,',
 '  selection, CPU/GPU timing, verification, checkpoint composition and worker startup.',
 '- Both versions begin with empty isolated run-local and persistent caches; no',
 '  pre-existing trajectory or CPU reference results are available. Normal reuse',
 '  BETWEEN METHODS WITHIN that same session remains enabled, as in the real GUI.',
 '  Thus later rows can be fast; these are not five independently cold programs.',
 '- Current GPU version includes its full CPU-reference qualification on every new',
 '  curve. That is why a fast GPU solver alone does not imply a faster cold total.',
 '- GUI rendering/startup and frozen-scene preparation before destination selection',
 '  are excluded. Already-installed native/GPU binaries are not recompiled; OS/driver',
 '  file/shader caches are not purged. No physical playback is timed.',
 '- Same 120 s search budget per leg; serial method/route searches; at most two',
 '  timing children under the same 3 GiB job cap. GPU solves share one slot.',
 '- One complete run per version, GPU first then CPU. This is a measured comparison,',
 '  not a statistical average or guarantee for every endpoint/system load.', '',
 f'Completion status: {summary["status"]}',
 f'Validated leg comparisons: {len(validation)}',
 f'All requests identical: {all(r.get("identical_inputs",False) for r in validation)}',
 f'All selected curves/clocks identical: {all(r.get("same_source_curve_and_clock",False) for r in validation)}',
 f'All selections identical: {all(r.get("same_selection",False) for r in validation)}',
 f'Production source unchanged: {summary["existing_source_unchanged"]}',
 f'Frozen input hashes unchanged: {summary["input_hashes_unchanged"]}', '',
 'Evidence: comparison.json, protocol.json, source_before.json, validation.json,',
 'gpu/summary.json, cpu/summary.json and each plans/METHOD/worker.log + job.json.',
 'CPU executable copy: cpu_worker_before_gpu.py, loaded only by cpu_entry.py.',
 'Reproduction: run_comparison.py in a NEW diagnostic folder, with the sourced ROS',
 'environment and OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1. It refuses existing',
 'protocol/caches rather than silently turning a repeat into a warm benchmark.']
(D/'REPORT.txt').write_text('\n'.join(lines)+'\n');print('\n'.join(lines[:22]))
