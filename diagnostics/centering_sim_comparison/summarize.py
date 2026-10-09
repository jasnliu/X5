"""Verify saved deterministic A/B traces, preservation and launcher results."""
import hashlib
import json
import math
from pathlib import Path
import re

D = Path(__file__).resolve().parent
ROOT = D.parents[1]
cases = []
for current in sorted((D/'matrix').glob('current_*.json')):
    name = current.name.removeprefix('current_')
    a = json.loads(current.read_text())
    b = json.loads((D/'matrix'/('reference_'+name)).read_text())
    assert a['completed'] and b['completed'], name
    for key in ('right_start', 'positions', 'movements', 'right_disabled_after_s'):
        assert a[key] == b[key], (name, key)
    if a['scenario'] in ('feedback_fault', 'send_fault'):
        assert a['fault_injected'] and b['fault_injected']
    assert [r['frame'] for r in a['pre_center_commands']] == [r['frame'] for r in b['pre_center_commands']]
    cases.append(dict(case=name.removesuffix('.json'),
        poses=len(a['positions']), commands=len(a['movements']),
        maximum_joint_difference_degrees=0.,
        right_disable_seconds=a['right_disabled_after_s'],
        current_pre_center_commands=a['pre_center_commands'],
        reference_pre_center_commands=b['pre_center_commands'],
        completed_both=True))
assert len(cases) == 12, len(cases)
launchers = {}
for name in ('start_beat.sh', 'start_beatTest.sh'):
    data = json.loads((D/(name+'.json')).read_text())
    assert data['completed'] and data['test_mode'] and data['bus']=='SimulatedMotors'
    assert max(abs(x-y) for x,y in zip(data['right_final'],data['center_goal'])) < math.radians(.2)
    launchers[name] = {k:v for k,v in data.items() if k!='poses'}
manifest=json.loads((D/'protected.json').read_text())
for path, digest in manifest.items():
    path=Path(path)
    if not path.is_absolute(): path=ROOT/path
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest, path
unit=(D/'full_tests.txt').read_text()
assert re.search(r'Ran 681 tests in .*\n\nOK\s*$',unit)
summary=dict(cases=cases, exact_matches=len(cases), tested_joints='right J1-J7',
    simulated_positions_compared=sum(c['poses'] for c in cases),
    center_command_frames_compared=sum(c['commands'] for c in cases),
    maximum_joint_difference_degrees=0., launchers=launchers, regression_tests=681,
    protected_files_unchanged=len(manifest), physical_hardware_used=False,
    boundary='Deterministic finite-speed simulated feedback; not physical dynamics or clearance validation.')
(D/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['# Right-arm centering comparison', '',
'## Result', '',
'**12/12 matched exactly:** right J1–J7 simulated positions, transmitted center command bytes, command timestamps relative to center dispatch, and right disable time. Both simulated arms finished and relaxed in every current-program case.', '',
'Both actual shell launchers were also run with `--test`, from the same right record3 endpoint, and exited cleanly after simulated centering/relaxation. The detailed MIT/CSP comparisons additionally run each launcher’s actual App/control/transport code with fake motor sockets, because ordinary `--test` does not exercise the hardware MIT return controller.', '',
'## Change', '',
'Only production file changed this turn: `camera_playback/app.py`, `App.fail`. Recoverable faults during a dual-arm return now reach the original right-arm fault-recentering code, rather than `DualRecording.fault`, which replaced J7’s center goal with the measured off-center angle. Left playback-fault handling and existing checks/tolerances are unchanged.', '',
'Before the fix, an injected transient return-feedback failure left J7 off-center in powered hold for the entire 25-second trial; the reference completed. After the fix, the same injected fault completed and matched the reference exactly. This demonstrates a software failure mechanism, not proof that the exact same transient occurred in the physical incident.', '',
'## Test matrix', '',
'| Case | Poses | Center commands | Right disable (s) | Max difference |',
'|---|---:|---:|---:|---:|']
for c in cases:
    lines.append(f"| {c['case']} | {c['poses']} | {c['commands']} | {c['right_disable_seconds']:.5f} | 0° |")
lines += ['', '## Scope and preservation', '',
'- Compare identical right starting poses/recordings, not the different launcher defaults.',
'- Timing is relative to the first right center command. The current program still performs extra left-arm path/freshness checks before dispatch. In the hybrid-deadline case its stationary fault-hold frames precede center dispatch by 0.316 simulated seconds; the reference dispatches immediately. Those identical hold-frame bytes and their different timestamps are retained separately in `pre_center_commands`. The centering motion itself is identical.',
'- Simulated transport executes actual controller/encoder-frame logic with finite-speed motor tracking, fixed callbacks, a 0.5-second delayed callback case, and one-shot feedback/send faults. It does not model full torque dynamics, gravity, collisions, or physical CAN timing.',
'- All processes ran without host networking/CAN interfaces and without real device nodes (`bwrap --unshare-net --dev /dev`); the reference/root filesystems were read-only. Python audit guards additionally reject CAN/device access.',
'- All 681 regression tests passed. All 309 protected reference/recording files match the pre-run SHA256 manifest. `start_beatTest.sh` and its reference runtime were not edited.',
'- No physical arm was commanded or tested. Do not interpret simulation agreement as physical clearance/safety validation.', '',
'## Reproduction', '',
'From `/home/jason/Proyectos3/X5`:', '',
'```bash',
'bash diagnostics/centering_sim_comparison/run_matrix.sh',
'bash diagnostics/centering_sim_comparison/run_launchers.sh',
'python3 diagnostics/centering_sim_comparison/summarize.py',
'```', '',
'Raw traces: `matrix/*.json`; shell logs: `start_beat*.sh.log`; full suite: `full_tests.txt`; comparison: `comparison.json`.', '']
(D/'REPORT.md').write_text('\n'.join(lines))
print('VERIFIED:',len(cases),'exact comparisons;',summary['simulated_positions_compared'],'poses;',summary['center_command_frames_compared'],'center frames; 681 tests;',len(manifest),'protected files unchanged')
