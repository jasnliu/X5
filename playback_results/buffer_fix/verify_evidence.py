"""Offline audit of retained physical evidence. Does not import motor code."""
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
BASE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


recording = read(ROOT/'recordings/record3.json')
source_hash = digest(ROOT/'recordings/record3.json')
assert source_hash == '138c4cd724f33135b33a1ba96c40a4b2a412ba195d47a02d55f5ea8a964afa7e'
report = dict(verified_at_utc=datetime.now(timezone.utc).isoformat(),
              offline_evidence_audit=True, physical_runs=[])

for test, mode, run in [
    (3, 'hardwaretest', '20261003T191158-1011e719'),
    (5, 'hardware', '20261003T191653-75d52cf5'),
]:
    directory = ROOT/'playback_results/start_beat_recording_only'/run
    result = read(directory/'result.json')
    assert result['success'] and result['playback_success'] and result['relaxed_verified']
    worker_dir = Path(result['worker_directory'])
    worker = read(worker_dir/'result.json')
    assert worker['success'] and worker['gains_restored']
    assert worker['error'] is None and worker['cleanup_error'] is None
    assert worker['endpoint']['goal_rad'] == recording['samples'][-1]['positions_rad']
    assert worker['endpoint']['max_error_deg'] <= .05
    assert worker['method']['duration_s'] == 4.8
    assert worker['method']['source_sha256'] == source_hash
    trace = list(csv.DictReader((worker_dir/'trace.csv').open()))
    playback = [row for row in trace if row['phase'] == 'PLAYBACK']
    assert float(playback[-1]['trajectory_time_s']) == 4.8
    assert [float(playback[-1][f'target{i}']) for i in range(1, 8)] == recording['samples'][-1]['positions_rad']
    times = [float(row['monotonic_s']) for row in playback]
    intervals = [b-a for a, b in zip(times, times[1:])]
    elapsed = times[-1]-times[0]
    assert 4.75 <= elapsed <= 4.9
    commands = rows(worker_dir/'commands.jsonl')
    assert all(row['kind'] in (17, 18) and row['side'] == 'right'
               and 1 <= row['motor'] <= 7 for row in commands)
    # No enable/disable/MIT packet or gripper/left-arm command from playback.
    disables = rows(directory/'disable_audit.jsonl')
    assert Counter(row['motor'] for row in disables) == Counter({i: 3 for i in range(1, 9)})
    assert all(row['max_error_deg'] <= .2 and row['span_deg'] <= .12
               and row['settled_s'] >= .6
               and row['t'] > result['center']['t'] > worker['endpoint']['t']
               for row in disables)
    phases = [row['phase'] for row in rows(directory/'phases.jsonl')]
    assert phases[-1] == 'RELAXED' and 'RECORDING PLAYBACK' in phases
    assert not any('ALIGN' in phase or 'STRIK' in phase for phase in phases)
    log = (BASE/f'test{test}_{mode}.log').read_text()
    assert all(error not in log for error in ('No buffer space', 'FAULT:', 'NOT RELAXED', 'Traceback', '[ERROR]'))
    assert log.count('process has finished cleanly') == 5
    assert read(BASE/f'ntfy_test{test}.json')['accepted']
    report['physical_runs'].append(dict(
        test=test, mode=mode, command=f'./start_beat.sh --{mode} --recording recordings/record3.json --recording-only --auto-run',
        directory=str(directory.relative_to(ROOT)), worker_directory=str(worker_dir.relative_to(ROOT)),
        source_duration_s=recording['duration_s'], playback_duration_s=4.8,
        trace_wall_duration_s=elapsed, playback_target_sets=len(playback),
        effective_target_hz=(len(playback)-1)/elapsed,
        median_target_interval_ms=1000*statistics.median(intervals),
        max_target_interval_ms=1000*max(intervals),
        endpoint_max_error_deg=worker['endpoint']['max_error_deg'],
        disable_max_center_error_deg=max(row['max_error_deg'] for row in disables),
        disable_center_settled_s=min(row['settled_s'] for row in disables),
        disable_packets_audited=len(disables), gains_restored=True,
        relaxed_verified=True, no_alignment_or_strike=True,
        worker_transport=worker['transport'], gui_transport=read(directory/'transport.json')))

post = read(BASE/'postflight/result.json')
assert post['query_only'] and post['all_16_relaxed']
assert len(post['states']) == 16 and all(state['mode'] == 0 for state in post['states'].values())
assert list(post['position_gains'].values()) == [80., 80., 60., 60., 30., 30., 30.]
for interface in read(BASE/'postflight/can.json'):
    info = interface['linkinfo']['info_data']
    assert info['state'] == 'ERROR-ACTIVE' and info['berr_counter'] == dict(tx=0, rx=0)
    assert info['bittiming']['bitrate'] == 1000000
report['postflight'] = dict(all_16_relaxed=True, original_gains_verified=True,
                           both_can_error_active=True, current_tx_rx_error_counters_zero=True)

before = read(BASE/'hashes_before.json')
protected = ['start_beat.sh', 'centering/motors.py', 'camera_playback/smooth_recording.py',
             'recordings/record1.json', 'recordings/record2.json', 'recordings/record3.json']
baseline = read(ROOT/'playback_results/start_beat_integration/hashes_before.json')
protected += [p for p in baseline if p.startswith('smooth_playback/') and p.endswith('.py')]
protected += ['playback.sh']
for path in protected:
    assert digest(ROOT/path) == (before[path] if path in before else baseline[path]), path
report['unchanged_protected_files'] = protected
tested = read(BASE/'test5_source/hashes.json')
for path, expected in tested.items():
    assert digest(ROOT/path) == expected, f'Changed since physical test: {path}'
report['current_production_hashes_match_final_physical_test'] = tested
regression = (BASE/'regression_verified.log').read_text()
assert 'Ran 385 tests' in regression and regression.rstrip().endswith('OK')
ui_checks = ['ui_hardware.log', 'ui_hardwaretest.log',
             'ui_recording_only_hardware.log', 'ui_recording_only_hardwaretest.log']
for path in ui_checks:
    assert 'PASS: real Tk' in (BASE/path).read_text()
report['offline_tests'] = dict(regressions_passed=385, ui_checks_passed=4)
(BASE/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
