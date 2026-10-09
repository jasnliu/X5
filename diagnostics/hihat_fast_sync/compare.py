"""Deterministic before/after acoustic simulation. No device access.

Run with the same ROS-sourced Python environment as the repository tests.
Uses the saved pre-change source, not a reimplementation of the old controller.
"""
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests')]
import test_hihat_sync as tests


def main():
    spec = importlib.util.spec_from_file_location(
        'old_hihat_sync', HERE / 'before/camera_playback/hihat_sync.py')
    old = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = old
    spec.loader.exec_module(old)
    rows = []
    scenarios = (
        ('100ms_offset', .100, 0., .3),
        ('135ms_delayed', .110, -.025, 1.1),
        ('negative_80ms', .030, .110, .7),
        ('long_delayed_batch', .070, -.030, 2.4),
    )
    for name, controller in (('before', old.HiHatSynchronizer),
                             ('after', tests.HiHatSynchronizer)):
        for label, latency, ride_shift, delay in scenarios:
            case = tests.SyncTests()
            with patch.object(tests, 'HiHatSynchronizer', controller):
                engine, matches = case.simulation(latency, ride_shift, delay)
            closes = [r for r in case.rows if r['kind'] == 'command' and r['closed']]
            first = next(r for r in closes
                         if abs(latency-ride_shift-r['advance']) <= .015)
            rows.append(dict(
                version=name, scenario=label,
                within_15ms_at_closure=closes.index(first)+1,
                seconds_after_first_close_target=round(first['target']-closes[0]['target'], 3),
                final_advance_ms=round(engine.advance*1000, 3),
                max_applied_advance_ms=round(max(abs(r['advance']) for r in closes)*1000, 3),
                matches=len(matches), skips=engine.skips,
            ))
    result = json.dumps(rows, indent=2)+'\n'
    (HERE / 'comparison.json').write_text(result)
    print(result, end='')


if __name__ == '__main__':
    main()
