"""Device-free comparison against the preserved pre-optimization implementation."""
import argparse
import cProfile
import importlib
import json
import pstats
import sys
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
HERE = Path(__file__).resolve().parent
from drum_planner.experiment.config import no_can
from drum_planner.timing import adaptive, core, benchmark


def reference():
    name = 'x5_timing_reference'
    package = types.ModuleType(name)
    package.__path__ = [str(HERE / 'reference')]
    sys.modules[name] = package
    return importlib.import_module(name + '.adaptive')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--version', choices=['reference', 'current'], required=True)
    p.add_argument('--cases', default='right_clearance')
    p.add_argument('--profile', action='store_true')
    p.add_argument('--output', required=True)
    args = p.parse_args()
    no_can()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    engine = reference() if args.version == 'reference' else adaptive
    policy = json.loads((HERE / 'reference/selected.json').read_text())
    rows = {}
    for case in args.cases.split(','):
        source = json.loads((HERE / 'inputs' / (case + '.json')).read_text())
        profiler = cProfile.Profile()
        start = time.monotonic()
        if args.profile: profiler.enable()
        law = engine.optimize(source, levels=policy['levels'], ftol=policy['ftol'],
                              maxiter=policy['maxiter'], log=lambda s: print(case, s, flush=True))
        if args.profile:
            profiler.disable(); profiler.dump_stats(str(out / (case + '.prof')))
            with (out / (case + '-profile.txt')).open('w') as f:
                pstats.Stats(profiler, stream=f).strip_dirs().sort_stats('cumulative').print_stats(45)
        elapsed = time.monotonic() - start
        proof = benchmark.validate(law, source, out / case / 'validation')
        (out / (case + '-law.json')).write_text(json.dumps(law))
        rows[case] = dict(elapsed_s=elapsed, duration_s=law['duration_s'], proof=proof)
        (out / 'report.json').write_text(json.dumps(rows, indent=2))
        print('PASS', case, rows[case]['elapsed_s'], law['duration_s'], flush=True)


if __name__ == '__main__': main()
