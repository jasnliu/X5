#!/usr/bin/env python3
"""Rebuild hardware result tables from immutable scores (no control imports/I/O).

Example: python3 diagnostics/strike_lab_hardware/build_report.py \
  experiment_results/hardware_final/SESSION --output diagnostics/strike_lab_hardware/final
Uses every attempt, including failed strokes. Never modifies source measurements.
"""
import argparse
import collections
import csv
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
MODES = ('powered', 'impedance', 'optimized', 'gravity', 'variable_damping',
         'hybrid', 'cosine', 'torque', 'baseline')
FIELDS = ('lower_zone_ms', 'peak_depth_deg', 'cycle_ms', 'upper_overshoot_deg',
          'entry_speed_rad_s', 'exit_speed_rad_s', 'timing_bracket_ms',
          'command_limited_fraction', 'release_lateness_ms', 'peak_estimated_torque',
          'rms_estimated_torque', 'maximum_temperature_c')


def percentile(values, p):
    x = sorted(values)
    index = (len(x)-1)*p
    a = int(index)
    return x[a]+(x[min(a+1, len(x)-1)]-x[a])*(index-a)


def load(path):
    session = json.loads((path/'session.json').read_text())
    assert session['backend'] == 'hardware', path
    trials = []
    for f in sorted(path.glob('*/metadata.json')):
        meta = json.loads(f.read_text())
        result = f.parent/'score.json'
        s = (json.loads(result.read_text()) if result.exists()
             else {'passed': False, 'reasons': ['incomplete_trial']})
        trials.append(dict(meta, session=str(path.relative_to(ROOT)), id=f.parent.name,
                           source_id=session['source_id'], score=s))
    return session, trials


def summarize(trials):
    result = dict(attempts=len(trials), passed=sum(t['score']['passed'] for t in trials),
                  failure_reasons=dict(collections.Counter(
                      r for t in trials for r in t['score']['reasons'])))
    for key in FIELDS:
        values = [t['score'][key] for t in trials if t['score'].get(key) is not None]
        result[key] = (dict(n=len(values), median=statistics.median(values),
                            p95=percentile(values, .95), minimum=min(values),
                            maximum=max(values), std=statistics.pstdev(values)) if values else None)
    result['by_stage'] = {
        stage: dict(attempts=sum(t['stage'] == stage for t in trials),
                    passed=sum(t['stage'] == stage and t['score']['passed'] for t in trials))
        for stage in sorted({t['stage'] for t in trials})}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sessions', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--plot', action='store_true')
    args = p.parse_args()
    selected = []
    session_data = {}
    for path in args.sessions:
        path = path.resolve()
        session, trials = load(path)
        selected.extend(trials)
        session_data[str(path.relative_to(ROOT))] = session
    # Pool validation/endurance only for the 200-trial headline. Keep any extra
    # cadence probes separate and preserve the per-stage counts in each group.
    groups = collections.defaultdict(list)
    for t in selected:
        cohort = 'comparison' if t['stage'] in ('validate', 'endurance') else t['stage']
        groups[t['session'], t['method'], t['parameters_id'], cohort].append(t)
    summaries = []
    for (session, method, pid, cohort), trials in groups.items():
        summaries.append(dict(session=session, method=method, parameters_id=pid, cohort=cohort,
                              parameters=trials[0]['parameters'], **summarize(trials)))
    summaries.sort(key=lambda s: (s['session'], MODES.index(s['method'])))
    history = []
    all_trials = []
    for f in sorted((ROOT/'experiment_results').glob('hardware_*/*/session.json')):
        session, trials = load(f.parent)
        all_trials.extend(trials)
        history.append(dict(session=str(f.parent.relative_to(ROOT)), source_id=session['source_id'],
                            **summarize(trials), modes={m:summarize([t for t in trials if t['method']==m])
                                for m in MODES if any(t['method']==m for t in trials)}))
    out = args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    variants = collections.defaultdict(list)
    for t in all_trials:
        variants[t['method'], t['source_id'], t['parameters_id']].append(t)
    out.with_suffix('.json').write_text(json.dumps(dict(
        note='Every attempt included; independent session/profile groups; no rescoring.',
        sessions=session_data, groups=summaries, history=history,
        exploration_variants=[dict(method=k[0], source_id=k[1], parameters_id=k[2],
            parameters=v[0]['parameters'], sessions=sorted({t['session'] for t in v}),
            attempts=len(v), passed=sum(t['score']['passed'] for t in v))
            for k,v in sorted(variants.items())]), indent=2)+'\n')
    with out.with_name(out.name+'_all_attempts.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['session', 'id', 'source_id', 'method', 'stage',
            'parameters_id', 'parameters', 'passed', 'reasons', *FIELDS])
        w.writeheader()
        for t in all_trials:
            row = {k:t[k] for k in ('session', 'id', 'source_id', 'method', 'stage', 'parameters_id')}
            row.update(parameters=json.dumps(t['parameters'], sort_keys=True),
                       passed=t['score']['passed'], reasons=json.dumps(t['score']['reasons']))
            row.update({k:t['score'].get(k) for k in FIELDS})
            w.writerow(row)
    rows = []
    for s in summaries:
        flat = {k:s[k] for k in ('session', 'method', 'parameters_id', 'cohort', 'attempts', 'passed')}
        flat['failure_reasons'] = json.dumps(s['failure_reasons'], sort_keys=True)
        for key in FIELDS:
            if s[key]:flat.update({key+'_'+k:v for k,v in s[key].items()})
        rows.append(flat)
    columns = list(dict.fromkeys(k for r in rows for k in r))
    with out.with_suffix('.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columns); w.writeheader(); w.writerows(rows)
    lines = ['# Hardware measurement tables', '',
             'All measured attempts included, not just passing strokes. Durations in milliseconds.', '',
             '| Mode | Pass / attempts | Depth min–max ° | Zone median / P95 ms | Cycle median ms | Upper overshoot max ° | Failures |',
             '|---|---:|---:|---:|---:|---:|---|']
    for s in summaries:
        d,z,c,u = (s[k] for k in ('peak_depth_deg','lower_zone_ms','cycle_ms','upper_overshoot_deg'))
        label = s['method']+(' ('+s['cohort']+')' if s['cohort']!='comparison' else '')
        lines.append(f"| {label} | {s['passed']}/{s['attempts']} | {d['minimum']:.3f}–{d['maximum']:.3f} | "
                     f"{z['median']:.2f} / {z['p95']:.2f} | {c['median']:.1f} | {u['maximum']:.3f} | "
                     + ', '.join(f'{k}: {v}' for k,v in s['failure_reasons'].items())+' |')
    lines += ['', '## Session history', '', '| Session | Attempts | Pass |', '|---|---:|---:|']
    for h in history:lines.append(f"| {h['session']} | {h['attempts']} | {h['passed']} |")
    out.with_suffix('.md').write_text('\n'.join(lines)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10,5))
        for s in summaries:
            if s['cohort']!='comparison' and any(g['cohort']=='comparison' for g in summaries):continue
            batch = groups[s['session'],s['method'],s['parameters_id'],s['cohort']]
            candidates = [t for t in batch if 'lower_zone_ms' in t['score']]
            chosen = min(candidates, key=lambda t:abs(t['score']['lower_zone_ms']-s['lower_zone_ms']['median']))
            with (ROOT/chosen['session']/chosen['id']/'trace.csv').open() as f:
                trace = list(csv.DictReader(f))
            ax.plot([float(r['sample_at'])*1000 for r in trace],
                    [float(r['depth_deg']) for r in trace], label=s['method'], linewidth=1.3)
        ax.axhspan(9, 11, color='#ffe0e0', zorder=0)
        ax.axhline(10, color='black', linestyle='--', linewidth=.8, label='10° target')
        ax.axhline(9, color='#ad3333', linestyle=':', linewidth=.8)
        ax.invert_yaxis();ax.set_ylim(10.5, -.3)
        ax.set_xlabel('Time from release (ms)');ax.set_ylabel('Downward J7 displacement (degrees)')
        ax.set_title('Real encoder traces: median-duration representative of each profile')
        ax.legend(ncol=2, fontsize=8, loc='upper right');ax.grid(alpha=.2)
        fig.tight_layout();fig.savefig(out.with_suffix('.png'), dpi=150);plt.close(fig)
    print(out.with_suffix('.md'))


if __name__ == '__main__':
    main()
