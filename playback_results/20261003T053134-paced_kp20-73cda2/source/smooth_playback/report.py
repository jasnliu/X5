"""Reproducible comparison and CAN/sequence audit; read existing physical logs."""
import csv
import json
from pathlib import Path
import statistics
import numpy as np
from .analyze import analyze
from .trajectory import ROOT


def main():
    root=ROOT/'playback_results';results=[];attempts=[]
    original_duration=json.loads((ROOT/'recordings/record3.json').read_text())['duration_s']
    for p in sorted(root.glob('*/summary.json')):
        summary=json.loads(p.read_text());attempts.append(dict(path=str(p.parent.relative_to(ROOT)),method=summary['method'],success=summary['success'],error=summary['error']))
        if not summary['success']:continue
        a=analyze(p.parent)
        events=[json.loads(s) for s in (p.parent/'events.jsonl').read_text().splitlines()]
        commands=[json.loads(s) for s in (p.parent/'commands.jsonl').read_text().splitlines()]
        with open(p.parent/'feedback.csv') as f:feedback=list(csv.DictReader(f))
        phases=[e['phase'] for e in events]
        disables=[c for c in commands if c['kind']==4]
        audit=dict(all_control_commands_right=all(c['side']=='right' for c in commands),
            disable_only_startup_or_verified_center=all(c['phase'] in ('ENABLE AT LIVE POSE','RELAX') for c in disables),
            endpoint_before_recenter_before_relax=phases.index('ENDPOINT VERIFIED')<phases.index('RECENTER')<phases.index('RELAX'),
            relax_after_settled_center=phases.index('RECENTER SETTLED')<phases.index('RELAX'),
            left_always_disabled=all(int(r['state'])==0 for r in feedback if r['side']=='left'),
            no_persistent_save_or_calibration=all(c['kind'] in (3,4,17,18) for c in commands),
            original_gains_restored=summary.get('original_gains_restored',True),
            max_motor_temperature_c=max(float(r['temperature_c']) for r in feedback),
            outgoing_command_count=len(commands))
        a['audit']=audit
        (p.parent/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
        if not all(v for k,v in audit.items() if isinstance(v,bool)):raise RuntimeError('Physical safety audit failed: '+str(p.parent))
        results.append(a)
    groups={}
    for name in sorted({x['method'] for x in results}):
        rows=[r for r in results if r['method']==name]
        values=[r['measured_position_ripple_3_15hz_vector_rms_deg'] for r in rows]
        speeds=[r['firmware_velocity_ripple_vector_rms_rad_s'] for r in rows]
        groups[name]=dict(n=len(rows),duration_s=rows[0]['duration_s'],ripple_median_deg=statistics.median(values),
                          ripple_min_deg=min(values),ripple_max_deg=max(values),
                          velocity_ripple_median_rad_s=statistics.median(speeds),
                          velocity_ripple_min_rad_s=min(speeds),velocity_ripple_max_rad_s=max(speeds),
                          timing_eligible=rows[0]['duration_s']<=1.25*original_duration,
                          max_endpoint_error_deg=max(r['endpoint_error_deg'] for r in rows),
                          max_center_error_deg=max(r['center_error_deg'] for r in rows))
    output=dict(attempts=attempts,methods=groups,runs=results,original_recording_duration_s=original_duration,
                timing_limit_s=1.25*original_duration,
                metric='Physical encoder position and motor-reported velocity 3-15 Hz bandpass, seven-joint vector RMS; host receipt timestamps. Not commanded-only jerk or whole-arm vibration measurement.',
                baseline_note='baseline: original fitted trajectory at ideal 50 Hz. legacy_gui: physical replay of commands timed by the unchanged GUI in simulation; no live camera/audio workload.')
    (root/'comparison.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(groups,indent=2))
    with open(root/'comparison.csv','w') as f:
        fields=['method',*next(iter(groups.values())).keys()]
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for name,g in groups.items():w.writerow(dict(method=name,**g))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names=list(groups);fig,axes=plt.subplots(1,3,figsize=(17,max(5,len(names)*.33)),layout='constrained')
    ax=axes[0]
    colors=['#4a8f75' if groups[n]['timing_eligible'] else '#a5abb3' for n in names]
    ax.barh(names,[groups[n]['ripple_median_deg'] for n in names],color=colors)
    for i,n in enumerate(names):
        for row in results:
            if row['method']==n:ax.plot(row['measured_position_ripple_3_15hz_vector_rms_deg'],i,'ko',ms=4)
    ax.set_xlabel('Position ripple RMS (degrees)');ax.set_title('Measured motor-side 3–15 Hz ripple')
    axes[1].barh(names,[groups[n]['velocity_ripple_median_rad_s'] for n in names],color=colors)
    axes[1].set_xlabel('Reported velocity ripple RMS (rad/s)');axes[1].set_title('Independent motor velocity feedback')
    axes[2].barh(names,[groups[n]['duration_s'] for n in names],color=colors)
    axes[2].axvline(original_duration,color='black',ls=':',label='Original recording')
    axes[2].axvline(original_duration*1.25,color='red',ls='--',label='25% duration margin')
    axes[2].legend();axes[2].set_xlabel('Playback duration (seconds)');axes[2].set_title('Gray methods excluded: too slow')
    fig.savefig(root/'comparison.png',dpi=150);plt.close(fig)

if __name__=='__main__':main()
