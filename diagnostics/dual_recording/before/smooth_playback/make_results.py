"""Generate the final readable decision record from preserved physical evidence."""
import json
from pathlib import Path
import statistics
from .trajectory import ROOT


def main():
    root=ROOT/'playback_results'
    data=json.loads((root/'comparison.json').read_text())
    groups=data['methods'];winner=groups['paced_precise']
    final=json.loads((root/'final_verification.json').read_text())
    paths=json.loads((root/'measured_path_proximity.json').read_text())
    pos='ripple_median_deg';vel='velocity_ripple_median_rad_s'
    improvements={name:dict(position_percent=100*(1-winner[pos]/groups[name][pos]),
                            velocity_percent=100*(1-winner[vel]/groups[name][vel]))
                  for name in ('baseline','legacy_gui','paced_kp20')}
    selection=dict(selected_method='paced_precise',duration_s=winner['duration_s'],
                   original_duration_s=data['original_recording_duration_s'],
                   successful_runs=winner['n'],all_run_median_improvement=improvements,
                   reason='Best repeated speed-eligible balance of measured ripple and precise endpoint. Slow methods disqualified. Raw cap-10 endpoint accuracy was insufficient; original-gain endpoint restoration fixes that without slowing the movement.',
                   all_data='comparison.json',final_verification=final)
    (root/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    gui=improvements['legacy_gui'];ideal=improvements['baseline']
    cancels=[x for x in data['attempts'] if x['deliberate_cancellation_test']]
    failures=[x for x in data['attempts'] if not x['success'] and not x['deliberate_cancellation_test']]
    lines=[
        '# record3 physical playback results',
        '',
        '## Conclusion',
        '',
        '**Use `./playback.sh`. The selected default is `paced_precise`.**',
        f"The recording movement takes **{winner['duration_s']:.1f} s**, versus **{data['original_recording_duration_s']:.6f} s** in record3 (24.96% longer). The old fitted playback takes 7.09575 s; the new movement is 32.35% shorter than that. Centering and approximately two seconds of endpoint verification are separate.",
        '',
        f"Across **{winner['n']} successful physical runs**, the selected method's all-run median position ripple was **{gui['position_percent']:.1f}% lower**, and independently reported motor-velocity ripple **{gui['velocity_percent']:.1f}% lower**, than physical replay of the old GUI command timing. Against an ideal 50 Hz old-trajectory comparator, the reductions were **{ideal['position_percent']:.1f}%** and **{ideal['velocity_percent']:.1f}%**. Both comparators also have four successful physical runs.",
        '',
        f"All four selected-method runs reached the original endpoint within **{winner['max_endpoint_error_deg']:.5f} degrees** (approximately one encoder count), then returned to verified center before relaxing. Final command targets match record3 exactly; hardware accuracy is finite, not mathematically exact. Center error before relaxation was at most **{winner['max_center_error_deg']:.5f} degrees**.",
        '',
        'The choice is the best tested balance, not a claim of zero vibration or a globally optimal controller. The lower-gain raw method reduced ripple in its screening run but had a 0.154-degree endpoint error. `paced_precise` retains its 4.8-second motion, then softly restores original gains while holding the exact endpoint and verifies 0.05-degree accuracy before recentering. No relax command is sent at the endpoint.',
        '',
        '## Measurement and comparison limits',
        '',
        '- Scores use actual right-motor CAN feedback, not simulated positions or commanded-curve jerk: seven-joint vector RMS after a third-order 3–15 Hz bandpass, 100 Hz interpolation, trimming 0.25 s at each end. The velocity score uses separately reported motor velocity, not numerical differentiation.',
        '- Timestamps are host receipt times, not hardware timestamps. These are motor-side ripple proxies; there is no accelerometer and no whole-arm visual verification. The camera sees the cymbal area. Different durations change the motion spectrum, so the comparison does not isolate one cause.',
        '- `baseline` physically replays the unchanged old fitted curve at ideal 50 Hz. `legacy_gui` physically replays command timing captured from the unchanged GUI in simulation (median 32.25 ms, maximum 48.78 ms). It is **not** a full live `start_beat.sh` camera/audio workload test.',
        '- A balanced nine-cycle campaign repeated baseline, cap-20 and GUI timing three times each. Three subsequent precise-method runs established the selection; a fourth plain `./playback.sh` run verified the delivered default. The earlier progress estimate of 13%/16% used the three-run validation cohorts. The headline above uses all four successful runs of each method, including screening/final runs.',
        '- Small sample sizes and overlapping ideal-baseline position ranges limit generalization. All four selected-method velocity scores were below both comparator ranges, and all four selected-method position scores were below the GUI-timing comparator range. No formal population-level statistical claim is made.',
        '',
        '## Path, timing and safety evidence',
        '',
        '- Commanded path stayed within 1.310 degrees per joint and 3.732 mm TCP of the source path at corresponding retimed phase; exact first/last joint targets were preserved. Validated peaks: 0.693 rad/s, 1.265 rad/s², 37.964 rad/s³ piecewise jerk. These are command checks, not the measured improvement score.',
        f"- Actual feedback sampled at 50 Hz stayed within **{max(x['max_joint_difference_at_nearest_recorded_phase_deg'] for x in paths):.3f} degrees per joint / {max(x['max_tcp_difference_at_nearest_recorded_phase_mm'] for x in paths):.3f} mm TCP** of the nearest raw-recorded-path phase across the four selected runs. This permits intentional local retiming and is sampled proximity, not a continuous bound or collision certification.",
        '- Speed/current ceilings were not increased. Temporary outer position gains were capped at 10; the saved original values were [80,80,60,60,30,30,30]. A guarded 0.5-second restoration ramp runs only after soft settling near the endpoint. All readbacks and writes are logged; no flash save, zero or calibration command is used.',
        '- Ntfy HTTP acceptance and a 10-second countdown preceded every physical cycle. This verifies service acceptance, not independently observed phone delivery. Left-arm commands were prohibited; left motors stayed disabled in successful-run feedback audits.',
        f"- **{len(cancels)} deliberate physical SIGINT tests passed**: one 1.25 s into playback, one 0.1 s into the endpoint gain ramp. Both recentered, restored gains and relaxed; neither disabled at the interruption pose. Their expected `Cancelled` summaries are excluded from efficacy scores, not discarded.",
        '- Final query-only postflight verified all 16 motors relaxed, original gains restored, both CAN buses ERROR-ACTIVE with zero current TX/RX error counters, and runtime source hashes identical to the final physical run.',
        f"- All **{final['protected_existing_files']} protected existing files** remained byte-for-byte unchanged. Original record3 SHA256: `{final['source_sha256']}`.",
        '',
        '## All retained physical methods',
        '',
        'All-run medians below; a single screening run is not equivalent to repeated validation. Every method, source snapshot and raw log is retained. Slow variants and comparators cannot be selected under the final speed requirement.',
        '',
        '| Method | Successful runs | Movement s | Position ripple ° RMS | Velocity ripple rad/s RMS | Timing eligible |',
        '|---|---:|---:|---:|---:|:---:|',
    ]
    for name,g in groups.items():
        lines.append(f"| `{name}` | {g['n']} | {g['duration_s']:.3f} | {g[pos]:.6f} | {g[vel]:.6f} | {'yes' if g['timing_eligible'] else 'no'} |")
    lines += ['', '### Method definitions', '',
              '- `retimed`: original cubic with global velocity/acceleration/jerk scaling. `matched`: original cubic slowed to the smooth-path duration.',
              '- `smooth`: Gaussian filtering plus endpoint-exact quintic path. `_csp04` lowers the firmware speed ceiling; `_200hz` also increases update rate. `smooth_slow` doubles that already-slow duration.',
              '- `paced`: locally retimed smooth path at 4.6 s; `_200hz` changes update rate. `paced_soft` uses 4.8 s and joint-specific firmware ceilings; `paced_soft200` increases update rate.',
              '- `paced_damped` / `paced_damped200`: same 4.8 s path with original position gains halved at 100/200 Hz.',
              '- `paced_kp10` / `paced_kp20` / `paced_kp30`: common position-gain caps during the same 4.8 s, 200 Hz motion, restored at center.',
              '- `paced_precise`: cap-10 motion plus soft endpoint settle, guarded original-gain restoration and tight endpoint verification before return. Original gains are independently checked again at center.',
              '', '## Retained failures and checks', '',
              f"There were **{len(data['runs'])} successful full cycles**, **{len(failures)} early failed motion attempts** and **{len(cancels)} intentional cancellation tests**. Both early failures happened before enabling: camera startup made feedback stale, and passive J4 sag exceeded the strict modeled limit. These were corrected with a fresh pre-enable state batch and bounded inward-only startup recovery. No failed data was deleted.",
              'Separate query-only checks failed while the arm was powered off; the user confirmed power had been off. After power returned, state-only requests brought the old CAN error counters back to zero; no privileged interface reconfiguration was needed.',
              '',
              '- New offline suite: 24 tests passed, with physical CAN creation prohibited.',
              '- Existing repository suite: 358 tests passed. An initial stdin-launched attempt failed four multiprocessing tests because `<stdin>` could not be reopened; the real-file entrypoint fixed the test invocation, without changing existing tests.',
              '- Final delivered-command log: `playback_results/final_default_console.txt`.',
              f"- Final full cycle: `{final['final_session']}`.",
              '- Raw/aggregate evidence: `playback_results/comparison.json`, `.csv`, `.png`; `selection.json`; `final_verification.json`; `measured_path_proximity.json`; every per-run directory and cancellation verdict.',
              '- Reproduce analysis: `python3 -m smooth_playback.report`, then `python3 -m smooth_playback.path_evidence`, then `python3 -m smooth_playback.make_results` (system Python with NumPy/SciPy/Matplotlib). No hardware access is used by these commands.',
              '',
              'Position-gain register 0x701E was checked against the [manufacturer RS04 manual](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS04/RS04User%20Manual260713.pdf); the downloaded reference is retained under `playback_results/reference/`. The chosen gains are empirical local tests, not a manufacturer tuning recommendation.',
              '']
    (ROOT/'PLAYBACK_RESULTS.md').write_text('\n'.join(lines))
    print(json.dumps(selection,indent=2))


if __name__=='__main__':main()
