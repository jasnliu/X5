# Development decision, frozen before held-out runs

Screened 8 initial hypotheses on 4 task/seed combinations, then 5 compact-route variants on 15 development task/seed combinations (main and 4 nearby tasks, three seeds each).

Reject strict monotonic phase sampling: no own feasible route on any initial task; it only returned the RRT fallback. Reject unrestricted-economy reranking and refinements as the main generation backbone: compared to endpoint corridors they produced substantially worse broad detours and recording-shape residual errors. Unseeded low-frequency search was too unreliable (one own valid selected route out of four), but initialization from a freshly generated compact route made that family worth retaining. This is initialization from a solver on the CURRENT task, never a recording interior.

Retain FOUR public modes: progress, sweep, single_arc, clearance. Compact corridor generation is folded into their common initialization/fallback; it is not called an inferior physical trajectory. Its separate UI preference duplicates the same objective and a strict subset of progress's available candidates. Keep sweep despite only modest median improvement: it won useful individual comparisons (e.g. goal2/509 and start1/127) and is not clearly inferior on both simplicity and recording similarity. No forced unique winner or artificial route differences.

Development medians (15 paired trials):
- progress: 30.3deg extra travel, wrist detour 1.15, tip shape residual 12.23cm, joint residual 4.07deg.
- sweep: 40.4deg, 1.16, 12.24cm, 4.54deg.
- single_arc: 38.2deg, 1.13, 12.24cm, 4.68deg.
- clearance: 48.8deg, 1.09, 12.51cm, 5.18deg; lower proximity penalty 0.11 vs about 0.19 for progress/sweep.

These are separate measures, NOT one aggregate evaluation score. Similarity to record3 is computed only after generation. Nominal timings can be slow for low-backtracking corner-stop fallbacks; preserve/report this trade-off rather than quietly discard geometrically good paths on speed alone. No global claim of human likeness or physical safety is made.

Freeze default weights, sampling/collision checks, generation and selection before 18 reserved tests (6 small held-out tasks x 3 seeds). Do not tune against these results. All 5 internal selections may be retained in analysis for the folded compact control, but ONLY the four survivors enter the public UI. Physical baseline, recordings, scene and zone3 remain unchanged.
