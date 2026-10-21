# Prospective offline study protocol

Frozen before development inspection: original acquisition scene/self-filter and measured right/left pose; record3 FINAL pose only during generation. Original zone3 TCP convention and baseline collision model/margins retained. No camera acquisition or CAN access. Standalone baseline sources are protected by hashes.

Main task + five <=1.5 degree start perturbations + five <=3 degree goal perturbations. The first two of each group are development; the remaining three each are held out. The original start/goal stays the main task. Invalid endpoint perturbations are rejected before any planning; no trajectory-based cherry picking.

Eight hypotheses: baseline control, endpoint-economy reranking, partial single-joint shortcutting, endpoint corridor RRT, monotonic joint phase curves, low-frequency arch curves, constrained elastic deformation, clearance-balanced reranking. RRT candidates are generated from endpoints; no saved baseline interior is a seed. Any failure fallback is explicitly tracked and not claimed as a new successful family.

Primary descriptive simplicity measurements: extra motor travel, broad wrist/tip turning, endpoint-normalized detours, endpoint-box and positive-X goal overshoot, normalized progress disagreement. Secondary: reversal count (2deg deadband), duration, estimated clearance, compute and memory.

Recording similarity is evaluation-only, never an objective: raw joint-arc-aligned tip RMS and endpoint-chord-residual wrist/tip RMS + joint residual RMS. Residual subtraction compensates for DIFFERENT endpoints and is only a shape statistic, not a physical retargeting or collision certificate. Raw comparisons retain the large start mismatch. Diverse-goal tests assess robustness, not reproduction of a ride recording at another destination.

Eliminate obviously poor or consistently duplicated methods; retain differing trade-offs. No universal weighted quality+imitation score; no declaration of a uniquely best human trajectory. Inspect held-out results only after freezing method definitions. All selected timed trajectories must independently pass original dense production checks and endpoint/zone/limit tests. Rejecting proposals is expected, not an application fault.

Resources: one persistent FCL world per job, one job at a time, 3GiB RAM / 128MiB swap / 96 tasks cap; CPU geometry operations. No NVIDIA GPU found (Ryzen 9 7940HS + Radeon 780M); no speculative GPU dependency installation. Early-stop stalled infeasible parametric searches. Candidate payloads on disk, bounded in-memory arrays. Cancellation kills process group via systemd.
