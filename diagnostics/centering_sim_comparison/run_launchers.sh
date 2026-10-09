#!/bin/bash
set -eo pipefail
ROOT=/home/jason/Proyectos3/X5
D="$ROOT/diagnostics/centering_sim_comparison"
for launcher in start_beat.sh start_beatTest.sh; do
 bwrap --die-with-parent --unshare-net --ro-bind / / --dev /dev --proc /proc \
 --bind "$D" "$D" --bind /tmp /tmp --chdir "$ROOT" \
 --setenv PYTHONPATH "$D/launcher_hook" --setenv PYTHONDONTWRITEBYTECODE 1 \
 --setenv BEAT_SIM_SMOKE "$D/${launcher}.json" --setenv ROS_LOG_DIR "$D/ros_logs" \
 --setenv XDG_CACHE_HOME "$D/cache" --setenv LIBGL_ALWAYS_SOFTWARE 1 \
 /usr/bin/timeout --signal=INT --kill-after=10 200 "./$launcher" --test --recording "$ROOT/recordings/record3.json" \
 > "$D/${launcher}.log" 2>&1
 grep -E 'SIMULATION LAUNCHER|process has finished' "$D/${launcher}.log"
done
