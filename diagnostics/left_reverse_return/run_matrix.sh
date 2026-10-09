#!/bin/bash
set -eo pipefail
ROOT=/home/jason/Proyectos3/X5
D="$ROOT/diagnostics/left_reverse_return"
source /opt/ros/jazzy/setup.bash
mkdir -p "$D/cache" "$D/ros_logs" "$D/matrix"
for case in 'mit normal record3' 'csp normal record3' 'mit feedback_fault record3' 'csp feedback_fault record3' 'mit send_fault record3' 'csp send_fault record3' 'mit jitter record3' 'csp jitter record3' 'mit post_strike record3' 'mit deadline_fault record3' 'mit normal record1' 'mit normal record2'; do
 read -r mode scenario record <<< "$case"
 for version in current reference; do
  dir="$ROOT"; [ "$version" = current ] || dir=/home/jason/Proyectos3/X5_beatTest
  out="$D/matrix/${version}_${mode}_${scenario}_${record}"
  echo "RUN $version $case"
  bwrap --die-with-parent --unshare-net --ro-bind / / --dev /dev --proc /proc \
   --bind "$D" "$D" --bind /tmp /tmp --chdir "$dir" \
   --setenv XDG_CACHE_HOME "$D/cache" --setenv ROS_LOG_DIR "$D/ros_logs" \
   --setenv PYTHONDONTWRITEBYTECODE 1 /usr/bin/python3 "$D/trace_return.py" \
   "$mode" "$out.json" "$scenario" "$record.json" > "$out.txt" 2>&1
  tail -1 "$out.txt"
 done
 /usr/bin/python3 - "$D/matrix" "${mode}_${scenario}_${record}" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); name=sys.argv[2]
a=json.loads((p/('current_'+name+'.json')).read_text());b=json.loads((p/('reference_'+name+'.json')).read_text())
assert a['completed'] and b['completed'], ('incomplete',name,a['final_status'],b['final_status'])
for key in ('right_start','movements','positions','right_disabled_after_s'):
 assert a[key]==b[key], (name,key,'DIFFERENCE')
print('EXACT MATCH:',name, len(a['positions']), 'poses',len(a['movements']),'frames')
PY
done
