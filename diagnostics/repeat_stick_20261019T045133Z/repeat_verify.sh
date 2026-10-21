#!/bin/bash
set -eu
cd /home/jason/Proyectos3/X5
D=$(cat /tmp/x5_repeat_stick_diag)
for i in 02 03 04 05; do
 echo "LIVE RUN $i START $(date -u +%FT%TZ)"
 ./plan_drums.sh live --headless --output "$D/verified_live_$i" > "$D/verified_live_$i.log" 2>&1
 tail -3 "$D/verified_live_$i.log"
 /usr/bin/python3 - "$D/verified_live_$i/plan/plan.json" <<'PY'
import sys,json
p=json.load(open(sys.argv[1]));assert p['status']=='planned',p.get('details');assert p['actual_current_robot_state_verified'];print('PASS:',p['status'])
PY
done
