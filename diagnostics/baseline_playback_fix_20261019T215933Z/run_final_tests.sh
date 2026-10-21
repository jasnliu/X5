#!/bin/bash
set -euo pipefail
cd /home/jason/Proyectos3/X5
d=$(cat /tmp/x5_baseline_playback_task)
while ! test -f "$d/physical_03.exit"; do sleep 2; done
test "$(cat "$d/physical_03.exit")" = 0
/usr/bin/python3 "$d/audit_physical.py" "$d/physical_03" > "$d/physical_03/audit.log" 2>&1
./plan_drums.sh test > "$d/regression.log" 2>&1
echo 0 > "$d/regression.exit"
mkdir -p "$d/mock_buttons"
/usr/bin/python3 diagnostics/baseline_owner_fix_20261019T213557Z/qa_buttons.py "$d/mock_buttons" > "$d/mock_buttons.log" 2>&1
echo 0 > "$d/mock_buttons.exit"
/usr/bin/python3 "$d/qa_ui_final.py" "$d/ui_final" > "$d/ui_final.log" 2>&1
echo 0 > "$d/ui_final.exit"
