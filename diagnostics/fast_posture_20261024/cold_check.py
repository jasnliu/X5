import json,subprocess,time
from pathlib import Path
h=Path('diagnostics/fast_posture_20261024');rows=[]
for side,targets in [('right',['R1','R2']),('left',['L1'])]:
 command=['./plan_drums.sh','--fast','--'+side,'--offline','--state','diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json','--scene','diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/camera/scene.json','--destinations',*targets,'--headless','--output',str(h/('cold_'+side))]
 t=time.monotonic()
 with (h/('cold_'+side+'.log')).open('w') as f:r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=240)
 row=dict(side=side,returncode=r.returncode,cold_process_wall_s=time.monotonic()-t,command=command,comparison_enabled=False,acquisition=False,result_cache_used=False)
 rows.append(row);(h/'cold_results.json').write_text(json.dumps(rows,indent=2)+'\n');print(row,flush=True)
