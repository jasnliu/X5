"""End-to-end saved-input launch, initialization and fresh R1/R4 calculation."""
import sys,time,json
from pathlib import Path
BEGAN=time.monotonic();sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment import app,checkpoints,config
from drum_planner.common import atomic_json,ROOT
folder=Path(sys.argv[1]).resolve()
original=app.command
app.command=lambda *a,**k:['/usr/bin/env','MESA_SHADER_CACHE_DISABLE=true',*original(*a,**k)]
context=app.prepare(folder,side='right')
context.update(checkpoints.freeze([ROOT/'destinations/right/destinationR1.json',ROOT/'destinations/right/destinationR4.json'],'right',folder))
atomic_json(folder/'context.json',context);prepared=time.monotonic()
target,job=app.start_plan(folder,'workspace_curve',config.settings(),60.)
app.wait(job);end=time.monotonic();result=json.loads((target/'plan.json').read_text())
atomic_json(folder/'cold_launch.json',dict(total_s=end-BEGAN,prepare_s=prepared-BEGAN,calculation_s=end-prepared,
    status=result['status'],plan=str(target/'plan.json'),shader_cache_disabled=True,result_cache_used=False))
assert result['status']=='planned'
