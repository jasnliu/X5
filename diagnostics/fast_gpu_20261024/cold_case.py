import argparse,json,time,sys,os
from pathlib import Path
BEGAN=time.monotonic()
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import atomic_json,digest
from drum_planner.experiment.config import settings,no_can
from drum_planner.experiment.worker import plan
no_can()
p=argparse.ArgumentParser();p.add_argument('--context',required=True);p.add_argument('--output',required=True)
p.add_argument('--cpu-only',action='store_true',help='Force real software Vulkan discovery, which must trigger explicit CPU fallback')
a=p.parse_args();folder=Path(a.output)
if folder.exists():raise ValueError('Use a fresh output directory')
context=json.loads(Path(a.context).read_text());folder.mkdir(parents=True)
for key in ('scene','state'):
 if digest(context[key])!=context[key+'_sha256']:raise ValueError('Changed input: '+key)
if a.cpu_only:os.environ['VK_DRIVER_FILES']='/usr/share/vulkan/icd.d/lvp_icd.json'
atomic_json(folder/'context.json',context)
result=plan(context,folder,'workspace_curve',settings(),60.)
atomic_json(folder/'cold.json',dict(elapsed_s=time.monotonic()-BEGAN,status=result['status'],
    shader_cache_disabled=os.environ.get('MESA_SHADER_CACHE_DISABLE'),result_cache_used=False))
if result['status']!='planned':raise SystemExit(1)
