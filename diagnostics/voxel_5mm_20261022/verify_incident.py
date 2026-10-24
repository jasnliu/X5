"""Native 10/5 mm incident reproduction; no devices or changed collision margins."""
import copy,json,time,sys
from pathlib import Path
from drum_planner import common,depth,planning
from drum_planner.experiment.config import no_can
no_can()
D=Path(__file__).resolve().parent
F=common.ROOT/'planner_runs/20261022T203104Z-hardware-right-5e5d2b/calculate-f2ec86b8'

def main():
    out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
    t=time.monotonic();fine=depth.refine_snapshot(F/'capture/camera/scene.json',out/'scene_5mm')
    report=dict(reconstruction_wall_s=time.monotonic()-t,source=str(F),software_only=True,motor_commands=0)
    req=json.loads((F/'plan/result/checkpoint_1/backend_request.json').read_text())
    req['check_states']=[req['start'],req['goal']]
    for label,path in [('10mm',F/'capture/camera/scene.json'),('5mm',fine)]:
        r=copy.deepcopy(req);r['scene']=planning.scene(path);t=time.monotonic()
        result=planning.backend(r,out/label)
        report[label]=dict(world_padding=result['world_padding'],checks=result['checks'],point_count=r['scene']['point_count'],
                          self_filtered_voxels=result['self_filtered_voxels'],wall_s=time.monotonic()-t)
        print(label,report[label],flush=True)
    assert [x['valid'] for x in report['10mm']['checks']]==[False,True]
    assert all(x['valid'] for x in report['5mm']['checks'])
    assert report['5mm']['world_padding']['robot_padding_m']==report['10mm']['world_padding']['robot_padding_m']==.025
    common.atomic_json(out/'endpoints.json',report)

if __name__=='__main__':main()
