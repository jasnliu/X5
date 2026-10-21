import json
from pathlib import Path
D=Path(__file__).parent;rows=[]
for folder in sorted(D.glob('verified_live_*')):
 if not folder.is_dir():continue
 p=json.loads((folder/'plan/plan.json').read_text());r=json.loads((folder/'plan/planned_view_report.json').read_text());gpu=r['renderer']
 assert r['animation_completed_cycles']>=1
 assert abs(r['animation_time_s']-p['cycle'][-1]['time_s'])<1e-8
 assert {'outward','return','HOLD (2 seconds)'}.issubset(r['animation_phases_seen'])
 assert not r['can_access'] and not r['camera_access'] and not r['motion_authorized']
 assert r['same_planning_scene'] and r['scene_immutable'] and r['mesh_builds']==1
 assert gpu['render_errors']==[] and not gpu['qt_offscreen'] and gpu['surface_geometry_uploads']==1
 assert (D/f"playback_{folder.name[-2:]}.log").read_text()==''
 rows.append(dict(run=folder.name,completed_cycles=r['animation_completed_cycles'],cycle_s=r['animation_time_s'],animation_draws=r['animation_draws'],phases=r['animation_phases_seen'],render_errors=gpu['render_errors'],offscreen=False))
assert len(rows)==5
out=dict(passed=True,full_native_gui_playbacks=5,runs=rows,camera_access=False,can_access=False,physical_motion_tested=False)
(D/'playback_verification.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
