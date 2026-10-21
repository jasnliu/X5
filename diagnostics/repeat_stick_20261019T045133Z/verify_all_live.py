import json,sys,pathlib,xml.etree.ElementTree as ET
import numpy as np
ROOT=pathlib.Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import common,planning,workflow
from drum_planner.execution import Curve
from drum_planner.read_only import ALLOWED
D=pathlib.Path(__file__).parent;rows=[]
for folder in sorted(D.glob('verified_live_*')):
 if not folder.is_dir():continue
 p=json.load(open(folder/'plan/plan.json'));r=json.load(open(folder/'plan/backend_request.json'));report=json.load(open(folder/'current_plan_report.json'));state=json.load(open(folder/'measured_start.json'))
 assert report['passed'] and report['error'] is None and p['status']=='planned'
 assert p['actual_current_robot_state_verified'] and report['telemetry']['error'] is None
 assert not p['motion_authorized'] and not p['hardware_execution_available']
 assert state['right_rad']==r['start']==p['outward'][0]['positions_rad']
 assert p['cycle'][-1]['positions_rad']==r['start']
 assert p['outward'][-1]['positions_rad']==common.endpoint().tolist()
 assert p['hold_at_goal_s']==2 and p['reverse_checked_on_same_static_scene']
 assert workflow.simulate(p)['motor_commands']==0
 assert p['joint_limit_check']['valid'] and not p['joint_limit_check']['angles_clamped']
 generated=ET.parse(r['urdf']).getroot();curve=Curve(p)
 for name,pp in zip(common.JOINTS,curve.curves):
  limits=generated.find(f"joint[@name='{name}']/limit");lo=float(limits.get('lower'));hi=float(limits.get('upper'))
  roots=pp.derivative().roots(extrapolate=False);roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])];values=pp(np.r_[pp.x,roots]);assert values.min()>=lo-1e-9 and values.max()<=hi+1e-9
  for order,bound in [(1,.6),(2,1.2),(3,6.)]:assert planning.extrema(pp,order)/curve.scale**order<=bound
 assert p['calibration_sha256']==common.digest(common.DEFAULT_CALIBRATION)
 assert p['recording_sha256']==common.digest(common.DEFAULT_RECORDING)
 packets=0
 for line in (folder/'can_query_audit.jsonl').read_text().splitlines():
  row=json.loads(line);assert row['operation']=='state_request_only' and bytes.fromhex(row['packet_hex']) in ALLOWED
  if row['event']=='sent':packets+=1
 for line in (folder/'encoder_batches.jsonl').read_text().splitlines():
  row=json.loads(line);assert row['all_16_motors_disabled']
  assert all(x['mode']==0 and x['fault_bits']==0 for arm in row['feedback'].values() for x in arm)
 sc=r['scene'];assert len(sc['frames'])==5 and sc['camera']['depth']['width']==1280 and sc['camera']['depth']['height']==800
 assert sc['depth_fusion']['minimum_support']==3
 assert sc['obstacle_padding_m']==.02
 assert p['robot_self_filter']['drumstick_padding_m']==.029
 assert not p['robot_self_filter']['collision_geometry_changed']
 assert p['optimization']['searched_candidates']==6
 rows.append(dict(run=folder.name,passed=True,candidate_count=6,outward_s=p['duration_outward_s'],cycle_s=p['cycle'][-1]['time_s'],query_packets=packets,continuous_position_velocity_acceleration_jerk_extrema_verified=True,motor_commands=0))
assert len(rows)==5
out=dict(passed=True,consecutive_live_planning_runs=5,runs=rows,total_query_packets=sum(x['query_packets'] for x in rows),all_16_motors_disabled_and_fault_free=True,motor_commands=0,mode_changes=0,gripper_commands=0,relax_commands=0,physical_motion_tested=False)
(D/'live_verification.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
