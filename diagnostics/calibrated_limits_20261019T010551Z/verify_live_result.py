import json,sys,pathlib,xml.etree.ElementTree as ET
import numpy as np
ROOT=pathlib.Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import common,planning,workflow
from drum_planner.execution import Curve
from drum_planner.read_only import ALLOWED
D=pathlib.Path(__file__).parent;folder=D/'verified_live';p=json.load(open(folder/'plan/plan.json'));r=json.load(open(folder/'plan/backend_request.json'));report=json.load(open(folder/'current_plan_report.json'));state=json.load(open(folder/'measured_start.json'))
assert report['passed'] and report['error'] is None and p['status']=='planned'
assert p['actual_current_robot_state_verified'] and report['telemetry']['error'] is None
assert not p['motion_authorized'] and not p['hardware_execution_available']
assert state['right_rad']==r['start']==p['outward'][0]['positions_rad']
assert r['start'][3]<0 and p['cycle'][-1]['positions_rad']==r['start']
assert p['outward'][-1]['positions_rad']==common.endpoint().tolist()
assert p['hold_at_goal_s']==2 and p['reverse_checked_on_same_static_scene']
assert workflow.simulate(p)['motor_commands']==0
assert p['joint_limit_check']['valid'] and not p['joint_limit_check']['angles_clamped']
source=ET.parse(ROOT/'model/openarmx.urdf').getroot();generated=ET.parse(r['urdf']).getroot();offsets=common.calibration()['kinematic_correction']['offsets_rad']
curve=Curve(p)
for name,pp in zip(common.JOINTS,curve.curves):
 orig=source.find(f"joint[@name='{name}']/limit");mapped=generated.find(f"joint[@name='{name}']/limit");lo=float(mapped.get('lower'));hi=float(mapped.get('upper'))
 assert np.isclose(lo+offsets[name],float(orig.get('lower')),rtol=0,atol=1e-12)
 assert np.isclose(hi+offsets[name],float(orig.get('upper')),rtol=0,atol=1e-12)
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
assert len(r['scene']['frames'])==5 and r['scene']['camera']['depth']['width']==1280 and r['scene']['camera']['depth']['height']==800
result=dict(passed=True,live_camera_and_encoder_capture=True,candidate_count=p['optimization']['searched_candidates'],
 selected_seed=p['optimization']['selected_seed'],outward_s=p['duration_outward_s'],cycle_s=p['cycle'][-1]['time_s'],
 raw_right_j4_rad=r['start'][3],calibrated_model_right_j4_rad=r['start'][3]+offsets['openarmx_right_joint4'],
 full_continuous_polynomial_within_mapped_limits=True,all_16_motors_remained_disabled=True,query_packets=packets,
 motor_commands=0,mode_changes=0,gripper_commands=0,relax_commands=0,physical_arm_motion_tested=False,
 self_filter=p['robot_self_filter'],physical_alignment_and_clearance_independently_verified=False)
(D/'live_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
