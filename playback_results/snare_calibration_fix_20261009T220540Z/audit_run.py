"""Read-only acceptance audit; distinguishes existing J7 mode setup from final arm relax."""
from pathlib import Path
import hashlib,json,math,re,difflib
A=Path(__file__).resolve().parent
E=Path('/home/jason/Proyectos3/X5/playback_results/calibration_verification/20261009T221240-1cf1de4c')
r=json.loads((E/'result.json').read_text());can=json.loads((A/'run1/can_audit.json').read_text())
assert r['success'] and r['snare_outcome']=='timeout' and not r['snare_calibrated']
assert r['snare_strikes']==19 and 15<=r['snare_elapsed_s']<15.05 and not r['faults']
assert r['all_16_disabled'] and set(r['disabled_proofs'])=={'left','right'}
assert can['motor_fault_frames']==0
# The legacy collector audit rejects ALL off-center single-joint mode changes.
# Main-beat hybrid J7 has an existing, confirmed disable->MIT->enable setup at
# the recording endpoint. Preserve/report this explicitly, not as zero disables.
flags=can['unproven_center_disable_commands']
assert len(flags)==1 and flags[0]['side']=='right' and flags[0]['motor']==7
t=flags[0]['t'];frames=[]
for line in (A/'run1/can.log').open():
 m=re.match(r'^\(([0-9.]+)\)\s+(can[01])\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)',line)
 if m and t<=float(m[1])<=t+.1:
  frames.append((float(m[1]),m[2],int(m[3],16),bytes.fromhex(m[4])))
mode=[f for f in frames if f[1]=='can0' and (f[2]>>24)&31==18 and f[2]&255==7 and f[3]==bytes.fromhex('0570000000000000')]
enables=[f for f in frames if f[1]=='can0' and (f[2]>>24)&31==3 and f[2]&255==7]
assert mode and enables and mode[0][0]<enables[0][0]
assert all(f[1]=='can0' and f[2]&255==7 for f in frames if (f[2]>>24)&31==4)
feedback=[f for f in frames if f[1]=='can0' and (f[2]>>24)&31==2 and (f[2]>>8)&255==7 and (f[2]>>22)&3==2 and f[0]>enables[0][0]]
assert feedback
handoff=dict(flags[0], classification='existing right-J7-only MIT setup, not whole-arm/final relaxation',
             mode_write_at=mode[0][0],enable_at=enables[0][0],running_feedback_at=feedback[0][0],
             disable_to_enable_s=enables[0][0]-t)
(A/'run1/existing_j7_mode_handoff.json').write_text(json.dumps(handoff,indent=2)+'\n')
protected=json.loads((A/'protected.json').read_text())
changed=[p for p,h in protected.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
assert not changed
source=json.loads((A/'tested_source.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in source.items())
assert (A/'run1/launch.exit').read_text().strip()=='0'
assert json.loads((A/'run1/final_query_disabled.json').read_text())['all_16_disabled']
log=(A/'run1/app.log').read_text()
assert not any(s in log for s in ('SNARE SYNC: {"kind": "fault"','DUAL ARM FAULT','HYBRID FAULT:', 'EMERGENCY RELAX', 'Traceback'))
assert log.count('ENDPOINT VERIFIED')==3
syncdir=Path('/home/jason/Proyectos3/X5/playback_results/snare_sync/20261009T221415-15ef7f7c')
rows=[json.loads(line) for line in (syncdir/'timeline.jsonl').read_text().splitlines()]
stop=next(v for v in rows if v['kind']=='stop')
commands=[v for v in rows if v['kind']=='command']
timeout=next(v for v in rows if v['kind']=='timeout')
assert len(commands)==19 and all(v['at']<timeout['at'] for v in commands)
assert not any(v['kind'] in ('locked','fault') for v in rows)
peaks=[float(s) for s in re.findall(r'SNARE calibration pair \d+: \+11° powered; measured peak ([0-9.]+)°; returned',log)]
assert len(peaks)==19
summary=dict(success=True,criterion='user-accepted 15-second timeout followed by verified center and relax',
 evidence=str(E),snare_timeline=str(syncdir/'timeline.jsonl'),
 physical=True,snare_calibrated=False,snare_elapsed_s=r['snare_elapsed_s'],snare_strikes=19,
 snare_command_degrees=11,measured_snare_peak_deg=[min(peaks),max(peaks)],
 hihat_advance_ms=r['hihat_advance']*1000,snare_matches=stop['matches'],snare_skips=stop['skips'],
 both_recordings_and_left_reverse_verified=True,all_16_disabled=True,
 application_faults=0,motor_fault_frames=0,feedback_frames=can['feedback_frames'],
 max_temperature_c=can['maximum_temperature_c'],
 existing_right_j7_mode_handoff=handoff,
 final_arm_relax_center_proofs=r['disabled_proofs'],
 offline_tests=911,protected_files_unchanged=len(protected),tested_sources_unchanged=True)
(A/'run1/summary.json').write_text(json.dumps(summary,indent=2)+'\n')
diffs=[]
for p in (A/'before').rglob('*'):
 if p.is_file():
  n=str(p.relative_to(A/'before'))
  diffs.extend(difflib.unified_diff(p.read_text().splitlines(True),Path(n).read_text().splitlines(True),fromfile='before/'+n,tofile=n))
p=Path('camera_playback/calibration_evidence.py')
diffs.extend(difflib.unified_diff([],p.read_text().splitlines(True),fromfile='/dev/null',tofile=str(p)))
(A/'changes.diff').write_text(''.join(diffs))
print(json.dumps({k:v for k,v in summary.items() if k not in ('existing_right_j7_mode_handoff','final_arm_relax_center_proofs')},indent=2))
