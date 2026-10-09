"""Recovery: only release arms ALREADY at center; never commands a new pose."""
import json,time
from pathlib import Path
from centering.motors import Motors
from hihat_collection.transport import DataMotors
from hihat_collection.collect import notify
from smooth_playback.trajectory import Geometry
from x5_collection.collect import query_disabled,dump
p=Path(__file__).resolve().parent/'startup_center_recovery';p.mkdir(exist_ok=False)
notify(p,'recovery','Both arms are already at center. Verifying settled encoder feedback, then releasing at CENTER ONLY. No strike or new pose command.')
b=DataMotors.adopt(Motors('right',control_gripper=True),Geometry().center,directory=p);b.start_audit(p)
b.require_center_before_relax=True
try:
 for _ in range(160):b.poll();time.sleep(.005)
 b.left_center_evidence();b.center_evidence()
 b.active=True;b.left_drive.active=True
 b.relax()
 for _ in range(60):b.poll();time.sleep(.005)
 if not all(s[1]==0 for s in b.states.values()):raise RuntimeError('Disable not confirmed')
finally:b.close()
query_disabled(p,'query_final_disabled.json');print('BOTH ARMS VERIFIED CENTERED, THEN DISABLED')
