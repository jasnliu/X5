"""Passive full-run audit: fault bits and center pose before EVERY disable."""
import argparse,json,math,re
from pathlib import Path
import numpy as np
from safe_zone.encoder import encoder_to_joint
from camera_playback.left_hold import LEFT_CENTER
from smooth_playback.trajectory import Geometry
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
goals={'left':LEFT_CENTER,'right':Geometry().center}
states={};faults=[];disables=[];unproven=[];max_temp=0;feedback=0
rx=re.compile(r'^\(([0-9.]+)\)\s+(can[01])\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)')
for line in (a.directory/'can.log').open():
    m=rx.match(line)
    if not m:continue
    stamp=float(m[1]);side='left' if m[2]=='can1' else 'right';cid=int(m[3],16);payload=bytes.fromhex(m[4]);kind=(cid>>24)&31
    if kind in (2,21):
        motor=(cid>>8)&255
        if motor not in range(1,9) or len(payload)!=8:continue
        if kind==21 or (cid>>16)&63:
            faults.append(dict(t=stamp,side=side,id=hex(cid),data=m[4]));continue
        feedback+=1
        state=(cid>>22)&3
        raw=int.from_bytes(payload[:2],'big')/65535*25.14-12.57
        states[side,motor]=(encoder_to_joint(side,motor,raw) if motor<8 else -raw,state,stamp)
        max_temp=max(max_temp,int.from_bytes(payload[6:8],'big')*.1)
    elif kind==4:
        motor=cid&255
        if motor not in range(1,9):continue
        before=states.get((side,motor))
        if before is not None and before[1]==0:
            disables.append(dict(t=stamp,side=side,motor=motor,already_disabled=True));continue
        known=all((side,i) in states for i in range(1,8))
        error=max(abs(states[side,i][0]-goals[side][i-1]) for i in range(1,8))*180/math.pi if known else None
        age=max(stamp-states[side,i][2] for i in range(1,8)) if known else None
        row=dict(t=stamp,side=side,motor=motor,already_disabled=False,max_center_error_deg=error,max_feedback_age_s=age)
        disables.append(row)
        if not known or error>.2 or age>.15:unproven.append(row)
result=dict(feedback_frames=feedback,motor_fault_frames=len(faults),faults=faults,
            maximum_temperature_c=max_temp,disable_commands=len(disables),
            active_disable_commands=sum(not r['already_disabled'] for r in disables),
            unproven_center_disable_commands=unproven,disables=disables)
(a.directory/'can_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print({k:v for k,v in result.items() if k not in ('disables','faults')})
assert not faults and not unproven
