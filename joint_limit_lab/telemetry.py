"""Pure RobStride feedback decoder for recorded frames. NO sockets or commands.

Position/sign conversion is the installed X5 convention. Speed/torque ranges
are protocol decoding scales from the installed SDK, NEVER operating limits.
"""
import math
import struct
from safe_zone.encoder import FRAME,EFF,encoder_to_joint

MODELS=('RS04','RS04','RS03','RS03','RS00','RS00','RS00')
VELOCITY_SCALES=(15.,15.,20.,20.,33.,33.,33.)
TORQUE_SCALES=(120.,120.,60.,60.,14.,14.,14.)


def decode(frame,side):
    if side not in ('right','left'):raise ValueError('Invalid arm')
    if not isinstance(frame,bytes) or len(frame)!=FRAME.size:raise ValueError('Malformed CAN recording')
    cid,dlc,data=FRAME.unpack(frame)
    if cid&0x60000000:raise ValueError('Recorded CAN error/RTR frame')
    if not cid&EFF:return None
    kind=(cid>>24)&31;joint=(cid>>8)&255
    if joint not in range(1,8):return None  # includes outgoing query echoes and gripper
    if dlc!=8:raise ValueError('Short motor response')
    if kind==21:return dict(side=side,joint=joint,kind='fault_report',raw=data.hex())
    if kind==17:
        index=struct.unpack('<H',data[:2])[0]
        if index not in (0x701A,0x701C,0x7017,0x7018):return None
        value=struct.unpack('<f',data[4:])[0]
        if not math.isfinite(value):raise ValueError('Nonfinite parameter feedback')
        return dict(side=side,joint=joint,kind='parameter',index=index,value=value)
    if kind!=2:return None
    pos,vel,tau,temp=struct.unpack('>4H',data)
    return dict(side=side,joint=joint,model=MODELS[joint-1],kind='state',
        q_rad=encoder_to_joint(side,joint,pos/65535*25.14-12.57),
        velocity_rad_s=-(vel/65535*2-1)*VELOCITY_SCALES[joint-1],
        torque_nm=-(tau/65535*2-1)*TORQUE_SCALES[joint-1],temperature_c=temp*.1,
        fault_bits=(cid>>16)&63,mode=(cid>>22)&3,
        interpretation='Torque is drive-reported, not an external calibrated load measurement')
