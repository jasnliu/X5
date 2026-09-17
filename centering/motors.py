"""Direct motor-native position control. No custom servo or heartbeat."""
import errno
import math
import struct
import time
from safe_zone.encoder import Observer, FRAME, EFF, request_frame, encoder_to_joint, joint_to_motor

SPEED = .4  # rad/s; motor firmware performs the position move
CURRENT = (6., 6., 4., 4., 2., 2., 2.)
RIGHT_GRIPPER_OPEN = math.radians(-3.)
RIGHT_GRIPPER_CLOSED = math.radians(7.)
GRIPPER_CURRENT = 2.

def packet(kind, motor, data=bytes(8)):
    return FRAME.pack(EFF | kind<<24 | 0xfd<<8 | motor, 8, data)

def parameter(motor, index, value, byte=False):
    data=struct.pack('<H2x',index)+(bytes([value,0,0,0]) if byte else struct.pack('<f',value))
    return packet(18,motor,data)

def allowed(frame, control_gripper=False):
    cid,_,data=FRAME.unpack(frame);i=cid&255
    if i not in range(1,9):return False
    if frame==packet(4,i):return True
    if i==8:
        if not control_gripper:return False
        if frame in (packet(3,8), parameter(8,0x7005,5,True),
                     parameter(8,0x7017,SPEED), parameter(8,0x7018,GRIPPER_CURRENT),
                     packet(17,8,bytes.fromhex('0570000000000000'))):return True
        if data[:2]!=b'\x16\x70':return False
        raw=struct.unpack('<f',data[4:])[0]
        return -RIGHT_GRIPPER_CLOSED-1e-6<=raw<=-RIGHT_GRIPPER_OPEN+1e-6
    return frame in (packet(3,i), parameter(i,0x7005,5,True),
                     parameter(i,0x7017,SPEED), parameter(i,0x7018,CURRENT[i-1]),
                     packet(17,i,bytes.fromhex('0570000000000000'))) or (data[:2]==b'\x16\x70' and struct.unpack('<f',data[4:])[0]==struct.unpack('<f',data[4:])[0] and abs(struct.unpack('<f',data[4:])[0])<=3.5)

class Motors(Observer):
    def __init__(self, control_side='left', control_gripper=False):
        if control_side not in ('left','right'):raise ValueError('Control side must be left or right')
        if control_gripper and control_side!='right':raise ValueError('Gripper control is right-Cartesian-only')
        super().__init__('can1','can0')
        self.control_side=control_side;self.control_gripper=control_gripper
        self.states={};self.modes={};self.last_query=0.;self.active=False

    def _send(self, side, frame):
        if side!=self.control_side:raise RuntimeError(f'{side.capitalize()} arm is query-only')
        if not allowed(frame,self.control_gripper):raise RuntimeError(f'Invalid {side}-motor command')
        for attempt in range(6):
            try:
                if self.sockets[side].send(frame)==16:return
                raise RuntimeError('Short CAN write')
            except OSError as e:
                if e.errno not in (errno.EAGAIN,errno.ENOBUFS) or attempt==5:raise
                time.sleep(.002)
        raise RuntimeError('CAN write failed')

    def send_left(self, frame):self._send('left',frame)
    def send_right(self, frame):self._send('right',frame)
    def send_control(self, frame):self._send(self.control_side,frame)

    def poll(self):
        now=time.monotonic()
        for side,s in self.sockets.items():
            for _ in range(512):
                try:frame=s.recv(16)
                except BlockingIOError:break
                cid,dlc,data=FRAME.unpack(frame)
                if cid&0x60000000:raise RuntimeError('CAN error')
                if not cid&EFF:continue
                kind=(cid>>24)&31;i=(cid>>8)&255
                if i not in range(1,9):continue
                if kind==21:raise RuntimeError(f'{side} motor {i}: fault')
                if kind not in (2,17):continue
                if dlc!=8:raise RuntimeError('Short feedback frame')
                if kind==17:
                    if side==self.control_side and data[:2]==b'\x05\x70':self.modes[i]=(data[4],now)
                    continue
                if (cid>>16)&63:raise RuntimeError(f'{side} motor {i}: fault')
                mode=(cid>>22)&3
                controlled=side==self.control_side and (i<=7 or self.control_gripper)
                if not controlled and mode!=0:raise RuntimeError(f'{side} motor {i} is not relaxed')
                if int.from_bytes(data[6:8],'big')*.1>=65:raise RuntimeError('Motor too hot')
                raw=int.from_bytes(data[:2],'big')/65535*25.14-12.57
                q=encoder_to_joint(side,i,raw) if i<=7 else -raw
                self.states[side,i]=(q,mode,now)
        if now-self.last_query>=.05:
            for s in self.sockets.values():
                for i in range(1,9):
                    if s.send(request_frame(i))!=16:raise RuntimeError('State query failed')
            self.last_query=now
        if self.active and not self.fresh():raise RuntimeError('Encoder feedback lost')

    def fresh(self):
        return len(self.states)==16 and all(time.monotonic()-v[2]<.3 for v in self.states.values())

    def positions(self):
        q={}
        for (side,i),(angle,_,_) in self.states.items():
            if i<=7:q[f'openarmx_{side}_joint{i}']=angle
            elif side==self.control_side and self.control_gripper:
                # Display the custom right gripper open at -3 degrees and
                # closed at +7 degrees using the standard RViz finger travel.
                fraction=(RIGHT_GRIPPER_CLOSED-angle)/(RIGHT_GRIPPER_CLOSED-RIGHT_GRIPPER_OPEN)
                q[f'openarmx_{side}_finger_joint1']=.044*max(0.,min(1.,fraction))
            else:q[f'openarmx_{side}_finger_joint1']=max(0.,min(.044,.044*angle/1.0472))
        return q

    def center(self, target=None, gripper_target=None):
        """Configure/enable the selected arm at its initial center target."""
        target=[0.]*7 if target is None else [float(q) for q in target]
        if len(target)!=7 or not all(math.isfinite(q) and abs(q)<=3.5 for q in target):
            raise RuntimeError('Invalid initial center target')
        if gripper_target is not None:
            gripper_target=float(gripper_target)
            if not self.control_gripper or not math.isfinite(gripper_target) or not RIGHT_GRIPPER_OPEN<=gripper_target<=RIGHT_GRIPPER_CLOSED:
                raise RuntimeError('Invalid right gripper target')
        if not self.fresh() or any(v[1]!=0 for v in self.states.values()):
            raise RuntimeError('Both arms and grippers must be relaxed with fresh encoder readings')
        self.active=True
        goals=target+([] if gripper_target is None else [gripper_target])
        currents=CURRENT+(() if gripper_target is None else (GRIPPER_CURRENT,))
        # Configure only the selected drives. Never recalibrate zero.
        for i,q in enumerate(goals,1):
            for frame in (parameter(i,0x7005,5,True),parameter(i,0x7017,SPEED),
                          parameter(i,0x7018,currents[i-1]),parameter(i,0x7016,joint_to_motor(self.control_side,i,q) if i<=7 else -q)):
                self.send_control(frame);yield
            self.modes.pop(i,None)
            self.send_control(packet(17,i,bytes.fromhex('0570000000000000')))
            deadline=time.monotonic()+.3
            while i not in self.modes:
                if time.monotonic()>deadline:raise RuntimeError(f'Motor {i}: position-mode readback missing')
                yield
            if self.modes[i][0]!=5:raise RuntimeError(f'Motor {i}: position mode not accepted')
        for i,q in enumerate(goals,1):
            self.send_control(packet(3,i));yield
            self.send_control(parameter(i,0x7016,joint_to_motor(self.control_side,i,q) if i<=7 else -q));yield

    def set_positions(self, joints):
        if not self.active or len(joints)!=7:raise RuntimeError(f'{self.control_side.capitalize()} arm is not active')
        for i,q in enumerate(joints,1):
            # OpenArmX uses -1 motor-to-URDF direction for every arm joint.
            self.send_control(parameter(i,0x7016,joint_to_motor(self.control_side,i,q)))

    def set_gripper(self, target):
        target=float(target)
        if not self.active or not self.control_gripper:raise RuntimeError('Right gripper is not active')
        if not math.isfinite(target) or not RIGHT_GRIPPER_OPEN<=target<=RIGHT_GRIPPER_CLOSED:
            raise RuntimeError('Invalid right gripper target')
        # Use the same displayed/encoder sign convention as joints 1-7.
        self.send_control(parameter(8,0x7016,-target))

    def relax(self):
        failures=[]
        for _ in range(3):
            for i in range(1,9):
                try:self.send_control(packet(4,i))
                except Exception as e:failures.append(str(e))
        if failures:raise RuntimeError('Disable delivery failed; use physical power cutoff')
        self.active=False
