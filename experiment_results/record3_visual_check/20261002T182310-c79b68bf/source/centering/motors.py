"""Direct motor-native position control. No custom servo or heartbeat."""
import errno
import math
import struct
import time
from safe_zone.encoder import Observer, FRAME, EFF, request_frame, encoder_to_joint, joint_to_motor

SPEED = .4  # rad/s; motor firmware performs the position move
# The recorded-path camera program temporarily uses this for right joints 1-7,
# then restores SPEED before any alignment, recovery, or centering movement.
RIGHT_PLAYBACK_SPEED = .8
# Playback cymbal strikes are deliberately confined to right motor 7. The
# playback controller uses 3.5 rad/s in both directions so its return does not
# need a speed change. The separate standalone J7 test remains at 2 rad/s.
RIGHT_STRIKE_RETURN_SPEED = 3.5
RIGHT_STRIKE_DOWN_SPEED = 3.5
RIGHT_STRIKE_SPEED = 2.0
# RS00 wire-format ranges. Hardware-test dynamics and fall/return tuning live
# in camera_playback.mit_strike; these helpers only provide packet/recovery I/O.
RIGHT_JOINT7_MIT_POSITION_LIMIT = 12.57
RIGHT_JOINT7_MIT_VELOCITY_LIMIT = 33.0
RIGHT_JOINT7_MIT_TORQUE_LIMIT = 14.0
RIGHT_JOINT7_MIT_KP_LIMIT = 500.0
RIGHT_JOINT7_MIT_KD_LIMIT = 5.0
# Stationary fallback/recovery hold gains (not the strike trajectory gains).
RIGHT_JOINT7_RETURN_KP = 10.0
RIGHT_JOINT7_RETURN_KD = 0.5
# Existing powered-strike GUI feedback rate; unchanged for --hardware/--test.
# The separate hardware-test MIT session owns its own 500 Hz target worker.
MAX_RIGHT_JOINT7_FEEDBACK_HZ = 50.0
CENTER_ENABLE_ATTEMPTS = 3
CENTER_MODE_READBACK_TIMEOUT = 0.4
CENTER_ENABLE_CONFIRM_TIMEOUT = 0.5
MOTOR_RELAXED_STATE = 0
MOTOR_RUNNING_STATE = 2
CURRENT = (6., 6., 4., 4., 2., 2., 2.)
RIGHT_GRIPPER_OPEN = math.radians(-3.)
RIGHT_GRIPPER_CLOSED = math.radians(7.)
GRIPPER_CURRENT = 2.

def packet(kind, motor, data=bytes(8)):
    return FRAME.pack(EFF | kind<<24 | 0xfd<<8 | motor, 8, data)

def parameter(motor, index, value, byte=False):
    data=struct.pack('<H2x',index)+(bytes([value,0,0,0]) if byte else struct.pack('<f',value))
    return packet(18,motor,data)

def _uint16(value, minimum, maximum):
    value=float(value)
    if not math.isfinite(value):raise ValueError('MIT value must be finite')
    value=max(minimum,min(maximum,value))
    return int((value-minimum)*65535.0/(maximum-minimum))

def motion_control_packet(motor, position, velocity, kp, kd, torque):
    """Build the installed OpenArmX SDK's RobStride MIT/operation frame."""
    if motor!=7:raise ValueError('MIT hardware-test control is right-J7-only')
    position_raw=_uint16(position,-RIGHT_JOINT7_MIT_POSITION_LIMIT,
                         RIGHT_JOINT7_MIT_POSITION_LIMIT)
    velocity_raw=_uint16(velocity,-RIGHT_JOINT7_MIT_VELOCITY_LIMIT,
                         RIGHT_JOINT7_MIT_VELOCITY_LIMIT)
    kp_raw=_uint16(kp,0.0,RIGHT_JOINT7_MIT_KP_LIMIT)
    kd_raw=_uint16(kd,0.0,RIGHT_JOINT7_MIT_KD_LIMIT)
    torque_raw=_uint16(torque,-RIGHT_JOINT7_MIT_TORQUE_LIMIT,
                       RIGHT_JOINT7_MIT_TORQUE_LIMIT)
    data=struct.pack('>HHHH',position_raw,velocity_raw,kp_raw,kd_raw)
    return FRAME.pack(EFF | 1<<24 | torque_raw<<8 | motor,8,data)

def allowed(frame, control_gripper=False):
    cid,_,data=FRAME.unpack(frame);i=cid&255
    if i not in range(1,9):return False
    if (cid>>24)&31==1:return i==7
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
                     parameter(i,0x7017,RIGHT_PLAYBACK_SPEED),
                     packet(17,i,bytes.fromhex('0570000000000000'))) or (
                         i==7 and frame in (
                             parameter(7,0x7005,0,True),
                             parameter(7,0x7017,RIGHT_STRIKE_SPEED),
                             parameter(7,0x7017,RIGHT_STRIKE_RETURN_SPEED),
                             parameter(7,0x7017,RIGHT_STRIKE_DOWN_SPEED),
                         )
                     ) or (data[:2]==b'\x16\x70' and struct.unpack('<f',data[4:])[0]==struct.unpack('<f',data[4:])[0] and abs(struct.unpack('<f',data[4:])[0])<=3.5)

class Motors(Observer):
    def __init__(self, control_side='left', control_gripper=False):
        if control_side not in ('left','right'):raise ValueError('Control side must be left or right')
        if control_gripper and control_side!='right':raise ValueError('Gripper control is right-Cartesian-only')
        super().__init__('can1','can0')
        self.control_side=control_side;self.control_gripper=control_gripper
        self.states={};self.modes={};self.last_query=0.;self.active=False
        self.isolated_motors=set()
        self.right_joint7_query_period=None;self.last_right_joint7_query=0.
        self.right_joint7_session=None

    def _send(self, side, frame):
        if side!=self.control_side:raise RuntimeError(f'{side.capitalize()} arm is query-only')
        cid,_,_=FRAME.unpack(frame)
        if (side=='right' and cid&255==7
                and getattr(self,'right_joint7_session',None) is not None):
            raise RuntimeError('J7 commands belong to the hardware-test worker')
        if (cid>>24)&31==1 and (side!='right' or cid&255!=7):
            raise RuntimeError('MIT motion control is right-J7-only')
        if (frame in tuple(parameter(i,0x7017,RIGHT_PLAYBACK_SPEED) for i in range(1,8))
                and side!='right'):
            raise RuntimeError('Fast playback speed is right-arm-only')
        if frame in (parameter(7,0x7017,RIGHT_STRIKE_SPEED),
                     parameter(7,0x7017,RIGHT_STRIKE_RETURN_SPEED),
                     parameter(7,0x7017,RIGHT_STRIKE_DOWN_SPEED)) and side!='right':
            raise RuntimeError('Fast strike speed is right-J7-only')
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
                isolated=(side==self.control_side
                          and i in getattr(self,'isolated_motors',set()))
                if kind==21:
                    if isolated:continue
                    raise RuntimeError(f'{side} motor {i}: fault')
                if kind not in (2,17):continue
                if dlc!=8:raise RuntimeError('Short feedback frame')
                if kind==17:
                    if side==self.control_side and data[:2]==b'\x05\x70':self.modes[i]=(data[4],now)
                    continue
                if (cid>>16)&63:
                    if isolated:continue
                    raise RuntimeError(f'{side} motor {i}: fault')
                mode=(cid>>22)&3
                controlled=side==self.control_side and (i<=7 or self.control_gripper)
                if not controlled and mode!=0:raise RuntimeError(f'{side} motor {i} is not relaxed')
                if int.from_bytes(data[6:8],'big')*.1>=65:
                    if isolated:continue
                    raise RuntimeError(f'{side} motor {i}: too hot')
                raw=int.from_bytes(data[:2],'big')/65535*25.14-12.57
                q=encoder_to_joint(side,i,raw) if i<=7 else -raw
                self.states[side,i]=(q,mode,now)
        session=getattr(self,'right_joint7_session',None)
        # Keep ordinary state-only queries alive during process startup. Stop
        # them only when the child's MIT command replies are actually arriving.
        session_j7=session is not None and getattr(session,'owns_feedback',True)
        priority_j7=(not session_j7 and self.active and self.control_side=='right'
                     and self.right_joint7_query_period is not None)
        if (priority_j7
                and now-self.last_right_joint7_query>=self.right_joint7_query_period):
            if self.sockets['right'].send(request_frame(7))!=16:
                raise RuntimeError('Right J7 state query failed')
            self.last_right_joint7_query=now
        if now-self.last_query>=.05:
            for s in self.sockets.values():
                for i in range(1,9):
                    if (priority_j7 or session_j7) and s is self.sockets['right'] and i==7:continue
                    if s.send(request_frame(i))!=16:raise RuntimeError('State query failed')
            self.last_query=now
        if self.active and not self.fresh():raise RuntimeError('Encoder feedback lost')

    def fresh(self):
        if len(self.states)!=16:return False
        now=time.monotonic();isolated=getattr(self,'isolated_motors',set())
        return all(
            (side==self.control_side and motor in isolated)
            or now-state[2]<.3
            for (side,motor),state in self.states.items()
        )

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

    def _center_motor_target(self, motor, joint_position):
        return (joint_to_motor(self.control_side,motor,joint_position)
                if motor<=7 else -joint_position)

    def _send_center_setup_frame(self, motor, step, frame):
        try:self.send_control(frame)
        except Exception as exc:
            raise RuntimeError(f'Motor {motor}: {step} failed: {exc}') from exc

    def _prepare_center_motor_confirmed(self, motor, joint_position, current_limit):
        """Configure one drive at its live pose and prove that it enabled."""
        target=self._center_motor_target(motor,joint_position)
        last_mode=None;last_state=None
        for _attempt in range(1,CENTER_ENABLE_ATTEMPTS+1):
            # A prior process may have left J7 in MIT mode. Start every attempt
            # from an explicit disabled state before selecting CSP.
            self._send_center_setup_frame(
                motor,'explicit disable',packet(4,motor));yield
            for step,frame in (
                    ('CSP mode selection',parameter(motor,0x7005,5,True)),
                    ('speed-limit setup',parameter(motor,0x7017,SPEED)),
                    ('current-limit setup',parameter(motor,0x7018,current_limit)),
                    ('safe target preload',parameter(motor,0x7016,target))):
                self._send_center_setup_frame(motor,step,frame);yield
            self.modes.pop(motor,None)
            self._send_center_setup_frame(
                motor,'CSP mode query',
                packet(17,motor,bytes.fromhex('0570000000000000')))
            mode_deadline=time.monotonic()+CENTER_MODE_READBACK_TIMEOUT
            while motor not in self.modes and time.monotonic()<=mode_deadline:yield
            mode_result=self.modes.get(motor)
            last_mode=None if mode_result is None else int(mode_result[0])
            if last_mode!=5:continue

            # Preload the live pose before enabling. No center target is sent
            # until every selected drive has independently confirmed running.
            enabled_at=time.monotonic()
            self._send_center_setup_frame(
                motor,'enable',packet(3,motor));yield
            self._send_center_setup_frame(
                motor,'post-enable target hold',
                parameter(motor,0x7016,target));yield
            enable_deadline=time.monotonic()+CENTER_ENABLE_CONFIRM_TIMEOUT
            while time.monotonic()<=enable_deadline:
                state=self.states.get((self.control_side,motor))
                if state is not None:
                    last_state=int(state[1])
                    if last_state==MOTOR_RUNNING_STATE and state[2]>=enabled_at:return
                yield

        state=self.states.get((self.control_side,motor))
        encoder=(None if state is None else math.degrees(float(state[0])))
        encoder_text='unavailable' if encoder is None else f'{encoder:.3f} deg'
        raise RuntimeError(
            f'Motor {motor}: CSP enable not confirmed after '
            f'{CENTER_ENABLE_ATTEMPTS} attempts '
            f'(run-mode readback={last_mode}, feedback operating state={last_state}, '
            f'encoder={encoder_text})'
        )

    def center(self, target=None, gripper_target=None, confirm_enabled=False):
        """Configure/enable the selected arm at its initial center target."""
        target=[0.]*7 if target is None else [float(q) for q in target]
        if len(target)!=7 or not all(math.isfinite(q) and abs(q)<=3.5 for q in target):
            raise RuntimeError('Invalid initial center target')
        if gripper_target is not None:
            gripper_target=float(gripper_target)
            if not self.control_gripper or not math.isfinite(gripper_target) or not RIGHT_GRIPPER_OPEN<=gripper_target<=RIGHT_GRIPPER_CLOSED:
                raise RuntimeError('Invalid right gripper target')
        if not self.fresh() or any(v[1]!=MOTOR_RELAXED_STATE for v in self.states.values()):
            raise RuntimeError('Both arms and grippers must be relaxed with fresh encoder readings')
        self.isolated_motors=set()
        self.active=True
        goals=target+([] if gripper_target is None else [gripper_target])
        currents=CURRENT+(() if gripper_target is None else (GRIPPER_CURRENT,))
        if confirm_enabled:
            live_goals=[self.states[self.control_side,i][0]
                        for i in range(1,8)]
            if gripper_target is not None:
                # The physical gripper can be manually left outside this
                # program's deliberately narrow -3°..+7° command envelope.
                # Do not turn that raw live angle into a rejected command.
                # Arm drives 1-7 are confirmed first at their exact live poses;
                # then the gripper is safely commanded to the requested target.
                live_goals.append(gripper_target)
            for i,(q,current) in enumerate(zip(live_goals,currents),1):
                yield from self._prepare_center_motor_confirmed(i,q,current)
            return
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
            if i in getattr(self,'isolated_motors',set()):continue
            # OpenArmX uses -1 motor-to-URDF direction for every arm joint.
            self.send_control(parameter(i,0x7016,joint_to_motor(self.control_side,i,q)))

    def set_right_joint7_position(self, target):
        """Send one right-J7 target without delaying it behind J1-J6 writes."""
        target=float(target)
        if not self.active or self.control_side!='right':
            raise RuntimeError('Right arm must be active to command J7')
        if 7 in getattr(self,'isolated_motors',set()):
            raise RuntimeError('Right J7 is isolated after a motor fault')
        if not math.isfinite(target) or abs(target)>3.5:
            raise RuntimeError('Invalid right J7 target')
        self.send_control(parameter(7,0x7016,joint_to_motor('right',7,target)))

    def _require_right_joint7(self):
        if not self.active or self.control_side!='right':
            raise RuntimeError('Right arm must be active to control J7')
        if 7 in getattr(self,'isolated_motors',set()):
            raise RuntimeError('Right J7 is isolated after a motor fault')

    def set_right_joint7_mit(self, position, velocity, kp, kd, torque=0.0):
        """Send one right-J7 MIT command using displayed joint coordinates."""
        self._require_right_joint7()
        values=tuple(float(value) for value in (position,velocity,kp,kd,torque))
        position,velocity,kp,kd,torque=values
        if (not all(math.isfinite(value) for value in values)
                or abs(position)>3.5
                or abs(velocity)>RIGHT_JOINT7_MIT_VELOCITY_LIMIT
                or not 0.0<=kp<=RIGHT_JOINT7_MIT_KP_LIMIT
                or not 0.0<=kd<=RIGHT_JOINT7_MIT_KD_LIMIT
                or abs(torque)>RIGHT_JOINT7_MIT_TORQUE_LIMIT):
            raise RuntimeError('Invalid right J7 MIT command')
        # Every OpenArmX arm joint uses the opposite motor/URDF direction.
        self.send_control(motion_control_packet(
            7,joint_to_motor('right',7,position),-velocity,kp,kd,-torque
        ))


    def request_right_joint7_mode_readback(self, clear=False):
        """Request J7's run mode without disturbing any other motor."""
        self._require_right_joint7()
        if clear:self.modes.pop(7,None)
        self.send_control(packet(17,7,bytes.fromhex('0570000000000000')))

    def right_joint7_mode_readback(self):
        result=self.modes.get(7)
        return None if result is None else int(result[0])

    def right_joint7_operating_state(self):
        """Return J7's feedback operating state and observation time."""
        result=self.states.get(('right',7))
        return None if result is None else (int(result[1]),float(result[2]))


    def command_right_joint7_mit_hold(self, anchor_position):
        """Stationary recovery hold with zero desired MIT velocity."""
        self.set_right_joint7_mit(
            anchor_position,0.0,
            RIGHT_JOINT7_RETURN_KP,RIGHT_JOINT7_RETURN_KD,0.0,
        )

    def reassert_right_joint7_csp_hold(self, anchor_position):
        """Enable J7 in its configured CSP mode and hold the anchor."""
        self._require_right_joint7()
        enabled_at=time.monotonic()
        self.send_control(packet(3,7))
        self.set_right_joint7_position(anchor_position)
        self.request_right_joint7_mode_readback()
        return enabled_at

    def hold_right_joint7_unknown_mode(self, anchor_position):
        """Target the same stationary J7 pose in both MIT and CSP protocols."""
        # If the run-mode readback was lost, do not guess which command family
        # the drive accepted. Both frames request the same stationary pose; the
        # drive applies the one matching its active mode.
        self.command_right_joint7_mit_hold(anchor_position)
        self.set_right_joint7_position(anchor_position)

    def restore_right_joint7_csp(self, anchor_position):
        """Restore ordinary CSP when leaving the stationary MIT session."""
        anchor_position=float(anchor_position)
        self._require_right_joint7()
        if not math.isfinite(anchor_position) or abs(anchor_position)>3.5:
            raise RuntimeError('Invalid right J7 CSP restore position')
        # Leave the already stationary MIT hold and preload the same target
        # in CSP before re-enabling it. Never used for per-strike braking.
        self.command_right_joint7_mit_hold(anchor_position)
        self.send_control(packet(4,7))
        time.sleep(.010)
        for frame in (
            parameter(7,0x7005,5,True),
            parameter(7,0x7017,SPEED),
            parameter(7,0x7018,CURRENT[6]),
            parameter(7,0x7016,joint_to_motor('right',7,anchor_position)),
        ):
            self.send_control(frame)
        time.sleep(.002)
        self.modes.pop(7,None)
        enabled_at=self.reassert_right_joint7_csp_hold(anchor_position)
        # reassert_right_joint7_csp_hold requests a fresh run-mode result after
        # the enable/hold commands were transmitted.
        return enabled_at

    def set_gripper(self, target):
        target=float(target)
        if not self.active or not self.control_gripper:raise RuntimeError('Right gripper is not active')
        if 8 in getattr(self,'isolated_motors',set()):raise RuntimeError('Right gripper is isolated after a motor fault')
        if not math.isfinite(target) or not RIGHT_GRIPPER_OPEN<=target<=RIGHT_GRIPPER_CLOSED:
            raise RuntimeError('Invalid right gripper target')
        # Use the same displayed/encoder sign convention as joints 1-7.
        self.send_control(parameter(8,0x7016,-target))

    def isolate_control_motor(self, motor):
        """Disable one selected drive while leaving every other drive powered."""
        motor=int(motor)
        if not self.active or motor not in range(1,9):
            raise RuntimeError('Invalid motor isolation request')
        if motor==8 and not self.control_gripper:
            raise RuntimeError('Gripper isolation is unavailable')
        self.send_control(packet(4,motor))
        if not hasattr(self,'isolated_motors'):self.isolated_motors=set()
        self.isolated_motors.add(motor)
        if motor==7:
            self.right_joint7_query_period=None
            self.last_right_joint7_query=0.

    def set_right_joint7_speed(self, speed):
        """Select only an explicitly allowlisted speed for right J7."""
        speed=float(speed)
        if not self.active or self.control_side!='right':
            raise RuntimeError('Right arm must be active to change J7 speed')
        if speed not in (SPEED,RIGHT_STRIKE_SPEED,
                         RIGHT_STRIKE_RETURN_SPEED,RIGHT_STRIKE_DOWN_SPEED):
            raise RuntimeError('Invalid right J7 speed')
        self.send_control(parameter(7,0x7017,speed))

    def set_right_joint7_feedback_rate(self, rate_hz=None):
        """Temporarily prioritize right-J7 state without raising other rates."""
        if rate_hz is None:
            self.right_joint7_query_period=None
            self.last_right_joint7_query=0.
            return
        if getattr(self,'right_joint7_session',None) is not None:
            raise RuntimeError('J7 feedback belongs to the hardware-test worker')
        rate_hz=float(rate_hz)
        if not self.active or self.control_side!='right':
            raise RuntimeError('Right arm must be active to prioritize J7 feedback')
        if (not math.isfinite(rate_hz) or rate_hz<=0.
                or rate_hz>MAX_RIGHT_JOINT7_FEEDBACK_HZ):
            raise RuntimeError('Invalid right J7 feedback rate')
        if self.sockets['right'].send(request_frame(7))!=16:
            raise RuntimeError('Right J7 state query failed')
        self.right_joint7_query_period=1./rate_hz
        self.last_right_joint7_query=time.monotonic()

    def set_right_arm_speed(self, speed):
        """Select normal or playback speed for all seven active right joints."""
        speed=float(speed)
        if not self.active or self.control_side!='right':
            raise RuntimeError('Right arm must be active to change playback speed')
        if speed not in (SPEED,RIGHT_PLAYBACK_SPEED):
            raise RuntimeError('Invalid right-arm playback speed')
        for motor in range(1,8):
            if motor in getattr(self,'isolated_motors',set()):continue
            self.send_control(parameter(motor,0x7017,speed))

    def relax(self):
        self.stop_right_joint7_session()
        failures=[]
        for _ in range(3):
            for i in range(1,9):
                try:self.send_control(packet(4,i))
                except Exception as e:failures.append(str(e))
        if failures:raise RuntimeError('Disable delivery failed; use physical power cutoff')
        self.active=False;self.isolated_motors=set()
        self.right_joint7_query_period=None;self.last_right_joint7_query=0.

    def stop_right_joint7_session(self):
        session=getattr(self,'right_joint7_session',None)
        if session is not None:session.stop()

    def close(self):
        self.stop_right_joint7_session()
        super().close()
