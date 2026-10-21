"""Direct zero-command tests; mocked sockets only."""
import math
import struct
import time
import unittest
from unittest.mock import Mock, call, patch
from centering.motors import Motors,packet,parameter,motion_control_packet,allowed,CURRENT,SPEED,RIGHT_PLAYBACK_SPEED,RIGHT_STRIKE_DOWN_SPEED,RIGHT_STRIKE_RETURN_SPEED,RIGHT_STRIKE_SPEED,RIGHT_GRIPPER_OPEN,RIGHT_GRIPPER_CLOSED,MAX_RIGHT_JOINT7_FEEDBACK_HZ,RIGHT_JOINT7_RETURN_KP,RIGHT_JOINT7_RETURN_KD
from safe_zone.encoder import FRAME,EFF,RIGHT_JOINT5_ZERO_OFFSET,request_frame

def fake(side='left',gripper=False):
    bus=Motors.__new__(Motors)
    bus.control_side=side;bus.control_gripper=gripper
    bus.states={(side,i):(0.,0,time.monotonic()) for side in ('left','right') for i in range(1,9)}
    bus.modes={};bus.active=False;bus.isolated_motors=set();bus.sockets={'left':Mock(),'right':Mock()}
    bus.last_query=0.;bus.right_joint7_query_period=None;bus.last_right_joint7_query=0.
    bus.sockets['left'].send.return_value=16;bus.sockets['right'].send.return_value=16
    return bus

class DirectCenterTests(unittest.TestCase):
    def test_sdk_csp_bytes(self):
        # Actual installed SDK csp_* helpers, evaluated without opening CAN.
        self.assertEqual(SPEED,.4)
        cases=((0x7005,5,True,'0570000005000000'),(0x7017,SPEED,False,'17700000cdcccc3e'),
               (0x7018,6.,False,'187000000000c040'),(0x7016,0.,False,'1670000000000000'))
        for index,value,byte,data in cases:
            self.assertEqual(parameter(1,index,value,byte),FRAME.pack(EFF|0x1200fd01,8,bytes.fromhex(data)))

    def test_all_seven_receive_only_zero_goals_right_untouched(self):
        bus=fake();sent=[]
        def send(frame):
            sent.append(frame);cid,_,data=FRAME.unpack(frame)
            if (cid>>24)&31==17:bus.modes[cid&255]=(5,time.monotonic())
            return 16
        bus.sockets['left'].send.side_effect=send
        list(bus.center())
        goals=[(cid&255,struct.unpack('<f',data[4:])[0]) for cid,_,data in map(FRAME.unpack,sent) if data[:2]==b'\x16\x70']
        self.assertEqual(goals,[(i,0.) for i in range(1,8)]*2)
        self.assertEqual([FRAME.unpack(f)[0]&255 for f in sent if (FRAME.unpack(f)[0]>>24)&31==3],list(range(1,8)))
        bus.sockets['right'].send.assert_not_called()

    def test_right_center_uses_only_right_ids_and_joint7_positive_limit(self):
        bus=fake('right');sent=[];target=[0.]*6+[1.4]
        def send(frame):
            sent.append(frame);cid,_,_=FRAME.unpack(frame)
            if (cid>>24)&31==17:bus.modes[cid&255]=(5,time.monotonic())
            return 16
        bus.sockets['right'].send.side_effect=send
        list(bus.center(target))
        goals=[(cid&255,struct.unpack('<f',data[4:])[0]) for cid,_,data in map(FRAME.unpack,sent) if data[:2]==b'\x16\x70']
        expected=[(i,RIGHT_JOINT5_ZERO_OFFSET if i==5 else 0. if i<7 else -1.4) for i in range(1,8)]*2
        self.assertEqual([i for i,_ in goals],[i for i,_ in expected])
        for (_,actual),(_,wanted) in zip(goals,expected):self.assertAlmostEqual(actual,wanted,places=6)
        self.assertEqual([FRAME.unpack(f)[0]&255 for f in sent if (FRAME.unpack(f)[0]>>24)&31==3],list(range(1,8)))
        bus.sockets['left'].send.assert_not_called()

    def test_confirmed_center_preloads_live_pose_and_confirms_every_drive(self):
        bus=fake('right',True)
        for i in range(1,9):
            # Deliberately leave the gripper well outside the program's narrow
            # -3°..+7° command envelope. Confirmed startup must use the safe
            # requested close target rather than replaying this raw live angle.
            bus.states['right',i]=(0.1*i if i<=7 else 0.5,0,time.monotonic())
        sent=[]
        def send(frame):
            self.assertTrue(allowed(frame,True))
            sent.append(frame)
        bus.send_control=Mock(side_effect=send)
        setup=bus.center([0.]*6+[1.4],RIGHT_GRIPPER_CLOSED,confirm_enabled=True)

        while True:
            try:next(setup)
            except StopIteration:break
            cid,_,data=FRAME.unpack(sent[-1]);kind=(cid>>24)&31;motor=cid&255
            if kind==17:
                bus.modes[motor]=(5,time.monotonic())
            elif kind==3:
                q=bus.states['right',motor][0]
                bus.states['right',motor]=(q,2,time.monotonic()+.001)

        self.assertTrue(all(bus.states['right',i][1]==2 for i in range(1,9)))
        self.assertEqual(
            [FRAME.unpack(frame)[0]&255 for frame in sent
             if (FRAME.unpack(frame)[0]>>24)&31==3],
            list(range(1,9)),
        )
        j7_targets=[struct.unpack('<f',data[4:])[0]
                    for cid,_,data in map(FRAME.unpack,sent)
                    if cid&255==7 and data[:2]==b'\x16\x70']
        self.assertEqual(len(j7_targets),2)
        for value in j7_targets:self.assertAlmostEqual(value,-0.7,places=6)
        self.assertTrue(all(abs(value+1.4)>.1 for value in j7_targets))
        gripper_targets=[struct.unpack('<f',data[4:])[0]
                         for cid,_,data in map(FRAME.unpack,sent)
                         if cid&255==8 and data[:2]==b'\x16\x70']
        self.assertEqual(len(gripper_targets),2)
        for value in gripper_targets:
            self.assertAlmostEqual(value,-RIGHT_GRIPPER_CLOSED,places=6)

    def test_confirmed_center_command_error_identifies_motor_and_step(self):
        bus=fake('right',True);bus.send_control=Mock(
            side_effect=RuntimeError('Invalid right-motor command'))
        with self.assertRaisesRegex(
                RuntimeError,
                'Motor 7: safe target preload failed: Invalid right-motor command'):
            bus._send_center_setup_frame(
                7,'safe target preload',parameter(7,0x7016,0.0))

    def test_confirmed_center_retries_j7_then_aborts_without_center_target(self):
        bus=fake('right',True);bus.fresh=Mock(return_value=True)
        sent=[];clock=[0.0]
        def monotonic():
            clock[0]+=.1
            return clock[0]
        bus.send_control=Mock(side_effect=lambda frame:sent.append(frame))
        setup=bus.center([0.]*6+[1.4],RIGHT_GRIPPER_CLOSED,confirm_enabled=True)

        with patch('centering.motors.time.monotonic',side_effect=monotonic):
            with self.assertRaisesRegex(
                    RuntimeError,
                    r'Motor 7: CSP enable not confirmed after 3 attempts.*operating state=0'):
                while True:
                    next(setup)
                    cid,_,_=FRAME.unpack(sent[-1]);kind=(cid>>24)&31;motor=cid&255
                    if kind==17:
                        bus.modes[motor]=(5,monotonic())
                    elif kind==3 and motor!=7:
                        q=bus.states['right',motor][0]
                        bus.states['right',motor]=(q,2,monotonic()+.01)

        j7_enables=[frame for frame in sent if frame==packet(3,7)]
        self.assertEqual(len(j7_enables),3)
        j7_targets=[struct.unpack('<f',data[4:])[0]
                    for cid,_,data in map(FRAME.unpack,sent)
                    if cid&255==7 and data[:2]==b'\x16\x70']
        self.assertTrue(j7_targets)
        self.assertTrue(all(abs(value)<1e-6 for value in j7_targets))

    def test_right_joint5_command_applies_offset_and_no_other_joint_changes(self):
        bus=fake('right');bus.active=True;bus._send=Mock()
        bus.set_positions([0.]*7)
        commands=[]
        for side,frame in (call.args for call in bus._send.call_args_list):
            cid,_,data=FRAME.unpack(frame)
            commands.append((side,cid&255,struct.unpack('<f',data[4:])[0]))
        self.assertEqual([(side,motor) for side,motor,_ in commands],[('right',i) for i in range(1,8)])
        for _,motor,value in commands:
            self.assertAlmostEqual(value,RIGHT_JOINT5_ZERO_OFFSET if motor==5 else 0.,places=6)

    def test_right_control_requires_left_relaxed_and_relaxes_only_right(self):
        bus=fake('right');bus.states['left',1]=(0.,2,time.monotonic())
        with self.assertRaises(RuntimeError):next(bus.center([0.]*6+[1.4]))
        bus.sockets['right'].send.assert_not_called();bus.states['left',1]=(0.,0,time.monotonic())
        bus.relax();calls=bus.sockets['right'].send.call_args_list
        self.assertEqual([FRAME.unpack(c.args[0])[0]&255 for c in calls],list(range(1,9))*3)
        bus.sockets['left'].send.assert_not_called()

    def test_gripper_and_recalibration_blocked(self):
        for frame in (packet(3,8),parameter(8,0x7016,0.),packet(6,1)):
            self.assertFalse(allowed(frame))
        self.assertTrue(allowed(packet(4,8)))

    def test_right_cartesian_gripper_setup_uses_only_small_degree_targets(self):
        bus=fake('right',True);sent=[]
        def send(frame):
            sent.append(frame);cid,_,_=FRAME.unpack(frame)
            if (cid>>24)&31==17:bus.modes[cid&255]=(5,time.monotonic())
            return 16
        bus.sockets['right'].send.side_effect=send
        list(bus.center([0.]*6+[1.4],RIGHT_GRIPPER_OPEN))
        gripper=[struct.unpack('<f',data[4:])[0] for cid,_,data in map(FRAME.unpack,sent) if cid&255==8 and data[:2]==b'\x16\x70']
        self.assertEqual(len(gripper),2)
        for value in gripper:self.assertAlmostEqual(value,math.radians(3),places=6)
        self.assertTrue(allowed(parameter(8,0x7016,math.radians(3)),True))
        self.assertTrue(allowed(parameter(8,0x7016,-math.radians(7)),True))
        self.assertFalse(allowed(parameter(8,0x7016,-math.radians(8)),True))
        bus.sockets['left'].send.assert_not_called()

    def test_right_cartesian_gripper_closes_to_displayed_positive_seven(self):
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        bus.set_gripper(RIGHT_GRIPPER_CLOSED)
        side,frame=bus._send.call_args.args;cid,_,data=FRAME.unpack(frame)
        self.assertEqual(side,'right');self.assertEqual(cid&255,8)
        self.assertAlmostEqual(struct.unpack('<f',data[4:])[0],-math.radians(7),places=6)
        with self.assertRaises(RuntimeError):bus.set_gripper(math.radians(8))

    def test_fast_speed_is_allowlisted_only_for_active_right_joint7(self):
        self.assertEqual(RIGHT_STRIKE_SPEED, 2.0)
        self.assertEqual(RIGHT_STRIKE_DOWN_SPEED, 3.5)
        self.assertEqual(RIGHT_STRIKE_RETURN_SPEED, 3.5)
        self.assertEqual(RIGHT_STRIKE_DOWN_SPEED, RIGHT_STRIKE_RETURN_SPEED)
        self.assertTrue(allowed(parameter(7,0x7017,RIGHT_STRIKE_SPEED)))
        self.assertTrue(allowed(parameter(7,0x7017,RIGHT_STRIKE_DOWN_SPEED)))
        self.assertFalse(allowed(parameter(6,0x7017,RIGHT_STRIKE_SPEED)))
        self.assertFalse(allowed(parameter(6,0x7017,RIGHT_STRIKE_DOWN_SPEED)))
        self.assertFalse(allowed(parameter(7,0x7017,2.5)))
        self.assertFalse(allowed(parameter(8,0x7017,RIGHT_STRIKE_SPEED),True))
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        bus.set_right_joint7_speed(RIGHT_STRIKE_DOWN_SPEED)
        bus.set_right_joint7_speed(RIGHT_STRIKE_RETURN_SPEED)
        bus.set_right_joint7_speed(SPEED)
        self.assertEqual(
            [call.args for call in bus._send.call_args_list],
            [('right',parameter(7,0x7017,RIGHT_STRIKE_DOWN_SPEED)),
             ('right',parameter(7,0x7017,RIGHT_STRIKE_RETURN_SPEED)),
             ('right',parameter(7,0x7017,SPEED))],
        )
        with self.assertRaises(RuntimeError):bus.set_right_joint7_speed(2.5)
        left=fake('left')
        with self.assertRaisesRegex(RuntimeError,'right-J7-only'):
            left._send('left',parameter(7,0x7017,RIGHT_STRIKE_SPEED))
        with self.assertRaisesRegex(RuntimeError,'right-J7-only'):
            left._send('left',parameter(7,0x7017,RIGHT_STRIKE_DOWN_SPEED))
        left.sockets['left'].send.assert_not_called()

    def test_right_joint7_position_uses_one_immediate_motor7_frame(self):
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        bus.set_right_joint7_position(1.25)
        self.assertEqual(bus._send.call_count,1)
        side,frame=bus._send.call_args.args
        cid,_,data=FRAME.unpack(frame)
        self.assertEqual((side,cid&255),('right',7))
        self.assertAlmostEqual(struct.unpack('<f',data[4:])[0],-1.25,places=6)
        with self.assertRaises(RuntimeError):bus.set_right_joint7_position(float('nan'))

    def test_isolating_one_drive_leaves_other_right_drives_commandable(self):
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        bus.isolate_control_motor(7)
        self.assertEqual(bus.isolated_motors,{7})
        self.assertEqual(bus._send.call_args_list,[call('right',packet(4,7))])

        bus._send.reset_mock()
        bus.set_positions([0.]*7)
        commanded=[FRAME.unpack(call.args[1])[0]&255 for call in bus._send.call_args_list]
        self.assertEqual(commanded,list(range(1,7)))
        with self.assertRaisesRegex(RuntimeError,'J7 is isolated'):
            bus.set_right_joint7_position(0.0)

    def test_right_joint7_mit_packet_matches_installed_openarmx_sdk(self):
        # These bytes were generated offline with the installed SDK's
        # CanPacketEncoder::create_motion_control_command for an RS00 J7.
        self.assertEqual(
            motion_control_packet(7,0.0,0.0,0.0,0.0,0.0),
            FRAME.pack(EFF|0x017fff07,8,bytes.fromhex('7fff7fff00000000')),
        )
        self.assertEqual(
            motion_control_packet(7,1.0,-3.5,10.0,1.0,2.0),
            FRAME.pack(EFF|0x01924807,8,bytes.fromhex('8a2e726c051e3333')),
        )
        minimum_nonzero_kd=5.0/65535.0  # Packet resolution, not fall tuning.
        self.assertEqual(
            FRAME.unpack(motion_control_packet(
                7,0.0,0.0,0.0,minimum_nonzero_kd,0.0
            ))[2],
            bytes.fromhex('7fff7fff00000001'),
        )
        self.assertTrue(allowed(motion_control_packet(
            7,0.0,0.0,0.0,minimum_nonzero_kd,0.0
        )))
        with self.assertRaises(ValueError):
            motion_control_packet(5,0.0,0.0,0.0,0.0,0.0)

    def test_left_joint6_mit_packet_is_permitted_and_self_consistent(self):
        # No installed-SDK reference bytes exist yet for motor 6 (unlike the
        # captured J7 bytes above) - this only checks internal self-consistency
        # of the (now permitted) encoding, not a captured hardware reference.
        frame = motion_control_packet(6,1.0,-3.5,10.0,1.0,2.0)
        cid,dlc,data = FRAME.unpack(frame)
        self.assertEqual(cid&255,6)
        self.assertEqual((cid>>24)&31,1)
        self.assertEqual(dlc,8)
        self.assertTrue(allowed(frame))
        with self.assertRaises(ValueError):
            motion_control_packet(8,0.0,0.0,0.0,0.0,0.0)

    def test_hardware_test_session_exit_restores_csp_at_stationary_anchor(self):
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        with patch('centering.motors.time.sleep'), patch(
                'centering.motors.time.monotonic',return_value=12.0):
            bus.restore_right_joint7_csp(1.0)
        restore=[frame for _,frame in (call.args for call in bus._send.call_args_list)]
        kinds=[(FRAME.unpack(frame)[0]>>24)&31 for frame in restore]
        self.assertEqual(kinds[0],1)
        self.assertEqual(
            restore[0],
            motion_control_packet(
                7,-1.0,0.0,RIGHT_JOINT7_RETURN_KP,
                RIGHT_JOINT7_RETURN_KD,0.0,
            ),
        )
        self.assertEqual(restore[1],packet(4,7))
        self.assertIn(parameter(7,0x7005,5,True),restore)
        self.assertIn(parameter(7,0x7017,SPEED),restore)
        self.assertIn(parameter(7,0x7018,CURRENT[6]),restore)
        self.assertEqual(restore[-3],packet(3,7))
        self.assertEqual(
            restore[-2],parameter(7,0x7016,-1.0)
        )
        self.assertEqual(
            restore[-1],packet(17,7,bytes.fromhex('0570000000000000'))
        )

    def test_right_joint7_csp_reassert_enables_holds_and_queries(self):
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        with patch('centering.motors.time.monotonic',return_value=15.0):
            enabled_at=bus.reassert_right_joint7_csp_hold(1.0)
        frames=[frame for _,frame in (call.args for call in bus._send.call_args_list)]
        self.assertEqual(enabled_at,15.0)
        self.assertEqual(frames,[
            packet(3,7),
            parameter(7,0x7016,-1.0),
            packet(17,7,bytes.fromhex('0570000000000000')),
        ])

    def test_unknown_mode_fault_hold_targets_same_j7_pose_in_mit_and_csp(self):
        bus=fake('right',True);bus.active=True;bus._send=Mock()
        bus.hold_right_joint7_unknown_mode(1.0)
        sent=[call.args for call in bus._send.call_args_list]
        self.assertEqual([side for side,_ in sent],['right','right'])
        self.assertTrue(all((FRAME.unpack(frame)[0]&255)==7 for _,frame in sent))
        self.assertEqual(
            sent[0][1],
            motion_control_packet(
                7,-1.0,0.0,RIGHT_JOINT7_RETURN_KP,
                RIGHT_JOINT7_RETURN_KD,0.0,
            ),
        )
        self.assertEqual(sent[1][1],parameter(7,0x7016,-1.0))

    def test_strike_priority_queries_only_right_j7_at_fifty_hz(self):
        bus=fake('right',True);bus.active=True
        for socket in bus.sockets.values():socket.recv.side_effect=BlockingIOError
        with patch('centering.motors.time.monotonic',return_value=10.0):
            bus.set_right_joint7_feedback_rate(MAX_RIGHT_JOINT7_FEEDBACK_HZ)
        bus.sockets['right'].send.assert_called_once_with(request_frame(7))
        self.assertAlmostEqual(bus.right_joint7_query_period,.02)

        for socket in bus.sockets.values():socket.send.reset_mock()
        bus.last_query=0.;bus.last_right_joint7_query=0.
        bus.states={(side,i):(0.,0,10.) for side in ('left','right') for i in range(1,9)}
        with patch('centering.motors.time.monotonic',return_value=10.0):bus.poll()
        right_queries=[call.args[0] for call in bus.sockets['right'].send.call_args_list]
        self.assertEqual(right_queries.count(request_frame(7)),1)
        self.assertEqual(len(right_queries),8)
        self.assertEqual(bus.sockets['left'].send.call_count,8)

        for socket in bus.sockets.values():socket.send.reset_mock()
        with patch('centering.motors.time.monotonic',return_value=10.021):bus.poll()
        bus.sockets['right'].send.assert_called_once_with(request_frame(7))
        bus.sockets['left'].send.assert_not_called()

        bus.set_right_joint7_feedback_rate(None)
        self.assertIsNone(bus.right_joint7_query_period)
        with self.assertRaises(RuntimeError):
            bus.set_right_joint7_feedback_rate(MAX_RIGHT_JOINT7_FEEDBACK_HZ+1.)

    def test_playback_speed_is_allowlisted_only_for_active_right_arm(self):
        self.assertEqual(RIGHT_PLAYBACK_SPEED, .8)
        for motor in range(1,8):
            self.assertTrue(allowed(parameter(motor,0x7017,RIGHT_PLAYBACK_SPEED)))
        self.assertFalse(allowed(parameter(8,0x7017,RIGHT_PLAYBACK_SPEED),True))

        bus=fake('right',True);bus.active=True;bus._send=Mock()
        bus.set_right_arm_speed(RIGHT_PLAYBACK_SPEED)
        bus.set_right_arm_speed(SPEED)
        self.assertEqual(
            [call.args for call in bus._send.call_args_list],
            [('right',parameter(motor,0x7017,speed))
             for speed in (RIGHT_PLAYBACK_SPEED,SPEED) for motor in range(1,8)],
        )
        with self.assertRaises(RuntimeError):bus.set_right_arm_speed(.7)

        left=fake('left');left.active=True
        with self.assertRaisesRegex(RuntimeError,'Right arm must be active'):
            left.set_right_arm_speed(RIGHT_PLAYBACK_SPEED)
        with self.assertRaisesRegex(RuntimeError,'right-arm-only'):
            left._send('left',parameter(1,0x7017,RIGHT_PLAYBACK_SPEED))
        left.sockets['left'].send.assert_not_called()

    def test_custom_right_gripper_maps_open_and_closed_into_rviz(self):
        bus=fake('right',True)
        bus.states['right',8]=(RIGHT_GRIPPER_OPEN,5,time.monotonic())
        self.assertAlmostEqual(bus.positions()['openarmx_right_finger_joint1'],.044)
        bus.states['right',8]=(RIGHT_GRIPPER_CLOSED,5,time.monotonic())
        self.assertAlmostEqual(bus.positions()['openarmx_right_finger_joint1'],0.)

    def test_full_can_queue_is_retried(self):
        import errno
        bus=fake();bus.sockets['left'].send.side_effect=[BlockingIOError(errno.ENOBUFS,'full'),BlockingIOError(errno.EAGAIN,'again'),16]
        bus.send_left(packet(4,1))
        self.assertEqual(bus.sockets['left'].send.call_count,3)

    def test_relax_every_left_motor_even_with_failure(self):
        bus=fake();bus.sockets['left'].send.side_effect=OSError('failure')
        with self.assertRaises(RuntimeError):bus.relax()
        calls=bus.sockets['left'].send.call_args_list
        self.assertEqual([FRAME.unpack(c.args[0])[0]&255 for c in calls],list(range(1,9))*3)
        bus.sockets['right'].send.assert_not_called()

    def test_initial_unrelaxed_or_missing_feedback_blocks_enable(self):
        for bad in ('enabled','missing'):
            bus=fake()
            if bad=='enabled':bus.states['left',1]=(0.,2,time.monotonic())
            else:bus.states.pop(('right',1))
            with self.assertRaises(RuntimeError):next(bus.center())
            bus.sockets['left'].send.assert_not_called()

    def test_cancel_pending_sequence_prevents_enable(self):
        bus=fake();sequence=bus.center();next(sequence);sequence.close();bus.relax()
        self.assertFalse(any((FRAME.unpack(c.args[0])[0]>>24)&31==3 for c in bus.sockets['left'].send.call_args_list))

if __name__=='__main__':unittest.main()
