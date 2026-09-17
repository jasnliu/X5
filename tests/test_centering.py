"""Direct zero-command tests; mocked sockets only."""
import math
import struct
import time
import unittest
from unittest.mock import Mock
from centering.motors import Motors,packet,parameter,allowed,CURRENT,SPEED,RIGHT_GRIPPER_OPEN,RIGHT_GRIPPER_CLOSED
from safe_zone.encoder import FRAME,EFF,RIGHT_JOINT5_ZERO_OFFSET

def fake(side='left',gripper=False):
    bus=Motors.__new__(Motors)
    bus.control_side=side;bus.control_gripper=gripper
    bus.states={(side,i):(0.,0,time.monotonic()) for side in ('left','right') for i in range(1,9)}
    bus.modes={};bus.active=False;bus.sockets={'left':Mock(),'right':Mock()}
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
