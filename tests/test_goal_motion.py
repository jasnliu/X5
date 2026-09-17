import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock
from types import SimpleNamespace
import numpy as np
from centering.motors import Motors,parameter
from safe_zone.encoder import FRAME
from safe_zone.geometry import Model,Zone,TCP,MEMBERSHIP_BUFFER_M
from goal_motion.control import GoalControl,TOLERANCE,HOLD_SECONDS,MAX_CORRECTION,CENTER,GOAL
from goal_motion.app import App
ROOT=Path(__file__).resolve().parents[1]

def bounds():
 m=Model(ROOT/'model/openarmx.urdf');j={v.get('name'):v for v in m.joints};l=[j[f'openarmx_left_joint{i}'].find('limit') for i in range(1,8)]
 return [float(v.get('lower')) for v in l],[float(v.get('upper')) for v in l]

class GoalTests(unittest.TestCase):
 def test_exact_goal(self):
  np.testing.assert_allclose(GOAL,[math.radians(45),0,0,math.radians(80),0,0,math.radians(-80)]);np.testing.assert_allclose(CENTER,np.zeros(7))

 def test_error_correction_is_small_and_converges_loaded_plant(self):
  lo,hi=bounds();q=np.zeros(7);controller=GoalControl(GOAL,lo,hi,0.)
  load=np.array([0,0,0,.10,0,0,.08])
  for n in range(1,1000):
   command,error,reached=controller.update(q,n*.02)
   self.assertLessEqual(np.max(np.abs(command-GOAL)),MAX_CORRECTION+1e-12)
   # approximate native position loop with gravity/load offset
   q+=np.clip((command-q)*2-load,-.35,.35)*.02
   if reached:break
  self.assertTrue(reached);self.assertLessEqual(np.max(np.abs(GOAL-q)),TOLERANCE)

 def test_lenient_stall_detection(self):
  lo,hi=bounds();controller=GoalControl(GOAL,lo,hi,0.);q=np.zeros(7)
  with self.assertRaisesRegex(RuntimeError,'STALL'):
   for n in range(1,200):controller.update(q,n*.02)

 def test_slow_but_real_motion_is_not_stall(self):
  lo,hi=bounds();controller=GoalControl(GOAL,lo,hi,0.);q=np.zeros(7)
  for n in range(1,200):
   q[0]+=math.radians(.45)*.02;q[3]+=math.radians(.45)*.02;q[6]-=math.radians(.45)*.02
   controller.update(q,n*.02)

 def test_zone_contains_center_goal_and_entire_nominal_path(self):
  m=Model(ROOT/'model/openarmx.urdf');z=Zone.load(ROOT/'left_zones/zone1.json',m.digest)
  for n in range(101):
   q=GOAL*n/100;p=m.transforms({f'openarmx_left_joint{i+1}':q[i] for i in range(7)})[TCP][:3,3]
   self.assertTrue(z.contains(p,MEMBERSHIP_BUFFER_M),(n,p))

 def test_runtime_zone_breach_recenters_only_while_powered(self):
  app=App.__new__(App);app.zone=Mock();app.zone.contains.return_value=False;app.tcp=Mock(return_value=np.zeros(3));app.zone_status=Mock()
  app.bus=SimpleNamespace(active=True);app.phase='MOVING TO GOAL';app.setup=object();app.begin_stage=Mock();app.status=Mock();app.center_goal=CENTER
  app.safety()
  self.assertIsNone(app.setup);name,target=app.begin_stage.call_args.args
  self.assertEqual(name,'ZONE RECENTERING');np.testing.assert_allclose(target,CENTER)
  app.begin_stage.reset_mock();app.phase='ZONE RECENTERING';app.safety();app.begin_stage.assert_not_called()
  app.bus.active=False;app.phase='MOVING TO GOAL';app.safety();app.begin_stage.assert_not_called()
  app.zone_status.set.assert_called_with('Zone: OUTSIDE')

 def test_right_zone_breach_uses_right_joint7_positive_center(self):
  center=np.array([0,0,0,0,0,0,1.4]);app=App.__new__(App);app.side='right';app.center_goal=center
  app.zone=Mock();app.zone.contains.return_value=False;app.tcp=Mock(return_value=np.zeros(3));app.zone_status=Mock()
  app.bus=SimpleNamespace(active=True);app.phase='HOLDING GOAL';app.setup=None;app.begin_stage=Mock();app.status=Mock()
  app.safety();name,target=app.begin_stage.call_args.args
  self.assertEqual(name,'ZONE RECENTERING');np.testing.assert_allclose(target,center)

 def test_stall_or_safety_failure_calls_relax(self):
  app=App.__new__(App);app.side='left';app.setup=object();app.control=object();app.bus=Mock();app.phase='MOVING';app.status=Mock();app.relax_at=None;app.begin_stage=Mock()
  app.fail('STALL: motor 4')
  app.bus.relax.assert_called_once();app.begin_stage.assert_not_called();self.assertEqual(app.phase,'FAULT');self.assertIn('STALL',app.status.set.call_args.args[0])

 def test_normal_sequence_holds_before_recentering_and_relaxing(self):
  app=App.__new__(App);app.side='left';app.center_goal=CENTER;app.goal=GOAL.copy();app.begin_stage=Mock();app.status=Mock();app.relax=Mock();app.control=object();app.hold_until=None
  app.phase='MOVING TO GOAL';app.complete_stage(10.)
  self.assertEqual(app.phase,'HOLDING GOAL');self.assertEqual(app.hold_until,10.+HOLD_SECONDS)
  app.begin_stage.assert_not_called();app.relax.assert_not_called()
  app.hold_goal(np.zeros(7),10.+HOLD_SECONDS-.01);app.begin_stage.assert_not_called()
  app.hold_goal(np.zeros(7),10.+HOLD_SECONDS)
  name,target=app.begin_stage.call_args.args
  self.assertEqual(name,'RECENTERING');np.testing.assert_allclose(target,CENTER);app.relax.assert_not_called()
  app.phase='RECENTERING';app.complete_stage()
  app.relax.assert_called_once();self.assertIn('centered',app.relax.call_args.args[0])

 def test_goal_hold_restarts_if_an_encoder_leaves_tolerance(self):
  app=App.__new__(App);app.side='left';app.center_goal=CENTER;app.begin_stage=Mock();app.status=Mock();app.hold_until=12.
  error=np.zeros(7);error[3]=TOLERANCE+1e-6
  app.hold_goal(error,11.)
  self.assertEqual(app.hold_until,11.+HOLD_SECONDS);app.begin_stage.assert_not_called()
  app.hold_goal(np.zeros(7),11.+HOLD_SECONDS-.01);app.begin_stage.assert_not_called()
  app.hold_goal(np.zeros(7),11.+HOLD_SECONDS)
  name,target=app.begin_stage.call_args.args
  self.assertEqual(name,'RECENTERING');np.testing.assert_allclose(target,CENTER)

 def test_zone_recovery_relaxes_after_center_is_reached(self):
  app=App.__new__(App);app.side='left';app.relax=Mock();app.control=object();app.phase='ZONE RECENTERING'
  app.complete_stage()
  self.assertIsNone(app.control);app.relax.assert_called_once();self.assertIn('centered',app.relax.call_args.args[0])

 def test_zone_recovery_controller_converges_to_center_before_disable(self):
  app=App.__new__(App);app.zone=Mock();app.zone.contains.return_value=False;app.tcp=Mock(return_value=np.zeros(3));app.zone_status=Mock();app.status=Mock()
  app.side='left';app.bus=Mock();app.bus.active=True;app.phase='MOVING TO GOAL';app.setup=None;app.control=None;app.last_command=0.;app.relax_at=None;app.center_goal=CENTER;app.lower,app.upper=map(np.asarray,bounds())
  app.safety();self.assertEqual(app.phase,'ZONE RECENTERING')
  np.testing.assert_allclose(app.bus.set_positions.call_args_list[0].args[0],CENTER)
  q=GOAL.copy();reached=False
  for n in range(1,2000):
   command,_,reached=app.control.update(q,app.control.started+n*.02)
   q+=np.clip((command-q)*2,-.35,.35)*.02
   if reached:break
  self.assertTrue(reached);self.assertLessEqual(np.max(np.abs(q)),TOLERANCE)
  app.complete_stage();app.bus.relax.assert_called_once();self.assertEqual(app.phase,'RELAXING')

 def test_position_commands_are_left_ids_1_to_7_and_motor_sign_is_negative(self):
  bus=Motors.__new__(Motors);bus.active=True;bus.control_side='left';bus._send=Mock()
  bus.set_positions(GOAL)
  self.assertEqual(bus._send.call_count,7)
  for i,c in enumerate(bus._send.call_args_list,1):
   self.assertEqual(c.args[0],'left');cid,_,data=FRAME.unpack(c.args[1]);self.assertEqual(cid&255,i);self.assertEqual(data[:2],b'\x16\x70')
   raw=np.frombuffer(data[4:],dtype='<f4')[0];self.assertAlmostEqual(raw,-GOAL[i-1],places=6)

 def test_position_commands_for_right_never_use_left_socket(self):
  bus=Motors.__new__(Motors);bus.active=True;bus.control_side='right';bus._send=Mock();goal=np.arange(7)*.1
  bus.set_positions(goal);self.assertEqual(bus._send.call_count,7)
  self.assertTrue(all(c.args[0]=='right' for c in bus._send.call_args_list))

 def test_encoder_panel_includes_selected_arm_gripper(self):
  app=App.__new__(App);app.side='right';app.q={f'openarmx_right_joint{i}':math.radians(i) for i in range(1,8)}
  app.bus=SimpleNamespace(states={('right',8):(math.radians(22.5),0,time.monotonic()),('left',8):(math.radians(99),0,time.monotonic())})
  text=app.encoder_text()
  self.assertIn('Right encoder degrees:',text);self.assertIn('7:    7.0',text)
  self.assertIn('Gripper (motor 8):   22.5',text);self.assertNotIn('99.0',text)

 def test_offline_encoder_panel_shows_zero_gripper(self):
  app=App.__new__(App);app.side='left';app.q={f'openarmx_left_joint{i}':0. for i in range(1,8)};app.bus=None
  self.assertIn('Gripper (motor 8):    0.0',app.encoder_text())

if __name__=='__main__':unittest.main()
