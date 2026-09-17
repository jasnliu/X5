from pathlib import Path
import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
import numpy as np
from visualization_msgs.msg import Marker
from centering.motors import SPEED,RIGHT_GRIPPER_OPEN,RIGHT_GRIPPER_CLOSED
from safe_zone.geometry import Model,Zone,RIGHT_TCP,MEMBERSHIP_BUFFER_M
from cartesian_goal.ik import CartesianIK,IKResult,IK_TOLERANCE_M,parse_xyz
from cartesian_goal.app import App,GRIPPER_CLOSE_SECONDS

ROOT=Path(__file__).resolve().parents[1]

class CartesianGoalTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.model=Model(ROOT/'model/openarmx.urdf');cls.zone=Zone.load(ROOT/'left_zones/zone1.json',cls.model.digest)
  joints={j.get('name'):j for j in cls.model.joints};limits=[joints[f'openarmx_left_joint{i}'].find('limit') for i in range(1,8)]
  cls.lower=np.array([float(v.get('lower')) for v in limits]);cls.upper=np.array([float(v.get('upper')) for v in limits])
  cls.ik=CartesianIK(cls.model,cls.zone,cls.lower,cls.upper,SPEED)

 def test_center_is_cartesian_origin(self):
  result=self.ik.solve([0,0,0]);np.testing.assert_allclose(result.joints,np.zeros(7));np.testing.assert_allclose(result.target,self.ik.center_tcp)

 def test_straight_up_five_centimeters_has_safe_accurate_joint_solution(self):
  result=self.ik.solve([0,0,.05])
  self.assertLessEqual(result.error_m,IK_TOLERANCE_M)
  np.testing.assert_allclose(self.ik.position(result.joints)-self.ik.center_tcp,[0,0,.05],atol=IK_TOLERANCE_M)
  self.assertTrue(np.all(result.joints>=self.lower));self.assertTrue(np.all(result.joints<=self.upper))
  self.assertTrue(self.ik.path_inside(result.joints));self.assertTrue(self.zone.contains(result.target,MEMBERSHIP_BUFFER_M))

 def test_right_cartesian_origin_stays_at_zero_joints_while_home_uses_positive_j7_limit(self):
  joints={j.get('name'):j for j in self.model.joints};limits=[joints[f'openarmx_right_joint{i}'].find('limit') for i in range(1,8)]
  lower=np.array([float(v.get('lower')) for v in limits]);upper=np.array([float(v.get('upper')) for v in limits])
  home=np.zeros(7);home[6]=upper[6];zone=Mock();zone.contains.return_value=True
  ik=CartesianIK(self.model,zone,lower,upper,SPEED,'right',RIGHT_TCP,home,np.zeros(7))
  expected=self.model.transforms({})[RIGHT_TCP][:3,3]
  np.testing.assert_allclose(ik.origin_tcp,expected);np.testing.assert_allclose(ik.home,home)
  result=ik.solve([0,0,0]);np.testing.assert_allclose(result.joints,np.zeros(7));np.testing.assert_allclose(result.target,expected)
  up=ik.solve([0,0,.05]);self.assertLessEqual(up.error_m,IK_TOLERANCE_M)
  np.testing.assert_allclose(ik.position(up.joints)-ik.origin_tcp,[0,0,.05],atol=IK_TOLERANCE_M)

 def test_outside_coordinate_is_rejected_before_motion(self):
  with self.assertRaisesRegex(ValueError,'outside zone1'):self.ik.solve([2,0,0])

 def test_bad_coordinate_text_is_rejected(self):
  for values in (['bad','0','0'],['nan','0','0'],['0','inf','0']):
   with self.assertRaises(ValueError):parse_xyz(values)

 def preview_app(self,values,inside=True):
  app=App.__new__(App);app.xyz=[Mock(),Mock(),Mock()]
  for v,text in zip(app.xyz,values):v.get.return_value=text
  app.ik=SimpleNamespace(center_tcp=np.array([.1,.2,.3]));app.zone=Mock();app.zone.contains.return_value=inside
  app.preview_status=Mock();app.preview_label=Mock();app.target_point=None;app.target_offset=None;app.target_inside=None
  return app

 def test_live_goal_preview_updates_without_solving_or_moving(self):
  app=self.preview_app(('.01','-.02','.03'))
  app._update_preview()
  np.testing.assert_allclose(app.target_offset,[.01,-.02,.03]);np.testing.assert_allclose(app.target_point,[.11,.18,.33])
  self.assertTrue(app.target_inside);self.assertIn('INSIDE zone1',app.preview_status.set.call_args.args[0])
  app.zone.contains.assert_called_once_with(app.target_point,MEMBERSHIP_BUFFER_M)

 def test_outside_and_invalid_live_previews(self):
  app=self.preview_app(('2','0','0'),inside=False);app._update_preview()
  np.testing.assert_allclose(app.target_point,[2.1,.2,.3]);self.assertFalse(app.target_inside)
  self.assertIn('OUTSIDE zone1',app.preview_status.set.call_args.args[0])
  app.xyz[0].get.return_value='not-a-number';app._update_preview()
  self.assertIsNone(app.target_point);self.assertIsNone(app.target_inside)

 def test_invalid_preview_publishes_marker_deletions(self):
  app=App.__new__(App);app.target_point=None;app.marker=Mock(side_effect=lambda idx,*_:SimpleNamespace(id=idx,action=None))
  markers=app.extra_markers();self.assertEqual([m.id for m in markers],[5,6])
  self.assertTrue(all(m.action==Marker.DELETE for m in markers))

 def test_gui_solver_error_does_not_start_motor_setup(self):
  app=App.__new__(App);app.hardware=True;app.side='left';app.center_goal=np.zeros(7);app.planned_tcp=Mock(return_value=np.zeros(3));app.phase='READY';app.bus=Mock();app.bus.fresh.return_value=True
  app.xyz=[Mock(),Mock(),Mock()]
  for v,text in zip(app.xyz,('2','0','0')):v.get.return_value=text
  app.ik=Mock();app.ik.center_tcp=np.zeros(3);app.ik.solve.side_effect=ValueError('Requested coordinate is outside zone1')
  app.zone=Mock();app.zone.contains.side_effect=[True,False];app.preview_status=Mock();app.preview_label=Mock();app.status=Mock();app.target_point=np.ones(3)
  app.start();app.bus.center.assert_not_called();np.testing.assert_allclose(app.target_point,[2,0,0]);self.assertFalse(app.target_inside)
  self.assertIn('outside zone1',app.status.set.call_args.args[0])

 def test_hardware_waits_for_fresh_feedback_after_ik(self):
  app=App.__new__(App);app.hardware=True;app.side='left';app.center_goal=np.zeros(7);app.planned_tcp=Mock(return_value=np.zeros(3));app.phase='READY';app.bus=Mock();app.bus.fresh.return_value=True
  app.xyz=[Mock(),Mock(),Mock()]
  for v,text in zip(app.xyz,('0','0','.05')):v.get.return_value=text
  result=IKResult(np.ones(7)*.1,np.array([0.,0.,.05]),0.);app.ik=Mock();app.ik.center_tcp=np.zeros(3);app.ik.solve.return_value=result
  app.zone=Mock();app.zone.contains.return_value=True;app.preview_status=Mock();app.preview_label=Mock()
  app.status=Mock();app.root=Mock();app.target_point=None;app.target_offset=None;app.target_inside=None;app.pending_result=None
  app.start();self.assertEqual(app.phase,'IK SOLVED');app.bus.center.assert_not_called();app.root.after.assert_called_once()
  app.zone=Mock();app.zone.contains.return_value=True;app.tcp=Mock(return_value=np.zeros(3));app.bus.center.return_value=iter(())
  app._start_solved_goal();self.assertEqual(app.phase,'SETTING UP');app.bus.center.assert_called_once()

 def test_right_center_outside_zone_is_rejected_before_ik_or_motor_setup(self):
  app=App.__new__(App);app.hardware=True;app.side='right';app.center_goal=np.array([0,0,0,0,0,0,1.4])
  app.phase='READY';app.bus=Mock();app.bus.fresh.return_value=True;app.zone=Mock();app.zone.contains.return_value=False
  app.planned_tcp=Mock(return_value=np.zeros(3));app.status=Mock();app.ik=Mock()
  app.start();app.ik.solve.assert_not_called();app.bus.center.assert_not_called()
  self.assertIn('right center is outside',app.status.set.call_args.args[0])

 def test_left_workflow_still_moves_immediately_after_centering(self):
  app=App.__new__(App);app.side='left';app.goal=np.ones(7);app.begin_stage=Mock()
  app.centered(10.)
  app.begin_stage.assert_called_once_with('MOVING TO GOAL',app.goal)

 def test_right_center_pauses_with_gripper_open_until_continue(self):
  app=App.__new__(App);app.side='right';app.control=object();app.phase='CENTERING';app.gripper_encoder=Mock(return_value=RIGHT_GRIPPER_OPEN)
  app.continue_button=Mock();app.status=Mock()
  app.centered(10.)
  self.assertEqual(app.phase,'WAITING FOR LOAD');self.assertIsNone(app.control)
  app.continue_button.config.assert_called_with(state='normal')
  self.assertIn('load the object',app.status.set.call_args.args[0])

 def test_right_continue_closes_before_starting_arm_goal(self):
  app=App.__new__(App);app.side='right';app.hardware=True;app.phase='WAITING FOR LOAD';app.bus=Mock();app.bus.fresh.return_value=True
  app.zone=Mock();app.zone.contains.return_value=True;app.tcp=Mock(return_value=np.zeros(3));app.continue_button=Mock();app.status=Mock()
  app.gripper_closed_latched=False;app.gripper_close_until=None;app.last_gripper_command=0.;app.goal=np.ones(7);app.begin_stage=Mock()
  with patch('cartesian_goal.app.time.monotonic',return_value=20.):app.continue_motion()
  app.bus.set_gripper.assert_called_once_with(RIGHT_GRIPPER_CLOSED)
  self.assertEqual(app.phase,'CLOSING GRIPPER');self.assertTrue(app.gripper_closed_latched)
  self.assertEqual(app.gripper_close_until,20.+GRIPPER_CLOSE_SECONDS);app.begin_stage.assert_not_called()
  app.extra_control(20.+GRIPPER_CLOSE_SECONDS-.01);app.begin_stage.assert_not_called()
  app.extra_control(20.+GRIPPER_CLOSE_SECONDS)
  app.begin_stage.assert_called_once_with('MOVING TO GOAL',app.goal)

 def test_right_gripper_closed_command_is_held_during_goal_and_return(self):
  app=App.__new__(App);app.side='right';app.hardware=True;app.bus=Mock();app.bus.active=True;app.bus.fresh.return_value=True
  app.continue_button=Mock();app.gripper_closed_latched=True;app.gripper_close_until=0.;app.last_gripper_command=0.;app.status=Mock();app.goal=np.zeros(7);app.begin_stage=Mock()
  for phase,now in (('MOVING TO GOAL',1.),('HOLDING GOAL',2.),('RECENTERING',3.),('ZONE RECENTERING',4.)):
   app.phase=phase;app.extra_control(now)
  self.assertEqual(app.bus.set_gripper.call_count,4)
  self.assertTrue(all(call.args==(RIGHT_GRIPPER_CLOSED,) for call in app.bus.set_gripper.call_args_list))

if __name__=='__main__':unittest.main()
