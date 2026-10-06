"""XYZ input interface layered on the selected-arm goal program."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import math
import time
import tkinter as tk
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from centering.motors import SPEED,RIGHT_GRIPPER_OPEN,RIGHT_GRIPPER_CLOSED
from goal_motion.app import App as JointGoalApp,format_goal,ROOT as PROJECT_ROOT
from goal_motion.control import TOLERANCE
from safe_zone.geometry import MEMBERSHIP_BUFFER_M,Model,Zone,RIGHT_TCP
from .ik import CartesianIK,parse_xyz

GRIPPER_TOLERANCE=math.radians(.75)
GRIPPER_CLOSE_SECONDS=.75
GRIPPER_COMMAND_INTERVAL=.1

class App(JointGoalApp):
    def __init__(self,hardware,side='left',manual_finish=None):
        self.manual_finish=(side=='right') if manual_finish is None else bool(manual_finish)
        super().__init__(hardware,side,center_joint7_at_max=(side=='right'),control_gripper=(side=='right'))
        self.gripper_closed_latched=False;self.gripper_close_until=None;self.last_gripper_command=0.
        arm=side.upper()
        self.root.title(f'{arm} arm: Cartesian Goal'+('' if hardware else ' — OFFLINE PREVIEW'))
        center_note='' if side=='left' else f' • center J7={np.degrees(self.center_goal[6]):.1f}°'
        self.header.config(text=f'{arm} gripper Cartesian goal'+center_note)
        # Live testing of this right arm showed that its second joint remains
        # near center instead of following small negative targets under load.
        # Use the arm's Cartesian redundancy for the standalone right program:
        # keep J2 centered and solve with the other six joints.  The left app
        # and the separate camera app retain their existing IK behavior.
        fixed_joints={1:self.center_goal[1]} if self._uses_manual_finish() else None
        self.ik=CartesianIK(self.model,self.zone,self.lower,self.upper,SPEED,side,self.tcp_frame,
                            self.center_goal,np.zeros(7),fixed_joints=fixed_joints)
        self.target_point=None;self.target_offset=None;self.target_inside=None;self.pending_result=None
        self.active_goal_offset=None;self.motion_goal_offset=None
        self.update_executor=None;self.update_future=None;self.update_token=0
        self.update_start_joints=None;self.update_requested_offset=None
        self.update_plan_started=None
        self.update_notice=None
        if self.manual_finish:
            update_model=Model(PROJECT_ROOT/'model/openarmx.urdf')
            update_zone=Zone.load(PROJECT_ROOT/'right_zones/zone1.json',update_model.digest,RIGHT_TCP)
            self.update_ik=CartesianIK(update_model,update_zone,self.lower,self.upper,SPEED,'right',RIGHT_TCP,
                                       self.center_goal,np.zeros(7),fixed_joints=fixed_joints)
            self.update_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='cartesian-update-ik')
        self.coordinate_form=tk.Frame(self.root);self.coordinate_form.pack(fill='x',padx=20,pady=6,before=self.start_button)
        self.xyz=[]
        for column,(name,value) in enumerate((('X','0.000'),('Y','0.000'),('Z','0.050'))):
            tk.Label(self.coordinate_form,text=name+' (m)').grid(row=0,column=column*2,sticky='e',padx=(5,2))
            variable=tk.StringVar(value=value);self.xyz.append(variable)
            tk.Entry(self.coordinate_form,textvariable=variable,width=9).grid(row=0,column=column*2+1,sticky='w',padx=(0,5))
        self.preview_status=tk.StringVar()
        self.preview_label=tk.Label(self.root,textvariable=self.preview_status,font=('Monospace',10))
        self.preview_label.pack(padx=12,pady=2,before=self.start_button)
        self.xyz_status=tk.StringVar(value='Hanging zero-joint gripper position = (0, 0, 0)')
        self.xyz_label=tk.Label(self.root,textvariable=self.xyz_status,font=('Monospace',10))
        self.xyz_label.pack(padx=12,pady=4,before=self.start_button)
        for variable in self.xyz:variable.trace_add('write',self._update_preview)
        self._update_preview()
        self.start_button.config(text='RUN CARTESIAN GOAL → HOLD 2 s → CENTER → RELAX')
        self.continue_button=None;self.update_button=None;self.finish_button=None
        if side=='right':
            self.start_button.config(text='START: CENTER + OPEN GRIPPER TO −3°')
            self.isolation_label.config(text='right_zones/zone1 active • 5 mm background tolerance • original zone shown\nLeft arm and left gripper remain relaxed. Right gripper participates in this sequence.')
            self.continue_button=tk.Button(self.root,text='CONTINUE: CLOSE GRIPPER TO +7° → MOVE',font=('Sans',14,'bold'),state='disabled',command=self.continue_motion)
            self.continue_button.pack(fill='x',padx=20,pady=5,after=self.start_button)
            if self.manual_finish:
                self.update_button=tk.Button(self.root,text='UPDATE: MOVE TO CHANGED GOAL',font=('Sans',14,'bold'),state='disabled',command=self.update_motion)
                self.update_button.pack(fill='x',padx=20,pady=5,after=self.continue_button)
                self.finish_button=tk.Button(self.root,text='END: RETURN TO CENTER → RELAX',font=('Sans',14,'bold'),state='disabled',command=self.finish_motion)
                self.finish_button.pack(fill='x',padx=20,pady=5,after=self.update_button)

    def _uses_manual_finish(self):
        return self.side=='right' and getattr(self,'manual_finish',True)

    def _cancel_update(self):
        self.update_token=getattr(self,'update_token',0)+1
        future=getattr(self,'update_future',None)
        if future is not None:future[1].cancel()
        self.update_future=None;self.update_start_joints=None;self.update_requested_offset=None
        self.update_plan_started=None

    def _refresh_update_button(self):
        button=getattr(self,'update_button',None)
        if button is None:return
        changed=(self.target_offset is not None and self.active_goal_offset is not None
                 and not np.allclose(self.target_offset,self.active_goal_offset,rtol=0.,atol=1e-9))
        available=(self.hardware and self.phase=='HOLDING GOAL'
                   and getattr(self,'update_future',None) is None
                   and self.target_inside is True and changed and self.bus.fresh())
        button.config(state='normal' if available else 'disabled')

    def initial_gripper_goal(self):
        return RIGHT_GRIPPER_OPEN if self.side=='right' else None

    def centered(self,now=None):
        if self.side!='right':
            super().centered(now);return
        if abs(self.gripper_encoder()-RIGHT_GRIPPER_OPEN)>GRIPPER_TOLERANCE:
            self.status.set(f'Arm centered — opening gripper to {math.degrees(RIGHT_GRIPPER_OPEN):+.0f}°')
            return
        self.control=None;self.phase='WAITING FOR LOAD'
        self.continue_button.config(state='normal')
        self.status.set('Centered with gripper open at −3° — load the object, then press Continue')

    def continue_motion(self):
        if self.side!='right' or not self.hardware or self.phase!='WAITING FOR LOAD':return
        if not self.bus.fresh():
            self.status.set('Cannot continue: waiting for fresh encoder feedback');return
        if not self.zone.contains(self.tcp(),MEMBERSHIP_BUFFER_M):
            self.status.set('Cannot continue: right gripper TCP is outside zone1');return
        try:self.bus.set_gripper(RIGHT_GRIPPER_CLOSED)
        except Exception as e:self.fail(str(e));return
        now=time.monotonic();self.gripper_closed_latched=True;self.gripper_close_until=now+GRIPPER_CLOSE_SECONDS;self.last_gripper_command=now
        self.phase='CLOSING GRIPPER';self.continue_button.config(state='disabled')
        self.status.set('Closing gripper to +7°…')

    def extra_control(self,now):
        if self.side!='right' or not self.hardware:return
        self.continue_button.config(state='normal' if self.phase=='WAITING FOR LOAD' and self.bus.fresh() else 'disabled')
        self._refresh_update_button()
        if getattr(self,'finish_button',None) is not None:
            ready=self.phase=='HOLDING GOAL' and self.bus.fresh()
            self.finish_button.config(state='normal' if ready else 'disabled')
        if not self.bus.active:return
        target=None
        if self.gripper_closed_latched:target=RIGHT_GRIPPER_CLOSED
        elif self.phase in ('CENTERING','WAITING FOR LOAD'):target=RIGHT_GRIPPER_OPEN
        if target is not None and now-self.last_gripper_command>=GRIPPER_COMMAND_INTERVAL:
            self.bus.set_gripper(target);self.last_gripper_command=now
        if self.phase=='CLOSING GRIPPER':
            remaining=max(0.,self.gripper_close_until-now)
            self.status.set(f'Closing gripper to +7° — moving arm in {remaining:.1f} s')
            if remaining<=0.:
                self.begin_stage('MOVING TO GOAL',self.goal)
                self.status.set('Gripper closed at +7° — moving right arm to goal')
        self._poll_update()

    def complete_stage(self,now=None):
        if self._uses_manual_finish() and self.phase in ('MOVING TO GOAL','MOVING TO UPDATED GOAL'):
            now=time.monotonic() if now is None else now
            completed_update=self.phase=='MOVING TO UPDATED GOAL'
            self.phase='HOLDING GOAL';self.hold_until=None
            # The movement controller's 30-second deadline is useful while
            # traveling, but this requested hold is intentionally indefinite.
            # Keep stall detection/correction active while retiring that travel
            # deadline until End creates a fresh recenter controller.
            if self.control is not None:self.control.started=now
            motion_offset=getattr(self,'motion_goal_offset',None)
            if motion_offset is not None:
                self.active_goal_offset=np.asarray(motion_offset,dtype=float).copy()
            if getattr(self,'finish_button',None) is not None:self.finish_button.config(state='normal')
            self.update_notice=None
            self._refresh_update_button()
            label='Updated goal reached' if completed_update else 'Goal reached'
            self.status.set(label+' — holding with gripper closed; edit coordinates to Update or press End')
            return
        super().complete_stage(now)

    def hold_goal(self,error,now):
        if self._uses_manual_finish():
            self.hold_until=None
            if self.control is not None:self.control.started=now
            if getattr(self,'update_future',None) is not None:
                self.status.set('Holding current goal with gripper closed — validating the changed coordinates')
            elif getattr(self,'update_notice',None):
                self.status.set(self.update_notice)
            else:
                self.status.set('Holding Cartesian goal with gripper closed — edit coordinates to Update or press End')
            return
        super().hold_goal(error,now)

    def update_motion(self):
        if (not self._uses_manual_finish() or not self.hardware
                or self.phase!='HOLDING GOAL' or self.update_future is not None):return
        if not self.bus.fresh():
            self.status.set('Cannot update: waiting for fresh encoder feedback');return
        try:offset=parse_xyz([v.get() for v in self.xyz])
        except ValueError as e:
            self.status.set('ERROR: '+str(e));return
        self._set_preview(offset)
        if not self.target_inside:
            self.status.set('ERROR: updated coordinate is outside zone1');return
        if (self.active_goal_offset is not None
                and np.allclose(offset,self.active_goal_offset,rtol=0.,atol=1e-9)):
            self.status.set('Update unavailable: coordinates have not changed');return
        start=self.arm().copy();token=self.update_token+1;self.update_token=token
        future=self.update_executor.submit(self.update_ik.solve_update,offset.copy(),start.copy())
        self.update_future=(token,future);self.update_start_joints=start
        self.update_requested_offset=offset.copy();self.update_notice=None
        self.update_plan_started=time.monotonic();self._refresh_update_button()
        self.status.set('Holding current goal — validating changed coordinates and the update path')

    def _poll_update(self):
        pending=getattr(self,'update_future',None)
        if pending is None:return
        token,future=pending
        if not future.done():return
        plan_started=getattr(self,'update_plan_started',None)
        elapsed=max(0.,time.monotonic()-(plan_started or time.monotonic()))
        self.update_plan_started=None
        self.update_future=None
        if token!=self.update_token or self.phase!='HOLDING GOAL':
            self._refresh_update_button();return
        try:result=future.result()
        except ValueError as e:
            self.update_notice='ERROR: goal update rejected: '+str(e)
            self.status.set(self.update_notice);self._refresh_update_button();return
        except Exception as e:
            self.fail('Goal update planning failed: '+str(e));return
        if not self.bus.fresh():
            self.update_notice='Update cancelled: encoder feedback is stale; retry when fresh'
            self.status.set(self.update_notice);self._refresh_update_button();return
        try:current_offset=parse_xyz([v.get() for v in self.xyz])
        except ValueError:
            current_offset=None
        if (current_offset is None or not np.allclose(
                current_offset,self.update_requested_offset,rtol=0.,atol=1e-9)):
            self.update_notice='Update cancelled: coordinates changed during planning; press Update again'
            self.status.set(self.update_notice);self._refresh_update_button();return
        if np.max(np.abs(self.arm()-self.update_start_joints))>TOLERANCE:
            self.update_notice='Update cancelled: arm shifted while planning; wait for the hold and retry'
            self.status.set(self.update_notice);self._refresh_update_button();return
        self.goal=result.joints.copy();self.motion_goal_offset=self.update_requested_offset.copy()
        self.target_point=result.target.copy();self.target_offset=self.update_requested_offset.copy();self.target_inside=True
        self.update_notice=None
        try:self.begin_stage('MOVING TO UPDATED GOAL',self.goal)
        except Exception as e:self.fail(str(e));return
        self.status.set(f'Updated goal verified safe in {elapsed:.2f} s — moving with the gripper closed')

    def finish_motion(self):
        if (not self._uses_manual_finish() or not self.hardware
                or self.phase!='HOLDING GOAL'):return
        if not self.bus.fresh():
            self.status.set('Cannot end: waiting for fresh encoder feedback');return
        self._cancel_update()
        if getattr(self,'finish_button',None) is not None:self.finish_button.config(state='disabled')
        try:self.begin_stage('RECENTERING',self.center_goal)
        except Exception as e:self.fail(str(e));return
        self.status.set('End requested — returning to customized right center with gripper closed, then relaxing')

    def relax(self,message=None):
        self._cancel_update()
        if self.continue_button is not None:self.continue_button.config(state='disabled')
        if getattr(self,'update_button',None) is not None:self.update_button.config(state='disabled')
        if getattr(self,'finish_button',None) is not None:self.finish_button.config(state='disabled')
        super().relax(message)

    def _clear_preview(self,message):
        self.target_point=None;self.target_offset=None;self.target_inside=None
        self.preview_status.set(message);self.preview_label.config(fg='#a02020')
        self._refresh_update_button()

    def _set_preview(self,offset):
        self.target_offset=np.asarray(offset,dtype=float)
        self.target_point=self.ik.center_tcp+self.target_offset
        self.target_inside=self.zone.contains(self.target_point,MEMBERSHIP_BUFFER_M)
        state='INSIDE zone1' if self.target_inside else 'OUTSIDE zone1'
        values=', '.join(f'{axis}={value:+.4f}' for axis,value in zip('XYZ',self.target_offset))
        self.preview_status.set(f'Goal preview (m): {values} — {state}')
        self.preview_label.config(fg='#7a007a' if self.target_inside else '#c00000')
        self._refresh_update_button()

    def _update_preview(self,*_):
        self.update_notice=None
        try:offset=parse_xyz([v.get() for v in self.xyz])
        except ValueError:
            self._clear_preview('Goal preview: enter finite numeric X, Y and Z values')
            return
        self._set_preview(offset)

    def start(self):
        if self.hardware and (self.phase not in ('READY','RELAXED') or not self.bus.fresh()):return
        if self.hardware and not self.zone.contains(self.planned_tcp(self.center_goal),MEMBERSHIP_BUFFER_M):
            self.status.set(f'ERROR: configured {self.side} center is outside {self.side}_zones/zone1.json');return
        try:
            offset=parse_xyz([v.get() for v in self.xyz])
        except ValueError as e:
            self._clear_preview('Goal preview: enter finite numeric X, Y and Z values')
            self.status.set('ERROR: '+str(e));return
        self._set_preview(offset)
        try:result=self.ik.solve(offset)
        except ValueError as e:
            self.status.set('ERROR: '+str(e));return
        self.goal=result.joints;self.target_point=result.target;self.target_offset=offset
        if not self.hardware:super().start();return
        self._cancel_update();self.active_goal_offset=None;self.motion_goal_offset=offset.copy()
        self.gripper_closed_latched=False;self.gripper_close_until=None;self.last_gripper_command=0.
        if getattr(self,'update_button',None) is not None:self.update_button.config(state='disabled')
        if getattr(self,'finish_button',None) is not None:self.finish_button.config(state='disabled')
        # IK intentionally runs while the arm is disabled. It can take longer
        # than the feedback freshness window, so resume polling before enabling.
        self.pending_result=result;self.phase='IK SOLVED';self.ik_refresh_deadline=time.monotonic()+2.
        self.status.set('IK solved — waiting for fresh encoders before enabling')
        self.root.after(20,self._start_solved_goal)

    def _start_solved_goal(self):
        if self.phase!='IK SOLVED':self.pending_result=None;return
        if not self.bus.fresh():
            if time.monotonic()>=self.ik_refresh_deadline:
                self.pending_result=None;self.phase='READY';self.status.set('ERROR: fresh encoders did not return after IK');return
            self.root.after(20,self._start_solved_goal);return
        result=self.pending_result;self.pending_result=None;self.phase='READY';super().start()
        if self.phase=='SETTING UP':
            detail=f'IK error {result.error_m*1000:.2f} mm; joint goal: '+format_goal(self.goal)
            if self.side=='right':detail+='; centering and opening gripper to −3°'
            self.status.set(detail)

    def publish(self):
        relative=self.tcp()-self.ik.center_tcp
        self.xyz_status.set('Current gripper (m): '+', '.join(f'{axis}={value:+.4f}' for axis,value in zip('XYZ',relative)))
        super().publish()

    def extra_markers(self):
        if self.target_point is None:
            removed=[]
            for idx in (5,6):
                marker=self.marker(idx,Marker.SPHERE,(0.,0.,0.,0.));marker.action=Marker.DELETE;removed.append(marker)
            return removed
        color=(1.,0.,1.,1.) if self.target_inside else (1.,0.,0.,1.)
        target=self.marker(5,Marker.SPHERE,color,.025)
        target.pose.position=Point(x=float(self.target_point[0]),y=float(self.target_point[1]),z=float(self.target_point[2]))
        label=self.marker(6,Marker.TEXT_VIEW_FACING,color,.025)
        label.pose.position=Point(x=float(self.target_point[0]),y=float(self.target_point[1]),z=float(self.target_point[2]+.04))
        label.text='Cartesian goal preview' if self.target_inside else 'OUTSIDE zone1'
        return [target,label]

    def run(self):
        try:super().run()
        finally:
            self._cancel_update()
            if self.update_executor is not None:self.update_executor.shutdown(wait=False,cancel_futures=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--hardware',action='store_true');parser.add_argument('--arm',choices=('left','right'),default='left');args=parser.parse_args()
    rclpy.init(args=[])
    try:App(args.hardware,args.arm).run()
    finally:
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
