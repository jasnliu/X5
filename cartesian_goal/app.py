"""XYZ input interface layered on the selected-arm goal program."""
import argparse
import math
import time
import tkinter as tk
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from centering.motors import SPEED,RIGHT_GRIPPER_OPEN,RIGHT_GRIPPER_CLOSED
from goal_motion.app import App as JointGoalApp,format_goal
from safe_zone.geometry import MEMBERSHIP_BUFFER_M
from .ik import CartesianIK,parse_xyz

GRIPPER_TOLERANCE=math.radians(.75)
GRIPPER_CLOSE_SECONDS=.75
GRIPPER_COMMAND_INTERVAL=.1

class App(JointGoalApp):
    def __init__(self,hardware,side='left'):
        super().__init__(hardware,side,center_joint7_at_max=(side=='right'),control_gripper=(side=='right'))
        self.gripper_closed_latched=False;self.gripper_close_until=None;self.last_gripper_command=0.
        arm=side.upper()
        self.root.title(f'{arm} arm: Cartesian Goal'+('' if hardware else ' — OFFLINE PREVIEW'))
        center_note='' if side=='left' else f' • center J7={np.degrees(self.center_goal[6]):.1f}°'
        self.header.config(text=f'{arm} gripper Cartesian goal'+center_note)
        self.ik=CartesianIK(self.model,self.zone,self.lower,self.upper,SPEED,side,self.tcp_frame,self.center_goal,np.zeros(7))
        self.target_point=None;self.target_offset=None;self.target_inside=None;self.pending_result=None
        form=tk.Frame(self.root);form.pack(fill='x',padx=20,pady=6,before=self.start_button)
        self.xyz=[]
        for column,(name,value) in enumerate((('X','0.000'),('Y','0.000'),('Z','0.050'))):
            tk.Label(form,text=name+' (m)').grid(row=0,column=column*2,sticky='e',padx=(5,2))
            variable=tk.StringVar(value=value);self.xyz.append(variable)
            tk.Entry(form,textvariable=variable,width=9).grid(row=0,column=column*2+1,sticky='w',padx=(0,5))
        self.preview_status=tk.StringVar()
        self.preview_label=tk.Label(self.root,textvariable=self.preview_status,font=('Monospace',10))
        self.preview_label.pack(padx=12,pady=2,before=self.start_button)
        self.xyz_status=tk.StringVar(value='Hanging zero-joint gripper position = (0, 0, 0)')
        tk.Label(self.root,textvariable=self.xyz_status,font=('Monospace',10)).pack(padx=12,pady=4,before=self.start_button)
        for variable in self.xyz:variable.trace_add('write',self._update_preview)
        self._update_preview()
        self.start_button.config(text='RUN CARTESIAN GOAL → HOLD 2 s → CENTER → RELAX')
        self.continue_button=None
        if side=='right':
            self.start_button.config(text='START: CENTER + OPEN GRIPPER TO −3°')
            self.isolation_label.config(text='right_zones/zone1 active • 5 mm background tolerance • original zone shown\nLeft arm and left gripper remain relaxed. Right gripper participates in this sequence.')
            self.continue_button=tk.Button(self.root,text='CONTINUE: CLOSE GRIPPER TO +7° → MOVE',font=('Sans',14,'bold'),state='disabled',command=self.continue_motion)
            self.continue_button.pack(fill='x',padx=20,pady=5,after=self.start_button)

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

    def relax(self,message=None):
        if self.continue_button is not None:self.continue_button.config(state='disabled')
        super().relax(message)

    def _clear_preview(self,message):
        self.target_point=None;self.target_offset=None;self.target_inside=None
        self.preview_status.set(message);self.preview_label.config(fg='#a02020')

    def _set_preview(self,offset):
        self.target_offset=np.asarray(offset,dtype=float)
        self.target_point=self.ik.center_tcp+self.target_offset
        self.target_inside=self.zone.contains(self.target_point,MEMBERSHIP_BUFFER_M)
        state='INSIDE zone1' if self.target_inside else 'OUTSIDE zone1'
        values=', '.join(f'{axis}={value:+.4f}' for axis,value in zip('XYZ',self.target_offset))
        self.preview_status.set(f'Goal preview (m): {values} — {state}')
        self.preview_label.config(fg='#7a007a' if self.target_inside else '#c00000')

    def _update_preview(self,*_):
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
        self.gripper_closed_latched=False;self.gripper_close_until=None;self.last_gripper_command=0.
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

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--hardware',action='store_true');parser.add_argument('--arm',choices=('left','right'),default='left');args=parser.parse_args()
    rclpy.init(args=[])
    try:App(args.hardware,args.arm).run()
    finally:
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
