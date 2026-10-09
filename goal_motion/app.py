"""Run one selected arm through center, configured goal, recenter, and relax."""
import argparse
import math
from pathlib import Path
import signal
import time
import tkinter as tk
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from centering.motors import Motors
from safe_zone.gripper_feedback import gripper_feedback_text
from safe_zone.geometry import Model,Zone,LEFT_TCP,RIGHT_TCP,FRAME,MEMBERSHIP_BUFFER_M
from .control import GoalControl,TOLERANCE,HOLD_SECONDS,CENTER,GOAL

ROOT=Path(__file__).resolve().parents[1]
LEFT_ZONE=ROOT/'left_zones/zone1.json'
RIGHT_ZONE=ROOT/'right_zones/zone1.json'
ZONE=LEFT_ZONE
def format_goal(goal):return ', '.join(f'J{i} → {math.degrees(q):+.1f}°' for i,q in enumerate(goal,1) if abs(q)>math.radians(.05)) or 'all joints at 0°'
GOAL_LABEL=format_goal(GOAL)

class App:
    def __init__(self,hardware,side='left',center_joint7_at_max=False,control_gripper=False):
        if side not in ('left','right'):raise ValueError('Control side must be left or right')
        self.hardware=hardware;self.side=side;self.other_side='right' if side=='left' else 'left';self.control_gripper=control_gripper
        self.tcp_frame=LEFT_TCP if side=='left' else RIGHT_TCP
        zone_path=LEFT_ZONE if side=='left' else RIGHT_ZONE
        self.model=Model(ROOT/'model/openarmx.urdf');self.zone=Zone.load(zone_path,self.model.digest,self.tcp_frame)
        joints={j.get('name'):j for j in self.model.joints};limits=[joints[f'openarmx_{side}_joint{i}'].find('limit') for i in range(1,8)]
        self.lower=np.array([float(v.get('lower')) for v in limits]);self.upper=np.array([float(v.get('upper')) for v in limits])
        self.center_goal=CENTER.copy()
        if center_joint7_at_max:self.center_goal[6]=self.upper[6]
        self.goal=GOAL.copy()
        self.node=Node(f'{side}_goal_motion');self.joints=self.node.create_publisher(JointState,'/joint_states',10)
        self.markers=self.node.create_publisher(MarkerArray,'/safe_zone/markers',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        arm=side.upper()
        self.root=tk.Tk();self.root.title(f'{arm} arm: Center → Goal → Hold → Center'+('' if hardware else ' — OFFLINE PREVIEW'))
        self.bus=Motors(side,control_gripper) if hardware else None;self.setup=None;self.control=None;self.phase='READY';self.last_command=0.;self.relax_at=None;self.hold_until=None
        self.q={f'openarmx_{side}_joint{i}':0. for side in ('left','right') for i in range(1,8)}
        self.q.update({f'openarmx_{side}_finger_joint1':0. for side in ('left','right')})
        self.status=tk.StringVar(value='Waiting for live encoders' if hardware else 'Offline preview — no CAN')
        self.zone_status=tk.StringVar(value='Zone: waiting')
        self.header=tk.Label(self.root,text=f'{arm}: center all joints, then '+GOAL_LABEL,font=('Sans',14,'bold'));self.header.pack(padx=18,pady=10)
        self.isolation_label=tk.Label(self.root,text=f'{side}_zones/zone1 active • 5 mm background tolerance • original zone shown\n{self.other_side.capitalize()} arm and both grippers remain relaxed.',fg='navy');self.isolation_label.pack()
        self.start_button=tk.Button(self.root,text='RUN: CENTER → GOAL → HOLD 2 s → CENTER → RELAX',font=('Sans',14,'bold'),command=self.start)
        self.start_button.pack(fill='x',padx=20,pady=10)
        self.emergency_button=tk.Button(self.root,text='EMERGENCY RELAX',font=('Sans',16,'bold'),bg='#bd2020',fg='white',command=self.relax)
        self.emergency_button.pack(fill='x',padx=20,pady=8,ipady=10)
        tk.Label(self.root,textvariable=self.status,wraplength=610).pack(padx=15,pady=5)
        tk.Label(self.root,textvariable=self.zone_status,font=('Sans',12,'bold')).pack(pady=4)
        self.angles=tk.StringVar();tk.Label(self.root,textvariable=self.angles,font=('Monospace',10),justify='left').pack(padx=12,pady=8)
        tk.Label(self.root,text='RViz: left-drag rotates • middle-drag pans • wheel zooms').pack(pady=6)
        self.root.bind('<Escape>',lambda _:self.relax());self.root.protocol('WM_DELETE_WINDOW',self.root.quit)
        signal.signal(signal.SIGINT,lambda *_:self.root.quit());signal.signal(signal.SIGTERM,lambda *_:self.root.quit())
        self.root.after(20,self.tick)

    def arm(self):return np.array([self.q[f'openarmx_{self.side}_joint{i}'] for i in range(1,8)])
    def left(self):return self.arm()  # Backward-compatible helper name.
    def gripper_encoder(self):
        if self.bus:
            state=self.bus.states.get((self.side,8))
            if state is not None:return float(state[0])
        return 0.
    def encoder_text(self):
        joints='  '.join(f'{i}: {self.q[f"openarmx_{self.side}_joint{i}"]*180/math.pi:6.1f}' for i in range(1,8))
        feedback=getattr(self.bus,'motor8_feedback',{}).get(self.side) if self.bus else None
        return f'{self.side.capitalize()} encoder degrees:\n{joints}\n'+gripper_feedback_text(feedback)
    def tcp(self):return self.model.transforms(self.q)[self.tcp_frame][:3,3]
    def planned_tcp(self,joints):
        values={f'openarmx_{self.side}_joint{i+1}':float(joints[i]) for i in range(7)}
        return self.model.transforms(values)[self.tcp_frame][:3,3]

    def initial_gripper_goal(self):return None
    def centered(self,now=None):self.begin_stage('MOVING TO GOAL',self.goal)
    def extra_control(self,now):pass

    def start(self):
        if not self.hardware:
            for i,v in enumerate(self.goal,1):self.q[f'openarmx_{self.side}_joint{i}']=float(v)
            self.phase='PREVIEW GOAL';self.status.set('Preview only: '+format_goal(self.goal));return
        if self.phase not in ('READY','RELAXED') or not self.bus.fresh():return
        if not self.zone.contains(self.planned_tcp(self.center_goal),MEMBERSHIP_BUFFER_M):
            self.status.set(f'Cannot start: configured {self.side} center is outside zone1');return
        if not self.zone.contains(self.tcp(),MEMBERSHIP_BUFFER_M):
            self.status.set(f'Cannot start: current {self.side} TCP is outside zone1');return
        self.setup=self.bus.center(self.center_goal,self.initial_gripper_goal());self.phase='SETTING UP';self.status.set(f'Centering {self.side} motors 1–7…')

    def begin_stage(self,name,desired):
        self.hold_until=None
        self.control=GoalControl(desired,self.lower,self.upper,time.monotonic());self.phase=name
        self.bus.set_positions(desired);self.last_command=0.;self.status.set(name.replace('_',' '))

    def complete_stage(self,now=None):
        if self.phase=='CENTERING':self.centered(now)
        elif self.phase=='MOVING TO GOAL':
            now=time.monotonic() if now is None else now
            self.phase='HOLDING GOAL';self.hold_until=now+HOLD_SECONDS
            self.status.set(f'Goal reached — holding for {HOLD_SECONDS:.1f} seconds')
        elif self.phase in ('RECENTERING','ZONE RECENTERING'):
            self.control=None;self.relax(f'Sequence complete: centered; {self.side} motors disabled')

    def hold_goal(self,error,now):
        # Require two continuous seconds inside the same joint tolerance used
        # to declare the goal reached.  Normal correction commands continue.
        if np.max(np.abs(error))>TOLERANCE:self.hold_until=now+HOLD_SECONDS
        remaining=max(0.,self.hold_until-now)
        self.status.set(f'Goal reached — holding for {remaining:.1f} seconds')
        if remaining<=0.:
            self.begin_stage('RECENTERING',self.center_goal)
            self.status.set(f'Hold complete — returning {self.side} arm to center')

    def relax(self,message=None):
        if message is None:message=f'{self.side.capitalize()} arm relaxed'
        self.setup=None;self.control=None;self.hold_until=None
        if self.bus:
            try:self.bus.relax();self.phase='RELAXING';self.relax_at=time.monotonic();self.status.set(message+' — confirming disabled feedback')
            except Exception as e:self.phase='FAULT';self.status.set(str(e))
        else:self.phase='RELAXED';self.status.set(message)

    def fail(self,message):
        self.relax('STOPPED: '+message+f'; {self.side} motors disabled')
        self.phase='FAULT'

    def safety(self):
        inside=self.zone.contains(self.tcp(),MEMBERSHIP_BUFFER_M)
        self.zone_status.set('Zone: INSIDE' if inside else 'Zone: OUTSIDE')
        if self.bus and self.bus.active and not inside and self.phase!='ZONE RECENTERING':
            self.setup=None
            self.begin_stage('ZONE RECENTERING',self.center_goal)
            self.status.set('OUTSIDE zone1 — returning to center, then relaxing')

    def marker(self,idx,kind,color,scale=.01):
        m=Marker();m.header.frame_id=FRAME;m.header.stamp=self.node.get_clock().now().to_msg();m.ns=f'{self.side}_goal_motion';m.id=idx;m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=scale;m.color.r,m.color.g,m.color.b,m.color.a=color;return m

    def extra_markers(self):return []

    def publish(self):
        msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg();msg.name=list(self.q);msg.position=list(self.q.values());self.joints.publish(msg)
        self.zone.rebuild();h=self.zone.hull
        points=self.marker(0,Marker.POINTS,(1.,.65,0.,1.));points.points=[Point(x=p[0],y=p[1],z=p[2]) for p in self.zone.points]
        mesh=self.marker(1,Marker.TRIANGLE_LIST,(0.,.7,1.,.20),1.);edges=self.marker(2,Marker.LINE_LIST,(0.,.8,1.,1.),.002)
        for tri in h.simplices:
            ps=[Point(x=float(h.points[i,0]),y=float(h.points[i,1]),z=float(h.points[i,2])) for i in tri]
            mesh.points.extend(ps+ps[::-1]);edges.points.extend([ps[0],ps[1],ps[1],ps[2],ps[2],ps[0]])
        tip=self.marker(3,Marker.SPHERE,(0.,1.,0.,1.) if self.zone.contains(self.tcp(),MEMBERSHIP_BUFFER_M) else (1.,0.,0.,1.),.025)
        p=self.tcp();tip.pose.position=Point(x=float(p[0]),y=float(p[1]),z=float(p[2]))
        label=self.marker(4,Marker.TEXT_VIEW_FACING,(1.,.6,0.,1.),.04);label.pose.position.z=1.5;label.text=self.phase+' — '+self.control_description()
        self.markers.publish(MarkerArray(markers=[points,mesh,edges,tip,label]+self.extra_markers()))

    def relaxed_feedback_sides(self):
        return (self.side,)

    def control_description(self):
        return f'{self.side.upper()} ONLY'

    def tick(self):
        try:
            if self.bus:
                self.bus.poll()
                if self.relax_at is not None and self.bus.fresh():
                    if all(self.bus.states[side,i][1]==0 and self.bus.states[side,i][2]>self.relax_at for side in self.relaxed_feedback_sides() for i in range(1,9)):
                        self.relax_at=None;self.phase='RELAXED';self.status.set(' + '.join(self.relaxed_feedback_sides()).capitalize()+' motors disabled')
                    elif time.monotonic()-self.relax_at>1.:
                        self.relax_at=None;self.phase='FAULT';self.status.set('Disable not confirmed — use physical power cutoff')
                if self.bus.fresh():
                    self.q=self.bus.positions()
                    if self.phase=='READY' and self.status.get()=='Waiting for live encoders':self.status.set('Ready — press Run')
                self.safety()
                if self.setup is not None:
                    try:next(self.setup)
                    except StopIteration:self.setup=None;self.begin_stage('CENTERING',self.center_goal)
                elif self.control is not None:
                    now=time.monotonic();command,error,reached=self.control.update(self.arm(),now)
                    if now-self.last_command>=.2:
                        self.bus.set_positions(command);self.last_command=now
                    if self.phase=='HOLDING GOAL':self.hold_goal(error,now)
                    elif reached:self.complete_stage(now)
                self.extra_control(time.monotonic())
                self.start_button.config(state='normal' if self.phase in ('READY','RELAXED') and self.bus.fresh() else 'disabled')
            else:self.safety()
            self.publish()
            self.angles.set(self.encoder_text())
        except Exception as e:self.fail(str(e))
        rclpy.spin_once(self.node,timeout_sec=0);self.root.after(20,self.tick)

    def run(self):
        try:self.root.mainloop()
        finally:
            if self.bus:
                try:
                    if self.bus.active:self.bus.relax()
                finally:self.bus.close()
            self.root.destroy();self.node.destroy_node()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--hardware',action='store_true');args=parser.parse_args()
    rclpy.init(args=[])
    try:App(args.hardware).run()
    finally:
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
