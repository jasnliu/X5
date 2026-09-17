"""Two buttons: send zero to the left joints, or disable the left motors."""
import argparse
import signal
import tkinter as tk
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .motors import Motors

class App:
    def __init__(self,hardware):
        self.node=Node('left_centering');self.publisher=self.node.create_publisher(JointState,'/joint_states',10)
        self.root=tk.Tk();self.root.title('LEFT arm: Center / Relax'+('' if hardware else ' — OFFLINE PREVIEW'))
        self.bus=None;self.move=None;self.relax_at=None
        self.q={f'openarmx_{side}_joint{i}':(-.2 if side=='left' and i!=4 else .2 if side=='left' else 0.) for side in ('left','right') for i in range(1,8)}
        self.q.update({f'openarmx_{side}_finger_joint1':0. for side in ('left','right')})
        self.status=tk.StringVar(value='Waiting for encoders' if hardware else 'Offline preview — no CAN')
        tk.Label(self.root,text='LEFT motors 1–7 → 0 radians\nGripper and right arm stay relaxed.',font=('Sans',14)).pack(padx=20,pady=12)
        tk.Label(self.root,text='can1 must be LEFT. Keep the path clear.\nRelax disables torque; it does NOT cut electrical power.\nSupport the arm when relaxing. No collision / safe-zone checks.',fg='darkred').pack(padx=15,pady=8)
        self.center_button=tk.Button(self.root,text='CENTER LEFT ARM',command=self.center,font=('Sans',14,'bold'))
        self.center_button.pack(fill='x',padx=20,pady=10)
        tk.Button(self.root,text='EMERGENCY RELAX',command=self.relax,bg='#bd2020',fg='white',font=('Sans',16,'bold')).pack(fill='x',padx=20,pady=10,ipady=12)
        tk.Label(self.root,textvariable=self.status,wraplength=520).pack(padx=15,pady=10)
        self.angles=tk.StringVar();tk.Label(self.root,textvariable=self.angles,font=('Monospace',10)).pack(padx=15,pady=8)
        tk.Label(self.root,text='RViz: left-drag rotates • wheel zooms').pack(pady=8)
        if hardware:
            try:self.bus=Motors()
            except Exception:
                self.root.destroy();self.node.destroy_node();raise
        self.root.bind('<Escape>',lambda _:self.relax())
        self.root.protocol('WM_DELETE_WINDOW',self.root.quit)
        signal.signal(signal.SIGINT,lambda *_:self.root.quit())
        signal.signal(signal.SIGTERM,lambda *_:self.root.quit())
        self.root.after(20,self.tick)

    def center(self):
        if self.move is not None or self.relax_at is not None:return
        if self.bus:
            if not self.bus.fresh() or self.bus.active:
                self.status.set('Wait for fresh encoders; relax before centering again.');return
            self.move=self.bus.center()
            self.status.set('Sending zero position to left motors 1–7…')
        else:
            for i in range(1,8):self.q[f'openarmx_left_joint{i}']=0.
            self.status.set('Preview: left motors at zero')

    def relax(self):
        self.move=None  # Cancel queued enables/position writes FIRST.
        if self.bus:
            try:
                self.bus.relax();self.relax_at=time.monotonic()
                self.status.set('Disable sent — waiting for relaxed feedback')
            except Exception as e:self.relax_at=None;self.status.set(str(e))
        else:self.status.set('Preview: left motors relaxed')

    def tick(self):
        try:
            if self.bus:
                self.bus.poll()
                if self.relax_at is not None:
                    if all(self.bus.states.get(('left',i),(0,1,0))[1]==0 and self.bus.states.get(('left',i),(0,1,0))[2]>self.relax_at for i in range(1,9)):
                        self.status.set('Left arm relaxed');self.relax_at=None
                    elif time.monotonic()-self.relax_at>1:
                        self.status.set('Relax not confirmed — use physical power cutoff');self.relax_at=None
                if self.move is not None:
                    try:next(self.move)
                    except StopIteration:self.move=None;self.status.set('Zero target sent. Press Relax to disable.')
                if self.bus.fresh():
                    self.q=self.bus.positions()
                    if self.status.get()=='Waiting for encoders':self.status.set('Ready — press Center Left Arm')
                self.center_button.config(state='normal' if self.bus.fresh() and not self.bus.active and self.move is None and self.relax_at is None else 'disabled')
            if not self.bus or self.bus.fresh():
                msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
                msg.name=list(self.q);msg.position=list(self.q.values());self.publisher.publish(msg)
                self.angles.set('Left motor degrees:\n'+'  '.join(f'{i}: {self.q[f"openarmx_left_joint{i}"]*180/3.14159265:.1f}' for i in range(1,8)))
        except Exception as e:
            self.move=None
            extra=''
            if self.bus and self.bus.active:
                try:self.bus.relax();extra='; disable sent'
                except Exception as stop:extra='; '+str(stop)
            self.status.set(str(e)+extra);self.center_button.config(state='disabled')
        rclpy.spin_once(self.node,timeout_sec=0)
        self.root.after(20,self.tick)

    def run(self):
        try:self.root.mainloop()
        finally:
            if self.bus:
                try:
                    if self.bus.active:self.bus.relax()
                except Exception as e:print(str(e),flush=True)
                finally:self.bus.close()
            self.root.destroy();self.node.destroy_node()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--hardware',action='store_true');args=parser.parse_args()
    rclpy.init(args=[])
    try:App(args.hardware).run()
    finally:
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
