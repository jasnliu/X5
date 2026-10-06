"""Standalone Tk control window and ROS encoder/simulation visualization."""
import json
import math
from pathlib import Path
import signal
import time
import tkinter as tk
from tkinter import ttk
from .cli import options_for, request_for
from .config import Rules, TARGET_DEG, ZONE_DEG, center_pose
from .methods import METHODS
from .runtime import Session


class App:
    def __init__(self,args,config,session_factory=Session,ros=True):
        self.args,self.config=args,config
        self.session_factory=session_factory;self.session=None
        self.prepared=False;self.busy=False;self.paused=False;self.closing=False;self.fault=False;self.stopping=False
        self.positions=[0.]*7;self.anchor=center_pose()[6]
        self.node=None;self.ros=None
        if ros:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            rclpy.init(args=[]);self.ros=rclpy;self.JointState=JointState
            self.node=Node('strike_lab_view');self.publisher=self.node.create_publisher(JointState,'/joint_states',10)
        self.root=tk.Tk();self.root.title('J7 Strike Experiment — '+('PHYSICAL HARDWARE' if args.hardware else 'SIMULATION / NO HARDWARE'))
        self.root.geometry('790x920');self.root.minsize(700,780)
        self.root.configure(bg='#f0f2f5')
        self.status=tk.StringVar(value='Not connected. Prepare closes the gripper and centers the right arm.')
        self.path=tk.StringVar(value='A separate immutable results session is created when prepared.')
        self.angles=tk.StringVar(value='Right arm: waiting');self.method=tk.StringVar(value=args.method)
        self.reps=tk.StringVar(value=str(args.repetitions));self.stage=tk.StringVar(value=args.stage)
        self.metric=tk.StringVar(value='No trials yet. All methods target exactly 10.000°.')
        mode='PHYSICAL RIGHT ARM — EXPLICIT HARDWARE MODE' if args.hardware else 'SYNTHETIC DYNAMICS — NO CAN / CAMERA / AUDIO'
        tk.Label(self.root,text=mode,font=('Sans',15,'bold'),fg='#a51919' if args.hardware else '#19476a',bg='#f0f2f5').pack(pady=(12,4))
        rules=Rules(**config.get('rules',{}))
        tk.Label(self.root,text=f'Close gripper → custom right center → J7 only: 10° down / return\n'
                 f'Virtual zone >9° • acceptance 10° ± {rules.acceptance_deg:g}° • left arm untouched',
                 font=('Sans',11),bg='#f0f2f5').pack(pady=5)
        controls=ttk.Frame(self.root);controls.pack(fill='x',padx=16,pady=5)
        ttk.Label(controls,text='Method mode:').grid(row=0,column=0,sticky='w')
        combo=ttk.Combobox(controls,textvariable=self.method,values=['all',*METHODS],state='readonly',width=25)
        combo.grid(row=0,column=1,sticky='ew',padx=6);combo.bind('<<ComboboxSelected>>',lambda _:self.load_parameters())
        ttk.Label(controls,text='Repetitions:').grid(row=0,column=2)
        ttk.Entry(controls,textvariable=self.reps,width=6).grid(row=0,column=3,padx=6)
        controls.columnconfigure(1,weight=1)
        self.description=tk.StringVar();ttk.Label(self.root,textvariable=self.description,wraplength=750).pack(padx=18,anchor='w')
        ttk.Label(self.root,text='Method parameters (JSON; target is fixed, not an adjustable parameter):').pack(padx=18,pady=(8,0),anchor='w')
        self.params=tk.Text(self.root,height=3,font=('Monospace',10),wrap='word');self.params.pack(fill='x',padx=18,pady=4)
        self.load_parameters()
        buttons=ttk.Frame(self.root);buttons.pack(fill='x',padx=16,pady=4)
        self.prepare_button=ttk.Button(buttons,text='1. CLOSE GRIPPER + CENTER',command=self.prepare);self.prepare_button.grid(row=0,column=0,sticky='ew',padx=3,pady=3)
        self.run_button=ttk.Button(buttons,text='2. RUN SELECTED METHOD',command=self.run_selected);self.run_button.grid(row=0,column=1,sticky='ew',padx=3,pady=3)
        self.all_button=ttk.Button(buttons,text='COMPARE ALL DEFAULTS',command=lambda:self.run_selected(all_methods=True));self.all_button.grid(row=1,column=0,sticky='ew',padx=3,pady=3)
        self.campaign_button=ttk.Button(buttons,text='RUN TUNING CAMPAIGN',command=self.campaign);self.campaign_button.grid(row=1,column=1,sticky='ew',padx=3,pady=3)
        for i in range(2):buttons.columnconfigure(i,weight=1)
        row=ttk.Frame(self.root);row.pack(fill='x',padx=18,pady=3)
        ttk.Label(row,text='Campaign through:').pack(side='left')
        ttk.Combobox(row,textvariable=self.stage,values=['screen','refine','validate','all'],state='readonly',width=10).pack(side='left',padx=8)
        self.pause_button=ttk.Button(row,text='PAUSE AFTER CURRENT',command=self.toggle_pause);self.pause_button.pack(side='left',padx=8)
        self.finish_button=ttk.Button(row,text='FINISH BATCH + RELAX',command=self.finish);self.finish_button.pack(side='left')
        self.stop_button=tk.Button(self.root,text='STOP + RELAX RIGHT ARM',command=self.stop,bg='#b91c1c',fg='white',font=('Sans',13,'bold'))
        self.stop_button.pack(fill='x',padx=18,pady=7,ipady=4)
        tk.Label(self.root,textvariable=self.status,bg='#f0f2f5',wraplength=750,font=('Sans',11,'bold')).pack(padx=18,pady=4)
        self.canvas=tk.Canvas(self.root,height=175,bg='white',highlightthickness=1,highlightbackground='#cbd5e1');self.canvas.pack(fill='x',padx=18,pady=4)
        self.points=[];self.plot_started=None;self.current_trial=None;self.redraw()
        ttk.Label(self.root,text='Live depth plot is decimated. Scores use the full-rate timestamped trace.').pack()
        ttk.Label(self.root,textvariable=self.metric,wraplength=750).pack(padx=18,pady=5)
        cols=('method','depth','zone','cycle','result')
        self.table=ttk.Treeview(self.root,columns=cols,show='headings',height=5)
        for name,title,width in zip(cols,('Method','Peak °','Zone ms','Cycle ms','Result'),(150,80,90,90,260)):
            self.table.heading(name,text=title);self.table.column(name,width=width)
        self.table.pack(fill='both',expand=True,padx=18,pady=5)
        ttk.Label(self.root,textvariable=self.angles,font=('Monospace',9)).pack(padx=18,pady=2)
        ttk.Label(self.root,textvariable=self.path,wraplength=750).pack(padx=18,pady=4)
        ttk.Label(self.root,text='RViz shows synthetic positions in simulation, live encoders in hardware. No sound-quality claim.').pack(pady=(0,8))
        self.root.bind('<Escape>',lambda _:self.stop());self.root.protocol('WM_DELETE_WINDOW',self.close_window)
        signal.signal(signal.SIGINT,lambda *_:self.close_window());signal.signal(signal.SIGTERM,lambda *_:self.close_window())
        self.refresh();self.root.after(20,self.tick)
        if args.auto_run:self.root.after(300,self.prepare)
        self.auto_pending=args.auto_run

    def load_parameters(self):
        mid=self.method.get()
        if mid=='all':
            self.description.set('All nine modes / eight method families, using saved/default parameters.')
            values=self.config.get('parameters',{})
        else:
            d=METHODS[mid];self.description.set(d.description)
            values=d.parameters(self.config.get('parameters',{}).get(mid))
        self.params.delete('1.0','end');self.params.insert('1.0',json.dumps(values,sort_keys=True))

    def prepare(self):
        if self.session and self.session.process.is_alive():return
        try:
            if self.session and not self.session.closed:self.session.close()
            self.fault=False;self.prepared=False;self.busy=True;self.stopping=False;self.paused=False
            self.pause_button.config(text='PAUSE AFTER CURRENT')
            self.session=self.session_factory(options_for(self.args,self.config));self.session.request('prepare')
            self.status.set('Preparing…' if not self.args.hardware else
                ('Preparing — motion notification waived for this session' if self.args.no_motion_notification
                 else 'Preparing — ntfy warning and countdown before movement'))
        except Exception as exc:self.status.set(str(exc));self.busy=False;self.fault=True
        self.refresh()

    def run_selected(self,all_methods=False):
        if not self.prepared or self.busy:return
        try:
            repeats=int(self.reps.get())
            if not 1<=repeats<=10000:raise ValueError('Repetitions must be 1–10000')
            mid='all' if all_methods else self.method.get();mids=list(METHODS) if mid=='all' else [mid]
            params=(self.config.get('parameters',{}) if all_methods else json.loads(self.params.get('1.0','end')))
            if mid!='all':params={mid:METHODS[mid].parameters(params)}
            else:
                for name,p in params.items():METHODS[name].parameters(p)
            self.session.request('run',methods=mids,repetitions=repeats,interval=self.args.interval,parameters=params)
            self.busy=True;self.status.set('Batch requested; all trials saved independently')
        except Exception as exc:self.status.set('Cannot start: '+str(exc))
        self.refresh()

    def campaign(self):
        if not self.prepared or self.busy:return
        try:
            mid=self.method.get();mids=list(METHODS) if mid=='all' else [mid]
            values=json.loads(self.params.get('1.0','end'))
            params=values if mid=='all' else {mid:values}
            for name,p in params.items():METHODS[name].parameters(p)
            self.session.request('campaign',methods=mids,stage=self.stage.get(),
                                 plan=self.config.get('plan',{}),parameters=params)
            self.busy=True;self.refresh()
        except Exception as exc:self.status.set('Cannot start campaign: '+str(exc))

    def toggle_pause(self):
        if self.session:
            self.paused=not self.paused;self.session.pause(self.paused)
            self.pause_button.config(text='RESUME' if self.paused else 'PAUSE AFTER CURRENT')

    def finish(self):
        if self.session and self.session.process.is_alive():
            self.paused=False;self.session.pause(False)
            self.pause_button.config(text='PAUSE AFTER CURRENT')
            self.session.request('finish');self.stopping=True;self.refresh()
            self.status.set('Will finish the remaining batch, then relax. STOP interrupts immediately.')

    def stop(self):
        self.prepared=False;self.busy=False;self.stopping=True
        if self.session:self.session.stop()
        self.status.set('Stop requested — worker will relax and flush trial records');self.refresh()

    def close_window(self):
        self.closing=True;self.stop()

    def refresh(self):
        alive=self.session is not None and self.session.process.is_alive()
        self.prepare_button.config(state='disabled' if alive else 'normal')
        for b in (self.run_button,self.all_button,self.campaign_button):
            b.config(state='normal' if alive and self.prepared and not self.busy and not self.fault and not self.stopping else 'disabled')
        self.finish_button.config(state='normal' if alive else 'disabled')

    def redraw(self):
        c=self.canvas;c.delete('all');w=max(700,c.winfo_width());h=175
        x0,y0=42,12;plotw=w-58;ploth=140
        y=lambda d:y0+d/12*ploth
        c.create_rectangle(x0,y(9),w-12,y(12),fill='#ffe4e6',outline='')
        for depth in (0,5,9,10,12):
            c.create_line(x0,y(depth),w-12,y(depth),fill='#cc3344' if depth in (9,10) else '#ddd',dash=(3,3))
            c.create_text(26,y(depth),text=f'{depth}°')
        if len(self.points)>1:
            end=max(.6,self.points[-1][0]);pts=[]
            for t,d in self.points:pts.extend((x0+t/end*plotw,y(d)))
            c.create_line(*pts,fill='#155e9b',width=2)

    def handle(self,msg):
        phase=msg['phase']
        if 'positions' in msg:self.positions=msg['positions']
        if 'anchor' in msg:self.anchor=msg['anchor']
        if 'session' in msg:self.path.set('Saved session: '+msg['session'])
        if phase=='READY' and not self.prepared and not self.stopping and not self.fault:
            self.prepared=True;self.busy=False
            if self.auto_pending:
                self.auto_pending=False
                self.campaign() if self.args.campaign else self.run_selected()
        if phase=='BATCH COMPLETE':
            self.busy=False;self.status.set('Batch complete. Report: '+msg['report'])
            if self.args.exit_after_batch:self.closing=True
        elif phase=='FAULT':
            self.fault=True;self.busy=False;self.prepared=False;self.status.set('FAULT — '+msg['error'])
            if self.args.exit_after_batch:self.closing=True
        elif phase=='CLOSED':self.busy=False;self.prepared=False
        elif phase=='TRIAL COMPLETE':
            s=msg['score'];fmt=lambda k:'—' if k not in s else f'{s[k]:.3f}'
            verdict='PASS' if s['passed'] else ', '.join(s['reasons'])
            self.table.insert('', 'end',values=(msg['method'],fmt('peak_depth_deg'),fmt('lower_zone_ms'),fmt('cycle_ms'),verdict))
            self.table.yview_moveto(1)
            self.metric.set(f"{msg['method']}: peak {fmt('peak_depth_deg')}° • lower zone {fmt('lower_zone_ms')} ms • {verdict}")
        elif not self.fault:
            self.status.set(('PAUSE REQUESTED — ' if self.paused else '')+phase)
        if 'depth_deg' in msg and 'trial' in msg:
            if msg['trial']!=self.current_trial:
                self.current_trial=msg['trial'];self.points=[];self.plot_started=time.monotonic()
            self.points.append((time.monotonic()-self.plot_started,msg['depth_deg']));self.redraw()

    def tick(self):
        if self.session:
            for msg in self.session.poll():self.handle(msg)
        self.angles.set('Right encoder degrees: '+'  '.join(f'J{i+1} {math.degrees(v):.2f}' for i,v in enumerate(self.positions)))
        if self.node:
            msg=self.JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
            for side in ('left','right'):
                for i in range(7):msg.name.append(f'openarmx_{side}_joint{i+1}');msg.position.append(self.positions[i] if side=='right' else 0.)
                msg.name.append(f'openarmx_{side}_finger_joint1');msg.position.append(0.)
            self.publisher.publish(msg)
        self.refresh()
        if self.closing and (self.session is None or not self.session.process.is_alive()):
            self.root.quit();return
        self.root.after(20,self.tick)

    def run(self):
        try:self.root.mainloop()
        finally:
            if self.session:self.session.close()
            if self.node:self.node.destroy_node();self.ros.shutdown()
            self.root.destroy()
