"""Unmodified regular capture UI; real camera/CAN, TX restricted to state queries.

This runs genuine automatic collection, NOT a calibration fit or mount validation.
Only Start, Stop/Resume automatic capture, and Quit are automated; no motion.
"""
import hashlib
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time

ROOT=Path('/home/jason/Proyectos3/X5')
sys.path.insert(0,str(ROOT))
BASE=Path(__file__).resolve().parent
RUN=BASE/('run_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()))
RUN.mkdir()
EXPECTED=frozenset(struct.pack('=IB3x8s',0x8200fd00|i,8,b'\x01'+b'\0'*7) for i in range(1,9))
CAN=socket.PF_CAN
real_socket=socket.socket
wire_log=(RUN/'wire_audit.jsonl').open('w')
lock=threading.Lock()
wire=dict(tx=0,rx=0,blocked=[],bindings=[],error_frames=[],rtr_frames=[],
          motor_feedback={str(i):0 for i in range(1,9)},modes_seen={},fault_bits_seen={})

def audit(direction,raw=None,detail=None):
    row=dict(monotonic=time.monotonic(),direction=direction)
    if raw is not None:
        row['packet_hex']=raw.hex()
    if detail is not None:
        row['detail']=detail
    with lock:
        wire_log.write(json.dumps(row)+'\n'); wire_log.flush()
        if direction in ('tx','rx'): wire[direction]+=1
        if direction=='blocked': wire['blocked'].append(row)
        if direction=='bind': wire['bindings'].append(detail)
        if direction=='rx' and len(raw)==16:
            cid,dlc,data=struct.unpack('=IB3x8s',raw)
            if cid&0x20000000:
                wire['error_frames'].append(dict(can_id_hex=hex(cid),payload_hex=data.hex(),monotonic=row['monotonic']))
            elif cid&0x40000000:
                wire['rtr_frames'].append(dict(can_id_hex=hex(cid),monotonic=row['monotonic']))
            elif cid&0x80000000 and ((cid>>24)&31)==2 and 1<=((cid>>8)&255)<=8:
                motor=str((cid>>8)&255)
                wire['motor_feedback'][motor]+=1
                mode=(cid>>22)&3; fault=(cid>>16)&63
                for name,value in [('modes_seen',mode),('fault_bits_seen',fault)]:
                    if value not in wire[name].setdefault(motor,[]): wire[name][motor].append(value)

class RestrictedSocket(real_socket):
    def bind(self,address):
        if self.family==CAN:
            if address != ('can0',):
                audit('blocked',detail=repr(address)); raise RuntimeError('Test forbids any CAN bus except can0')
            audit('bind',detail='can0')
        return super().bind(address)
    def send(self,data,*args,**kwargs):
        if self.family==CAN:
            if data not in EXPECTED:
                audit('blocked',bytes(data)); raise RuntimeError('Test blocked a non-query CAN command')
            audit('tx',bytes(data))
        return super().send(data,*args,**kwargs)
    def recv(self,*args,**kwargs):
        data=super().recv(*args,**kwargs)
        if self.family==CAN: audit('rx',data)
        return data
    def sendall(self,*args,**kwargs):
        if self.family==CAN:
            audit('blocked',detail='sendall'); raise RuntimeError('CAN sendall forbidden')
        return super().sendall(*args,**kwargs)
    def sendto(self,*args,**kwargs):
        if self.family==CAN:
            audit('blocked',detail='sendto'); raise RuntimeError('CAN sendto forbidden')
        return super().sendto(*args,**kwargs)
    def sendmsg(self,*args,**kwargs):
        if self.family==CAN:
            audit('blocked',detail='sendmsg'); raise RuntimeError('CAN sendmsg forbidden')
        return super().sendmsg(*args,**kwargs)
socket.socket=RestrictedSocket

from camera_calibration import app as module, cli
OriginalApp=module.App
report={}

def launch(args):
    app=OriginalApp(args)
    started=time.monotonic(); ready=None; last_sequence=0; displayed=0
    seen_batches=set(); max_encoder_age=0.; pauses=[]; was_paused=False; closed=False
    phase='endurance'; phase_start=None; auto_duration=None; stopped_count=None; stop_verified=False; resume_verified=False; screenshot_saved=False
    app.session.data['diagnostic_only'] = True; app.session.save()
    samples_log=(RUN/'health_samples.jsonl').open('w')
    def finish(reason):
        nonlocal closed
        if closed: return
        closed=True
        now=time.monotonic(); camera=app.camera; observer=app.observer
        seconds=0. if ready is None else now-ready
        healthy_modes=all(wire['modes_seen'].get(str(i))==[0] and wire['fault_bits_seen'].get(str(i))==[0] for i in range(1,9))
        report.update(success=(reason=='completed' and seconds>=65 and app.failure is None and not app.camera_waiting
                               and not pauses and not wire['blocked'] and not wire['error_frames'] and not wire['rtr_frames']
                               and wire['bindings']==['can0'] and healthy_modes and observer is not None
                               and observer.error is None and max_encoder_age<.5 and not app.session.data['preview_only']
                               and auto_duration is not None and auto_duration>=65 and stop_verified and resume_verified),
                      reason=reason,failure=app.failure,observer_error=observer.error if observer else None,
                      command=args.command,preview_only=app.session.data['preview_only'],
                      combined_ready_duration_s=seconds,wall_duration_s=now-started,
                      displayed_frames=displayed,encoder_batches_observed=len(seen_batches),
                      maximum_encoder_age_s=max_encoder_age,pauses=pauses,
                      quality_text=app.quality.get(),status_text=app.status.get(),session=str(app.session.path),
                      saved_pose_count=len(app.session.data['samples']),board_mount_not_revalidated=True,
                      automatic_continuous_duration_s=auto_duration,stop_button_verified=stop_verified,
                      resume_button_verified=resume_verified,auto_status=app.auto_status.get(),counter_text=app.counts.get(),
                      saved_phases=[s['phase'] for s in app.session.data['samples']],
                      saved_stationary_durations_s=[s['stationary_evidence']['duration_s'] for s in app.session.data['samples']])
        app.close()
        report['wire_audit_summary']=wire
        report['camera_report']=camera.jpeg_report if camera else None
        report['camera_released']=camera is not None and camera.process is not None and camera.process.poll() is not None
        report['observer_released']=observer is not None and not observer.thread.is_alive()
        report['success']=report['success'] and report['camera_released'] and report['observer_released']
        samples_log.close()
        (RUN/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        (BASE/'latest_run.txt').write_text(str(RUN)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    def start():
        # Setup was described by the user previously; this run saves no poses.
        # Motor disabled/fault state is checked from actual feedback by the
        # ORIGINAL observer, not assumed, replaced, or synthesized here.
        app.board_ok.set(True); app.camera_ok.set(True); app.arm_ok.set(True)
        try: app.start_button.invoke()
        except Exception as e:
            app.fatal(str(e)); finish('startup_error')
    def poll():
        nonlocal ready,last_sequence,displayed,max_encoder_age,was_paused
        nonlocal phase,phase_start,auto_duration,stopped_count,stop_verified,resume_verified,screenshot_saved
        if closed: return
        now=time.monotonic(); camera=app.camera; observer=app.observer
        error=app.failure or (camera.error if camera else None) or (observer.error if observer else None)
        if error:
            if not app.failure: app.fatal(error)
            finish('application_error'); return
        latest=camera.latest if camera else None
        batches=observer.samples() if observer else []
        encoder_age=now-batches[-1]['end'] if batches else None
        if app.sequence and app.sequence!=last_sequence:
            displayed+=1; last_sequence=app.sequence
        for batch in batches: seen_batches.add(batch['end'])
        if app.camera_waiting and not was_paused:
            pauses.append(dict(monotonic=now,status=app.status.get()))
        was_paused=app.camera_waiting
        if ready is None and latest is not None and batches and app.sequence and not app.camera_waiting and app.auto.active:
            if now-latest[0]['received_monotonic']<.5 and encoder_age<.25:
                ready=now
        if ready is not None:
            max_encoder_age=max(max_encoder_age,encoder_age if encoder_age is not None else 999.)
        samples_log.write(json.dumps(dict(monotonic=now,ready_since=ready,frame_sequence=app.sequence,
                                          camera_age_s=now-latest[0]['received_monotonic'] if latest else None,
                                          encoder_age_s=encoder_age,encoder_batches=len(seen_batches),
                                          camera_waiting=app.camera_waiting,error=error,automatic_active=app.auto.active,
                                          test_phase=phase,pose_count=len(app.session.data['samples']),auto_status=app.auto_status.get()))+'\n')
        if ready is not None and not screenshot_saved and now-ready>3:
            from PIL import ImageGrab
            x,y=app.root.winfo_rootx(),app.root.winfo_rooty()
            ImageGrab.grab(bbox=(x,y,x+app.root.winfo_width(),y+app.root.winfo_height())).save(RUN/'automatic_ui.png')
            screenshot_saved=True
        if ready is not None and phase=='endurance':
            if not app.auto.active:
                finish('automatic_stopped_unexpectedly'); return
            if now-ready>=65:
                auto_duration=now-ready
                stopped_count=len(app.session.data['samples'])
                app.auto_button.invoke()
                if app.auto.active:
                    finish('stop_button_failed'); return
                phase='stopped'; phase_start=now
        elif phase=='stopped':
            if app.auto.active or len(app.session.data['samples'])!=stopped_count:
                finish('capture_after_stop'); return
            if now-phase_start>=3:
                stop_verified=True
                app.auto_button.invoke()
                if not app.auto.active:
                    finish('resume_button_failed'); return
                phase='resumed'; phase_start=now
        elif phase=='resumed' and now-phase_start>=3:
            resume_verified=app.auto.active
            if len(app.session.data['samples'])!=stopped_count:
                finish('unexpected_extra_pose_after_resume'); return
            finish('completed'); return
        if ready is None and now-started>25:
            finish('readiness_timeout'); return
        if now-started>100:
            finish('test_timeout'); return
        app.root.after(80,poll)
    app.root.protocol('WM_DELETE_WINDOW',lambda:finish('closed_early'))
    app.root.after(100,start)
    app.root.after(150,poll)
    return app
module.App=launch
try:
    rc=cli.main(['capture','--output',str(RUN/'sessions')])
finally:
    wire_log.close()
if not report:
    raise SystemExit('No completed endurance report; test did not pass')
raise SystemExit(0 if report['success'] else 1)
