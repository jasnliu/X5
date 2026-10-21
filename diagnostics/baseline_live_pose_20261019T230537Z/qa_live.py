"""Actual camera/disabled encoders; native GUI/planning; NO actuator access."""
import json,sys,time
from pathlib import Path
import numpy as np
from PyQt5 import QtCore,QtTest,QtWidgets
from drum_planner.baseline import app,service,transport,tasks
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION,JOINTS,LEFT_JOINTS
from drum_planner.read_only import ALLOWED
from drum_planner.cli import main

root=Path(sys.argv[1]).resolve();root.mkdir(parents=True,exist_ok=True);session=root/'actual_startup';attempts=[]
def forbidden(*a,**kw):
    attempts.append('FORBIDDEN');raise AssertionError('No physical controller or motors permitted')
service.launch=forbidden;transport.BaselineMotors=forbidden
QtWidgets.QInputDialog.getText=forbidden;QtWidgets.QMessageBox.question=forbidden
Original=app.View
class QAView(Original):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.window.setWindowTitle('X5 QUERY-ONLY TEST — no motors enabled or moved')
        self.qa_stage='idle';self.qa_began=time.monotonic();self.qa_steps=[];self.idle_start=None
        self.qa_samples=0;self.qa_last_stamp=None;self.qa_replanned=False
        self.initial_model_uploads=[layer.geometry_uploads for *_,layer in self.robot.mesh_entries]
        assert self.live_display and self.reverse_button.isEnabled() and not self.active
    def poll(self,*a,**kw):
        key=super().poll(*a,**kw);now=time.monotonic()
        if self.qa_stage=='idle':
            if self.display_pose is not None:
                assert 0<=now-self.display_pose['stamp']<.5,(self.display_error,self.display_pose['stamp'],now)
                assert not self.display_error,self.display_error
                if self.idle_start is None:self.idle_start=now
                if self.qa_last_stamp!=self.display_pose['stamp']:
                    self.qa_samples+=1;self.qa_last_stamp=self.display_pose['stamp']
                if self.robot.last_positions is not None:
                    assert np.allclose(self.robot.last_positions,[self.display_pose['joints'][j] for j in JOINTS],atol=.001)
                if now-self.idle_start>=60:
                    assert self.qa_samples>800,self.qa_samples
                    self.qa_steps+=['60_seconds_actual_20Hz_query_only_feedback','live_model_matches_measured_joints','bounded_latest_only_storage']
                    self.viewport.save_window(self.window,root/'live_encoders.png')
                    self.qa_stage='animation'
                    for mode in range(self.mode.count()):self.mode.setCurrentIndex(mode)
                    self.mode.setCurrentIndex(0)
                    self.color.setCurrentIndex(1);self.color.setCurrentIndex(0)
                    self.viewport.set_angles(35,-30);self.draw_canvas();self.focus_arm()
                    self.rate_box.setValue(.5);self.seek(self.plan.obj['outward'][-1]['time_s'])
                    assert np.allclose(self.robot.last_positions,self.plan.req['goal'])
                    self.rate_box.setValue(1.);self.seek(0)
                    self.play_button.click();assert self.clock.playing and not self.live_display
                    self.qa_steps+=['mesh_points_colors_orbit_fit','speed_scale','planned_goal_and_timeline','play_switches_to_preview']
            elif now-self.qa_began>5:raise AssertionError('No encoder feedback: '+self.display_error)
        elif self.qa_stage=='animation' and self.clock.completed_cycles>=1:
            assert {'outward','HOLD (2 seconds)','return'}.issubset(self.phases_seen),self.phases_seen
            self.live_box.click();assert self.live_display
            self.qa_steps.append('full_planned_animation_then_live_again')
            self.prepare_button.click();assert self.job and self.telemetry is None and not self.start_when_ready
            self.qa_stage='readiness'
        elif self.qa_stage=='readiness' and self.job is None:
            assert not self.last_error,self.last_error
            if self.prepared:
                packet,curve=tasks.check_packet(self.prepared)
                assert packet['proof']['all_valid'];self.qa_replanned=bool(packet.get('replanned'))
            else:
                assert 'REPLACEMENT' in self.hardware_label.text(),self.hardware_label.text()
                self.qa_replanned=True
            assert not self.active and self.client is None and not attempts and not self.start_when_ready
            self.qa_steps+=['real_readiness_capture_lock_handoff','native_readiness_or_checked_replacement','no_implicit_controller_start']
            self.set_live_display(True);self.qa_stage='resumed';self.resume_began=now
        elif self.qa_stage=='resumed' and self.display_pose and self.display_pose['stamp']>self.resume_began+.5:
            assert not self.display_error and self.telemetry is not None
            assert self.initial_model_uploads==[layer.geometry_uploads for *_,layer in self.robot.mesh_entries]
            assert not self.viewport.errors
            self.qa_steps+=['query_reader_resumes_after_readiness','retained_robot_meshes_no_reupload','reverse_control_stays_clickable','save_view','clean_close']
            self.save_view();self.viewport.save_window(self.window,root/'live_ready.png')
            atomic_json(root/'live_ui_report.json',dict(passed=True,checks=self.qa_steps,read_only_batches=self.qa_samples,
                idle_seconds=60,replanned=self.qa_replanned,plan=str(self.plan.path),render=self.report(),
                motor_commands=0,controller_launch_attempts=attempts))
            self.qa_stage='done';self.close_button.click()
        if self.last_error:raise AssertionError(self.last_error)
        if now-self.qa_began>240:raise AssertionError('Non-motion GUI test timeout')
        return key
app.View=QAView
code=main(['baseline','--output',str(session)])
assert code==0 and not attempts,(code,attempts)
# Audit every retained packet; bounded logs rotate during the soak, and the exact-query gate audits all sends.
tx=[]
for path in session.rglob('can_query_audit.jsonl*'):
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    assert all(bytes.fromhex(r['packet_hex']) in ALLOWED for r in rows)
    tx.extend(r for r in rows if r.get('event')=='sent')
assert tx
atomic_json(root/'live_tx_audit.json',dict(sent_queries=len(tx),exact_allowlist=True,motion_commands=0,enable_commands=0,
    disable_commands=0,controller_launch_attempts=attempts,all_checked=True))
print('LIVE QUERY-ONLY QA PASSED',len(tx),'audited requests; zero actuator commands',flush=True)
