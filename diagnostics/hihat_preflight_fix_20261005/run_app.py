"""Physical regression driver: ordinary controls/models; defer file choice until calibrated.

Only automates load_recording(record3) and the existing 5-second --verify-swing
RUN/Center+Relax helper. Never substitutes feedback or changes motion parameters.
"""
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import camera_playback.app as module
OUT=Path(__file__).resolve().parent

class VerificationApp(module.App):
    def __init__(self,*args,**kwargs):
        args=list(args)
        self.selected=Path(args[3]);assert self.selected.resolve()==ROOT/'recordings/record3.json'
        args[3]=None
        super().__init__(*args,**kwargs)
        self.test_stream=(OUT/'physical_background.jsonl').open('w',buffering=1)
        self.load_started=self.loaded_at=None
        self.last_watch=time.monotonic();self.max_gap=0.
        self.query_times=[];self.heartbeat_times=[];self.status_times=[]
        self.calibration_selected=None
        write=self.hihat._write
        def audited_write(cmd):
            now=time.monotonic()
            if cmd==b'Q':self.query_times.append(now)
            if cmd==b'H':self.heartbeat_times.append(now)
            write(cmd)
        self.hihat._write=audited_write
        handle=self.hihat._handle_line
        def audited_line(line):
            handle(line)
            if line.startswith('HIHAT '):self.status_times.append(time.monotonic())
        self.hihat._handle_line=audited_line
        self.root.after(20,self.watch)

    def watch(self):
        now=time.monotonic()
        if self.load_started is not None and self.loaded_at is None:
            self.max_gap=max(self.max_gap,now-self.last_watch)
        self.last_watch=now
        e=self.hihat_calibration.engine
        if e.ready and self.load_started is None and not self.close_requested:
            self.calibration_selected=e.selected_angle
            self.load_started=now
            print('REGRESSION: calibration succeeded; loading record3 with live ESP32 supervision',flush=True)
            assert self.load_recording(self.selected,show_dialog=False)
        if self.load_started is not None and self.loaded_at is None and self.recording_preflight is None and self.playback_trajectory is not None:
            self.loaded_at=now
            print('REGRESSION: record3 ready; validation seconds='+str(now-self.load_started),flush=True)
        self.test_stream.write(json.dumps(dict(at=now,app_phase=self.phase,calibration=e.phase,
            selected=e.selected_angle,calibration_fault=e.failure,arm_active=self.bus.active,
            loading=self.recording_preflight is not None,telemetry_at=self.hihat.telemetry_at,
            hihat_fault=self.hihat_fault_detail))+'\n')
        self.root.after(20,self.watch)

    def run(self):
        try:return super().run()
        finally:
            load_h=[t for t in self.heartbeat_times if self.load_started and self.loaded_at and self.load_started<=t<=self.loaded_at]
            (OUT/'physical_background_result.json').write_text(json.dumps(dict(
                physical=True,calibration_angle=self.calibration_selected,
                load_started=self.load_started,loaded_at=self.loaded_at,
                load_seconds=None if self.loaded_at is None else self.loaded_at-self.load_started,
                max_gui_gap_during_load=self.max_gap,heartbeats_during_load=len(load_h),
                max_heartbeat_gap_during_load=max((b-a for a,b in zip(load_h,load_h[1:])),default=None),
                status_replies_during_load=sum(self.load_started<=t<=self.loaded_at for t in self.status_times) if self.loaded_at else 0,
                hihat_fault=self.hihat_fault_detail,calibration_fault=self.hihat_calibration.engine.failure,
                final_phase=self.phase,arm_active=self.bus.active),indent=2)+'\n')
            self.test_stream.close()

if __name__=='__main__':
    module.App=VerificationApp
    module.main()
