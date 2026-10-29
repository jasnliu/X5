"""Same generated hardware planning workers, with no actuator owner or ESP32.

Only acquisition can open CAN, through generated_acquisition's query allowlist.
Planning uses the normal resource-limited, CAN-forbidden worker. Nothing here
can Start, enable, hold, relax, or calibrate a motor, including on cancellation.
"""
import json
from pathlib import Path
import time
import uuid
from drum_planner.common import ROOT, atomic_json
from drum_planner.experiment.runtime import Job, PYTHON


def run(folder=None):
    folder=Path(folder) if folder is not None else ROOT/'planner_runs'/(
        'generated-prepare-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
    folder=folder.resolve();folder.mkdir(parents=True,exist_ok=False)
    capture=folder/'capture';plan=folder/'plan';job=None
    print('PREPARE ONLY: fresh camera + query-only encoders. NO arm commands, ESP32, hi-hat calibration, or audio.',flush=True)
    print('Output: '+str(folder),flush=True)
    try:
        for kind,command,output,timeout,allow_can in (
            ('acquisition',[str(PYTHON),'-m','camera_playback.generated_acquisition','--folder',str(capture)],capture,60,True),
            ('planning',[str(PYTHON),'-m','camera_playback.generated_planning','prepare',
                         '--folder',str(plan),'--capture',str(capture)],plan,300,False)):
            job=Job(command,folder/(kind+'-job'),timeout=timeout,allow_can=allow_can)
            while (code:=job.poll()) is None:time.sleep(.05)
            if code:
                error=output/'error.json'
                detail=json.loads(error.read_text())['error'] if error.exists() else 'See '+str(job.log_path)
                raise RuntimeError(kind+' failed: '+detail)
            job.close();job=None
        from .generated_planning import load
        mission,_=load(plan/'mission.json')
        atomic_json(folder/'prepare_only.json',dict(status='ready',mission=str(plan/'mission.json'),
            duration_s=mission['duration_s'],actuator_owner_created=False,esp32_access=False,
            motor_commands=0,capture_query_report=str(capture/'query_report.json')))
        print('PATHS READY (not executed): '+str(plan/'mission.json'),flush=True)
        print('Preview without devices: ./start_beat.sh --generated-simulate '+str(plan/'mission.json'),flush=True)
        return plan/'mission.json'
    except BaseException as exc:
        atomic_json(folder/'prepare_only.json',dict(status='failed',error=str(exc),
            actuator_owner_created=False,esp32_access=False,motor_commands=0))
        raise
    finally:
        if job is not None:job.close()


def main():
    try:run()
    except KeyboardInterrupt:
        print('Preparation cancelled; query/camera/planning workers closed. No actuator owner existed.',flush=True)
        return 130
    except Exception as exc:
        print('PREPARE ONLY FAILED: '+str(exc),flush=True)
        return 1
    return 0
