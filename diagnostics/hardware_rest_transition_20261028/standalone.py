"""Rebuild standalone destination from the same saved pair, never a device."""
from pathlib import Path
import shutil
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
from cymbal_destination import inputs,worker,output
no_can()
base=ROOT/'diagnostics/hardware_rest_transition_20261028'
run=base/'standalone';acq=run/'acquisition';camera=acq/'camera';camera.mkdir(parents=True,exist_ok=False)
vision=base/'verified/latest_dt1/hardware/dt1/vision'
for p in vision.iterdir():
 if p.is_file():shutil.copy2(p,camera/p.name)
capture=ROOT/'planner_runs/20261028T023247Z-hardware-right-ae273b/calculate-7f013e4d/capture'
shutil.copy2(capture/'measured_start.json',acq/'measured_start.json')
inputs.build_scene(acq,DEFAULT_CALIBRATION)
atomic_json(acq/'acquisition_report.json',dict(software_only=True,saved_capture=str(capture),camera_opened=False,motor_commands=0))
r=worker.run(run);assert r['status']=='ready';output.verified(run)
print('STANDALONE SAVED-CAPTURE ENDPOINT VERIFIED')
