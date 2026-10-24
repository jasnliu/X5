"""Full hardware Calculate entrypoint with recorded camera + fake encoders only."""
import importlib.util,json,sys
from pathlib import Path
from drum_planner.common import ROOT
spec=importlib.util.spec_from_file_location('harness',ROOT/'diagnostics/feedback_isolation_20261021/verify_calculation.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.INCIDENT=ROOT/'planner_runs/20261022T203104Z-hardware-right-5e5d2b/calculate-f2ec86b8'
original=m.work.calculate
binding=json.loads((m.INCIDENT/'destinations.json').read_text())
def calculate(*a,**kw):return original(*a,**kw,destinations=binding)
m.work.calculate=calculate
m.main()
