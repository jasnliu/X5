"""Load the byte-identical pre-GPU worker without replacing installed programs."""
import importlib.util,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location('drum_planner.experiment.pre_gpu_benchmark',Path(__file__).with_name('cpu_worker_before_gpu.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.main()
