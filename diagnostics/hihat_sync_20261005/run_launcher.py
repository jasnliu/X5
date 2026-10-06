"""Use the existing launch graph, swapping only the evidence-duration wrapper."""
from pathlib import Path
import runpy,sys
import launch.actions
import launch_ros.actions  # load Node against the real ExecuteProcess class first
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
original=launch.actions.ExecuteProcess

def execute(*args,**kwargs):
    command=kwargs.get('cmd',[])
    if command[:3]==['/usr/bin/python3','-m','camera_playback.app']:
        kwargs['cmd']=[command[0],str(Path(__file__).with_name('run_app.py'))]+command[3:]
    return original(*args,**kwargs)

if __name__=='__main__':
    launch.actions.ExecuteProcess=execute
    runpy.run_path(str(ROOT/'launch_right_camera_playback.py'),run_name='__main__')
