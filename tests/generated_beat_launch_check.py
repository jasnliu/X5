"""Real ROS launch shutdown against a fake owner; no device access."""
import json
from pathlib import Path
import sys
import tempfile
from launch import LaunchDescription,LaunchService
from launch.actions import EmitEvent,TimerAction
from launch.events import Shutdown
from camera_playback.generated_launch import HeldArmProcess


def main():
    with tempfile.TemporaryDirectory(prefix='generated-launch-check-') as folder:
        proof=Path(folder)/'returned'
        # A fake powered owner handles the first close request but stays alive
        # longer than BOTH launch escalation deadlines to model operator return.
        child="""
import signal,time,sys
from pathlib import Path
signal.signal(signal.SIGINT,lambda *_:None)
time.sleep(1.0)
Path(sys.argv[1]).write_text('explicit fake rest return completed')
"""
        owner=HeldArmProcess(cmd=[sys.executable,'-c',child,str(proof)],
            sigterm_timeout='.05',sigkill_timeout='.05',output='screen')
        launch=LaunchService(noninteractive=True)
        launch.include_launch_description(LaunchDescription([owner,TimerAction(period=.35,
            actions=[EmitEvent(event=Shutdown(reason='software-only fake close request'))])]))
        code=launch.run()
        assert code==0 and owner.return_code==0 and proof.exists()
    path=Path('diagnostics/generated_beat_20261028/launch_hold_report.json')
    path.write_text(json.dumps(dict(passed=True,hardware_access=False,
        check='real ROS shutdown did not kill fake owner past both escalation deadlines'),indent=2)+'\n')

if __name__=='__main__':main()
