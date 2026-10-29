"""Contained offline entry point. No hardware backend is imported."""
import json
from drum_planner.experiment.config import no_can
from .cli import parser,dispatch
from .schema import write,identity
from pathlib import Path


def main():
    no_can();args=parser().parse_args()
    print('OFFLINE SOFTWARE TEST — no camera, CAN, enable/disable or motor commands',flush=True)
    result=dispatch(args)
    keys=('profile','complete_probe_coverage','synthetic','hardware_limits_known',
          'predicted_movement_s','calculation_s','missing_coverage')
    print(json.dumps({k:result[k] for k in keys if k in result},indent=2))
    write(Path(args.output)/'completion.json',dict(completed=True,arguments_sha256=identity(vars(args)),
        hardware_access=False,motor_commands=0))
    print('Output:',args.output,flush=True)


if __name__=='__main__':main()
