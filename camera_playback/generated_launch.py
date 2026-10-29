"""ROS launch adapter: never time-kill an owner holding suspended arms."""
from launch.actions import ExecuteProcess, TimerAction


class HeldArmProcess(ExecuteProcess):
    def _shutdown_process(self, context, *, send_sigint):
        # Keep ROS's process lifecycle / explicit SIGINT delivery, but remove
        # its standard timed SIGTERM/SIGKILL escalation for THIS owner only.
        # The panel now handles explicit Ctrl-C/close by disabling both arms.
        # Let that disable/cleanup finish rather than racing a forced kill;
        # no pose condition blocks exit. Perception keeps normal cleanup.
        actions=super()._shutdown_process(context,send_sigint=send_sigint)
        return None if actions is None else [a for a in actions if not isinstance(a,TimerAction)]
