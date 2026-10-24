"""Software-only trajectory experiments; planning and playback cannot use CAN.

The optional, separate start_pose entry point only queries the selected arm,
then closes before the software-only workers/UI start. It cannot move motors.
"""
