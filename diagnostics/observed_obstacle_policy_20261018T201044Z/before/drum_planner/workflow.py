"""Testable software-only cycle/recovery policy. Never controls torque."""
import numpy as np


def recovery_action(*, stall=False, feedback_ok=True, motion_available=True,
                    at_saved_start=False, stationary=False, supported=False,
                    checked_return_available=False):
    if stall or not feedback_ok or not motion_available:
        return 'STOP_AND_HOLD_INTENT_NO_RELAX_OPERATOR_REQUIRED'
    if at_saved_start and stationary and supported:
        return 'RELAX_INTENT_ONLY'
    if checked_return_available:
        return 'RETURN_TO_SAVED_START_INTENT'
    return 'STOP_AND_HOLD_INTENT_NO_RELAX_OPERATOR_REQUIRED'


def simulate(plan):
    if plan.get('status')!='planned' or plan.get('motion_authorized') is not False:
        raise ValueError('Only an explicitly non-executable planned preview can be simulated')
    outward=plan['outward'];cycle=plan['cycle']
    start=np.array(plan['start_state']['right_rad'])
    if not np.allclose(outward[0]['positions_rad'],start,atol=1e-10,rtol=0):
        raise ValueError('Wrong starting pose')
    if not np.allclose(cycle[-1]['positions_rad'],start,atol=1e-10,rtol=0):
        raise ValueError('Return does not end at the saved start')
    held=next(p for p in cycle if p['phase']=='hold_complete')
    if abs(held['time_s']-outward[-1]['time_s']-2.)>1e-9:
        raise ValueError('Expected a two-second hold')
    if not np.array_equal(held['positions_rad'],outward[-1]['positions_rad']):
        raise ValueError('Arm moved during the hold')
    return dict(status='software_cycle_verified',events=['START','OUTWARD','HOLD_2_SECONDS','RETURN_TO_SAVED_START',
                       recovery_action(at_saved_start=True,stationary=True,supported=True)],
                supported_rest_is_synthetic=True,right_gripper='custom CLOSED +7 degrees for all movement',
                left_arm='static, no commands',motor_commands=0,
                recoverable_fault=recovery_action(checked_return_available=True),
                stall=recovery_action(stall=True,checked_return_available=True),
                missing_feedback=recovery_action(feedback_ok=False,checked_return_available=True),
                unsafe_return=recovery_action(checked_return_available=False),
                unsupported_rest=recovery_action(at_saved_start=True,stationary=True,supported=False),
                physical_recovery_implemented=False)
