"""Explicit emergency shutdown, distinct from a normal centered stop.

Writers must be dead before disable packets are sent. No geometry, gains
restoration, center evidence or peripheral health can gate this path.
"""
from goal_motion.app import App as JointGoalApp


def stop_writer(session):
    process = session.process
    event = getattr(session, 'stop_event', None)
    if event is None:
        event = getattr(session, 'cancel', None)
    if event is not None:
        event.set()
    process.join(.15)
    if process.is_alive():
        process.terminate()
        process.join(.3)
    if process.is_alive():
        process.kill()
        process.join(.3)
    if process.is_alive():
        raise RuntimeError('Motor writer could not be stopped; USE PHYSICAL POWER CUTOFF')
    # J7 sessions own extra status/queue handles and a bus ownership reference.
    if hasattr(session, 'stop_event'):
        session.stop()


def relax(app, reason):
    if getattr(app, 'emergency_busy', False):
        return
    app.emergency_busy = True
    app.emergency_latched = True  # Restart required: interrupted gains are unknown.
    try:
        app.control = app.setup = app.hold_until = None
        app.playback_active = app.alignment_active = app.search_active = False
        app.strike_active = app.continuous_strike_active = False
        app.hybrid_stopping = app.hybrid_restoring = False
        bus = app.bus
        for sock in getattr(bus, 'sockets', {}).values():
            sock.emergency_latched = True
        dual = getattr(app, 'dual', None)
        if dual is not None:
            dual.returning = False
            dual.return_prepared = False
        bus.mit_center_return = None
        sessions = []
        for session in (getattr(dual, 'session', None),
                        getattr(app, 'smooth_playback_session', None),
                        getattr(app, 'hybrid_session', None),
                        getattr(app, 'hardware_test_session', None),
                        getattr(bus, 'playback_session', None),
                        getattr(bus, 'right_joint7_session', None),
                        getattr(bus, 'left_joint6_session', None)):
            if session is not None and all(session is not old for old in sessions):
                sessions.append(session)
        failures = []
        for session in sessions:
            try:
                stop_writer(session)
            except Exception as exc:
                failures.append(str(exc))
        # Stop peripheral callbacks without invoking arm failure/recovery again.
        calibration = getattr(app, 'hihat_calibration', None)
        if calibration is not None:
            try:
                calibration.close()
            except Exception as exc:
                print('EMERGENCY calibration cleanup: '+str(exc), flush=True)
        sync = getattr(app, 'hihat_sync', None)
        if sync is not None:
            try:
                sync.stop()
            except Exception as exc:
                print('EMERGENCY hi-hat sync cleanup: '+str(exc), flush=True)
        hihat = getattr(app, 'hihat', None)
        if hihat is not None:
            try:
                hihat.release_all()
            except Exception as exc:
                print('EMERGENCY hi-hat release unconfirmed: '+str(exc), flush=True)
        if failures:
            raise RuntimeError('; '.join(failures))
        for name in ('smooth_playback_session', 'hybrid_session', 'hardware_test_session'):
            setattr(app, name, None)
        bus.playback_session = bus.right_joint7_session = bus.left_joint6_session = None
        if dual is not None:
            dual.session = None
        bus.left_monitor = None
        bus.left_playback_active = False
        bus.emergency_relax_reason = str(reason)
        print('EMERGENCY RELAX REQUEST: '+str(reason)+'; restart required', flush=True)
        JointGoalApp.relax(app, str(reason)+'; restart required')
    except Exception as exc:
        app.phase = 'FAULT'
        app.status.set('Emergency disable unconfirmed: '+str(exc)+'; USE PHYSICAL POWER CUTOFF')
        print(app.status.get(), flush=True)
    finally:
        app.emergency_busy = False
