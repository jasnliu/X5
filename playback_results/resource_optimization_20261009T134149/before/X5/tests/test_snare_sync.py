"""Offline snare calibration, worker timing, and two-stage workflow regression."""
from dataclasses import replace
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from camera_playback.snare_sync import SnareSynchronizer
from camera_playback.snare_sync_runtime import SnareSyncRuntime
from camera_playback.hihat_sync import TimingFault
from camera_playback.hihat_sync_runtime import HiHatSyncRuntime
from camera_playback.snare_beat import SnareStatus, minimum_hit_spacing
from camera_playback.tempo import SYNC_PERIOD, SYNC_BPM, swing_events
from camera_playback.mit_strike import Sample
from camera_playback.hybrid_strike import HybridStatus
from camera_playback import hybrid_workflow as hw, snare_workflow
import test_snare_beat as sb
from test_hybrid_workflow import app_fixture


@pytest.fixture(scope='module')
def rig():
    sb.SnareBeatTests.setUpClass()
    return sb.SnareBeatTests()


def engine_pair(e, index, *, used=0., error=0., lateness=0., duplicate=False):
    target=e.epoch+(index-1)*SYNC_PERIOD
    release=target-e.nominal_lead-used+lateness
    state=SnareStatus(calibrating=True,strike_index=index,scheduled_at=target,
                      released_at=release,advance=used)
    e.observe(state)
    now=max(target+.5,release+.5)
    for name,onset in [('ride',target),('snare',target+error)]:
        messages=[dict(kind='hit',instrument=name,event_at=onset),
                  dict(kind='progress',instrument=name,finalized_at=now)]
        if duplicate and name=='snare':messages.append(dict(kind='hit',instrument=name,event_at=onset+.02))
        e.feed(name,messages,now)
    e.tick(now,dict(ride=True,snare=True))


def engine():
    e=SnareSynchronizer(.146)
    e.start(10.);e.set_epoch(11.,10.)
    return e


def test_shared_80_bpm_and_independent_snare_correction_uses_actual_release():
    assert SYNC_BPM==80. and SYNC_PERIOD==.75
    e=engine()
    engine_pair(e,1,error=.07,lateness=.003)
    assert e.advance==pytest.approx(.067)  # Do not add the 146 ms descent twice.
    engine_pair(e,2,used=.067,error=.004)
    assert not e.locked
    engine_pair(e,3,used=.067,error=-.004)
    assert e.complete and e.advance==pytest.approx(.067)


@pytest.mark.parametrize('bad', ['bad_error','duplicate','missing','old_offset'])
def test_two_consecutive_good_pairs_required(bad):
    e=engine();engine_pair(e,1,error=.002)
    if bad=='missing':
        target=11.+SYNC_PERIOD
        e.observe(SnareStatus(calibrating=True,strike_index=2,scheduled_at=target,
                            released_at=target-.146))
        for k in e.events:e.feed(k,[dict(kind='progress',instrument=k,finalized_at=target+.6)],target+.6)
        e.tick(target+.6,dict(ride=True,snare=True))
    else:
        engine_pair(e,2,error=.04 if bad=='bad_error' else 0.,
                    duplicate=bad=='duplicate',used=.02 if bad=='old_offset' else 0.)
    assert not e.locked
    engine_pair(e,3,used=e.advance,error=.001)
    assert not e.locked
    engine_pair(e,4,used=e.advance,error=.001)
    assert e.locked


def test_wrong_instruments_lost_release_and_timeout_are_not_good_pairs():
    e=engine()
    e.feed('snare',[dict(kind='hit',instrument='hihat',event_at=11.)],12.)
    assert not e.events['snare']
    with pytest.raises(TimingFault,match='lost'):
        e.observe(SnareStatus(calibrating=True,strike_index=2,scheduled_at=11.75))
    with pytest.raises(TimingFault,match='45 seconds'):e.tick(56.,{})


@pytest.mark.parametrize('latency,delivery',[(.045,.6),(-.07,.8),(.09,2.4)])
def test_real_snare_controller_converges_with_delayed_audio_and_unchanged_stroke(rig,latency,delivery):
    c=rig.controller();rig.prime(c)
    c.start(1.,SYNC_PERIOD,calibration=True)
    e=SnareSynchronizer(rig.template.curve.down)
    e.start(.12);e.set_epoch(1.,.12)
    q,v=rig.anchor,0.;pending=[];next_ride=1.;seen=0;locked_at=None
    previous_release=None
    for i in range(60,22000):
        now=i*.002
        command=c.update(rig.sample(now,q,v),now,-rig.anchor)
        q,v=command.position,command.velocity
        while next_ride<=now:
            pending.append((next_ride+delivery,'ride',next_ride));next_ride+=SYNC_PERIOD
        if c.status.strike_index>seen:
            seen=c.status.strike_index;e.observe(c.status)
            at=c.status.released_at+rig.template.curve.down+latency
            pending.append((at+delivery+.2,'snare',at))
            if previous_release is not None:
                assert c.status.released_at-previous_release>=minimum_hit_spacing(rig.template,rig.rules)-.003
            previous_release=c.status.released_at
            assert c.method.target==rig.template.target
            assert c.method.curve.duration==rig.template.curve.duration
        ready=[p for p in pending if p[0]<=now];pending=[p for p in pending if p[0]>now]
        for _,name,at in reversed(ready):e.feed(name,[dict(kind='hit',instrument=name,event_at=at)],now)
        for name,lag in [('ride',delivery),('snare',delivery+.2)]:
            e.feed(name,[dict(kind='progress',instrument=name,finalized_at=now-lag-.002)],now)
        e.tick(now,dict(ride=True,snare=True));c.set_advance(e.advance)
        if e.locked and locked_at is None:
            locked_at=now;c.finish();e.stop(now)
        if locked_at is not None and not c.status.moving and not c.status.scheduled:break
    assert locked_at is not None
    assert e.advance==pytest.approx(latency,abs=.004)
    assert q==pytest.approx(rig.anchor,abs=1e-5)
    assert c.status.ready


@pytest.mark.parametrize('advance',[-.25,.05,.25])
def test_fixed_swing_offset_does_not_change_grid_or_shorten_return_and_first_bar_is_feasible(rig,advance):
    c=rig.controller();c.rng=__import__('random').Random(42);rig.prime(c)
    c.start(1.,.5,advance=advance)
    assert c.grid.at-rig.template.curve.down-advance>=1.-.5/3+.01-1e-9
    q,v=rig.anchor,0.;seen=0
    for i in range(60,4500):
        now=i*.002;command=c.update(rig.sample(now,q,v),now,-rig.anchor)
        q,v=command.position,command.velocity
        if c.status.strike_index>seen:
            seen=c.status.strike_index
            assert c.status.advance==advance
            assert c.status.released_at+rig.template.curve.down+advance==pytest.approx(c.status.scheduled_at,abs=.0021)
            assert c.status.scheduled_at==pytest.approx(1.+(4*(c.status.measure-1)+(c.status.beat-1)/2)*.5)
    assert seen>=4


def test_runtime_missing_detector_and_fault_use_stop_not_relax():
    a=app_fixture();a.root=Mock();a.audio_receiver=Mock()
    a._request_continuous_stop=Mock()
    r=SnareSyncRuntime(a,None)
    with pytest.raises(TimingFault,match='required'):r.start_calibration(11.)
    worker=Mock(status=SnareStatus(at=10.,error='lost feedback'))
    a.bus.left_joint6_session=worker
    r.engine=engine();r.last_observed=10.;a.continuous_strike_active=True
    with patch('camera_playback.snare_sync_runtime.time.monotonic',return_value=10.):r.tick()
    worker.finish.assert_called_once()
    a._request_continuous_stop.assert_called_once()
    a.bus.disable_all.assert_not_called()


def test_both_real_runtimes_and_worker_calibrate_then_fixed_offsets_at_user_bpm(rig):
    """Entire timing workflow, ideal servo and synthetic audio, zero physical I/O."""
    a=app_fixture();a.root=Mock();a._strike_bpm=120.
    a.strike_active=False;a.continuous_strike_active=True;a.continuous_strike_degrees=10.5
    a.audio_receiver=Mock();a.audio_receiver.ready.return_value=True
    a.hihat_sound_monitor=SimpleNamespace(receiver=Mock())
    a.hihat_sound_monitor.receiver.ready.return_value=True
    a.hihat_sync=hat=HiHatSyncRuntime(a)
    a.snare_sync=sn=SnareSyncRuntime(a,None)
    sn.receiver=Mock();sn.receiver.ready.return_value=True;sn.receiver.poll.return_value=[]
    c=rig.controller();c.rng=__import__('random').Random(42);rig.prime(c);now=.12;q,v=rig.anchor,0.
    pending=[];ride_next=None;ride_count=0;request_id=0;last_seen=0
    rows=[];hat.log=lambda row:rows.append(('hihat',row));sn.log=lambda row:rows.append(('snare',row))
    phases=[];calibrations=[];swing_started=False;checkpoint=None
    class Left:
        template=rig.template
        @property
        def status(self):return c.status
        @property
        def finished(self):return not c.status.scheduled and not c.status.moving and c.status.ready
        def start_calibration(self,epoch):c.start(epoch,SYNC_PERIOD,calibration=True)
        def set_calibration_advance(self,advance):c.set_advance(advance)
        def finish(self):c.finish()
        def start_swing(self,bpm,advance):
            nonlocal checkpoint
            checkpoint=advance
            c.start(now+1.,60/bpm,advance=advance)
    a.bus.left_joint6_session=Left()
    class Ride:
        status=HybridStatus(ready=True)
        def request(self,action,*args):
            nonlocal ride_next,ride_count,request_id,swing_started
            request_id+=1
            if action=='calibrate':
                ride_next=args[1];ride_count=0;calibrations.append(a.sync_instrument)
                self.status=replace(self.status,swing=True,calibrating=True)
            elif action=='finish':
                ride_next=None;self.status=replace(self.status,swing=False,ready=True)
            elif action=='swing':
                assert args[1]==swing_events(120)
                assert calibrations==['hihat','snare']
                swing_started=True;self.status=replace(self.status,swing=True,calibrating=False)
            self.status=replace(self.status,request_id=request_id)
            return request_id
    a.hybrid_session=ride=Ride()
    def close_hat(closed):
        if closed:
            assert a.sync_instrument=='hihat'
            pending.append((now+.1+.8,'hihat',now+.1))
    a.hihat.send_state.side_effect=close_hat
    with patch('camera_playback.hybrid_workflow.time.monotonic',side_effect=lambda:now), \
         patch('camera_playback.hihat_sync_runtime.Path.open',Mock()), \
         patch('camera_playback.hihat_sync_runtime.Path.mkdir'):
        hw.start_sync_calibration(a)
        for i in range(61,28000):
            now=i*.002
            command=c.update(rig.sample(now,q,v),now,-rig.anchor);q,v=command.position,command.velocity
            ride.status=replace(ride.status,at=now,sample=Sample(0.,0.,0.,2,now))
            if ride_next is not None and now>=ride_next:
                pending.append((ride_next+.6,'ride',ride_next));ride_next+=SYNC_PERIOD;ride_count+=1
                ride.status=replace(ride.status,bottoms=ride_count)
            if c.status.strike_index>last_seen:
                last_seen=c.status.strike_index
                assert a.sync_instrument=='snare'
                at=c.status.released_at+rig.template.curve.down+.055
                pending.append((at+.8,'snare',at))
            available=[p for p in pending if p[0]<=now];pending=[p for p in pending if p[0]>now]
            for _,name,at in available:
                msg=[dict(kind='hit',instrument=name,event_at=at)]
                hat.feed(name,msg);sn.feed(name,msg)
            for name,lag in [('ride',.6),('hihat',.8),('snare',.8)]:
                msg=[dict(kind='progress',instrument=name,finalized_at=now-lag-.004)]
                hat.feed(name,msg);sn.feed(name,msg)
            if i%5==0:
                hw.tick(a,now);hat.tick();sn.tick()
                phases.append(a.phase)
            assert not a.continuous_stop_requested, a.status.set.call_args
            if swing_started:break
        assert swing_started
        assert hw.SYNC_CALIBRATION in phases and hw.SNARE_SYNC_CALIBRATION in phases
        assert hat.calibrated_advance==pytest.approx(.1,abs=.004)
        assert sn.calibrated_advance==pytest.approx(.055,abs=.004)
        assert checkpoint==sn.calibrated_advance
        assert hat.engine.period==.5 and hat.engine.locked
        assert not sn.active and c.advance==checkpoint
        assert [name for name,row in rows if row['kind']=='locked']==['hihat','snare']
        a.bus.disable_all.assert_not_called()
        hw.request_stop(a)
        assert c.grid is None and not sn.active and not hat.active
        a.bus.disable_all.assert_not_called()


def test_calibration_large_reversals_latch_next_release_and_keep_full_return(rig):
    c=rig.controller();rig.prime(c);c.start(1.,SYNC_PERIOD,calibration=True)
    q,v=rig.anchor,0.;seen=0;previous=None
    for i in range(60,6000):
        now=i*.002
        command=c.update(rig.sample(now,q,v),now,-rig.anchor);q,v=command.position,command.velocity
        if c.status.strike_index>seen:
            seen=c.status.strike_index
            if previous:
                assert abs(c.status.advance-previous.advance)<=.2+1e-9
                assert c.status.released_at-previous.released_at>=minimum_hit_spacing(rig.template,rig.rules)-.003
            previous=c.status
            old_due=c.grid.at-rig.template.curve.down-c.scheduled_advance
            c.set_advance(.25 if seen%4<2 else -.25)
            assert c.grid.at-rig.template.curve.down-c.scheduled_advance==old_due
    assert seen>=12


@pytest.mark.parametrize('failure',['ride_audio','snare_audio','ride_status','snare_status'])
def test_detector_or_worker_staleness_never_continues_calibration(failure):
    a=app_fixture();a.root=Mock();a.continuous_strike_active=True
    a.audio_receiver=Mock();a.audio_receiver.ready.return_value=failure!='ride_audio'
    a._request_continuous_stop=Mock()
    worker=Mock(status=SnareStatus(at=9. if failure=='snare_status' else 10.))
    a.bus.left_joint6_session=worker
    r=SnareSyncRuntime(a,None);r.engine=engine()
    r.receiver=Mock();r.receiver.poll.return_value=[]
    r.receiver.ready.return_value=failure!='snare_audio'
    r.last_observed=9. if failure=='ride_status' else 10.
    with patch('camera_playback.snare_sync_runtime.time.monotonic',return_value=10.):r.tick()
    assert not r.active
    worker.finish.assert_called_once()
    a._request_continuous_stop.assert_called_once()
    a.bus.disable_all.assert_not_called()


def test_control_receiver_drains_before_calibration_without_tuning_or_moving(tmp_path):
    import tempfile
    from pathlib import Path
    from camera_playback.audio import AudioSender
    a=app_fixture();a.root=Mock()
    with tempfile.TemporaryDirectory() as directory:
        r=SnareSyncRuntime(a,Path(directory)/'snare.sock')
        sender=AudioSender(r.receiver.path,'snare')
        try:
            # Many startup/ready messages over time; none can clog the socket
            # until the much later snare stage. No calibration is active yet.
            for i in range(100):
                with patch('camera_playback.audio.time.monotonic',return_value=10.+i):
                    sender.status('ready')
                    sender.progress(9.+i,9.5+i)
                    r.poll()
                    assert r.receiver.ready(10.+i)
            assert r.engine is None and r.calibrated_advance is None
            a.bus.disable_all.assert_not_called()
        finally:sender.close();r.close()


def test_main_audio_tick_polls_snare_control_even_while_inactive():
    from camera_playback.app import App
    a=app_fixture();a.root=Mock();a.sound_fault_detail=None
    a.audio_receiver=Mock(state='ready');a.audio_receiver.poll.return_value=[]
    a.audio_receiver.ready.return_value=True
    a.hihat_sync=Mock();a.snare_sync=Mock(active=False)
    a.sound_status=Mock();a.sound_label=Mock()
    a._sound_required_now=Mock(return_value=False)
    App.sound_tick(a)
    a.snare_sync.poll.assert_called_once()
    a.snare_sync.feed.assert_called_once_with('ride',[])
