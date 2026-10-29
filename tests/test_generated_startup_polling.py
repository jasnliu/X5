"""Do not confuse ideal all-at-once encoders with actual request/reply cadence."""
from unittest.mock import patch
import numpy as np
import pytest
from camera_playback.generated_transport import GeneratedMotors
from camera_playback.generated_motion import Motion
from tests.generated_packet_device import PacketDevice
from tests.test_generated_stream import make_curve


def test_fake_device_does_not_erase_unread_replies_when_new_frame_is_pending():
    from safe_zone.encoder import request_frame
    t=[1.];d=PacketDevice({s:np.zeros(7) for s in ('left','right')},clock=lambda:t[0])
    sock=d.sockets['left'];sock.send(request_frame(1));t[0]+=.005
    sock.send(request_frame(1))  # New pending reply, previous already delivered.
    assert len(sock.recv(16))==16
    with pytest.raises(BlockingIOError):sock.recv(16)
    t[0]+=.002;assert len(sock.recv(16))==16


def test_worker_handoff_queries_oldest_actual_feedback_not_old_parent_schedule():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from camera_playback.generated_can import FeedbackQueries
    from safe_zone.encoder import request_frame
    schedule=FeedbackQueries(.020,max_requests=4)
    schedule.sent={('right',i):1. if i<=4 else 9. for i in range(1,9)}
    sock=Mock(send=Mock(return_value=16))
    bus=SimpleNamespace(sockets={'right':sock},states={('right',i):(0,2,9.95 if i<=4 else 9.905) for i in range(1,9)})
    with patch('time.monotonic',return_value=10.):schedule.poll(bus)
    assert [c.args[0] for c in sock.send.call_args_list]==[request_frame(i) for i in range(5,9)]


@pytest.mark.parametrize('period',[.020,.0335,.040,.050,.060,(.020,.060,.035,.045)])
def test_real_arming_with_request_only_feedback_at_gui_cadence(period):
    from itertools import cycle
    intervals=cycle(period if isinstance(period,tuple) else [period])
    t=[1.];rest={s:np.zeros(7) for s in ('left','right')}
    device=PacketDevice(rest,clock=lambda:t[0])
    bus=GeneratedMotors.from_owner(device.owner(),rest)
    motion=Motion(bus,{s:make_curve(s) for s in rest},clock=lambda:t[0])
    ages=[]
    with patch('time.monotonic',side_effect=lambda:t[0]):
        # Start from NO feedback, not an artificial fresh sixteen-drive batch.
        for _ in range(8):t[0]+=next(intervals);bus.poll()
        motion.start()
        for _ in range(700):
            t[0]+=next(intervals);bus.poll()
            ages.append(max(t[0]-v[2] for v in bus.states.values()))
            motion.step()
            if motion.stage=='LEFT':break
        assert motion.stage=='LEFT'
        assert bus.enabled_drives=={(s,i) for s in rest for i in range(1,9)}
        assert max(ages)<.120
        for s in rest:np.testing.assert_allclose(bus.positions_for(s),rest[s],atol=.0002)
        assert len([c for c in device.calls if c[2]==3])==16


def test_actual_missing_encoder_still_stops_arming_with_unchanged_deadline():
    t=[1.];rest={s:np.zeros(7) for s in ('left','right')}
    d=PacketDevice(rest,clock=lambda:t[0]);bus=GeneratedMotors.from_owner(d.owner(),rest)
    motion=Motion(bus,{s:make_curve(s) for s in rest},clock=lambda:t[0])
    with patch('time.monotonic',side_effect=lambda:t[0]):
        for _ in range(8):t[0]+=.0335;bus.poll()
        motion.start();d.drop.add(8+6)  # right J7 really stops replying
        with pytest.raises(RuntimeError,match='Fresh feedback missing'):
            for _ in range(20):t[0]+=.0335;bus.poll();motion.step()
        assert t[0]-bus.states['right',7][2]>.120
        assert len(bus.enabled_drives)<16
