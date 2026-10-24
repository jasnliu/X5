"""Device-free destination persistence, routing, recovery and compatibility tests."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import pytest

from destination_capture.feedback import Feedback, Snapshot, fresh
from destination_capture.store import DestinationStore
from motion_recording.recording import gripper_name, joint_names
from safe_zone.geometry import Model
from safe_zone.gripper_feedback import EncoderState, motor8_feedback

ROOT = Path(__file__).resolve().parents[1]


def state(side, offset=0.):
    values = {name: offset + .01 * (i + 1) for i, name in enumerate(joint_names(side))}
    values[gripper_name(side)] = .005
    return EncoderState(values, motor8_feedback={side: motor8_feedback(32890)})


def save(store, values=None, **overrides):
    metadata = dict(captured_at_utc='2026-10-21T12:34:56.000Z', feedback_age_s=.01,
                    batch_duration_s=.003, bus='can0' if store.side == 'right' else 'can1')
    metadata.update(overrides)
    return store.save(state(store.side) if values is None else values,
                      Model(ROOT / 'model/openarmx.urdf'), **metadata)


@pytest.mark.parametrize('side,letter', [('right', 'R'), ('left', 'L')])
def test_single_snapshot_is_compatible_with_existing_endpoint_loader(tmp_path, side, letter):
    from drum_planner.common import endpoint
    store = DestinationStore(tmp_path, side)
    path = save(store)
    record = json.loads(path.read_text())
    assert path == tmp_path / side / f'destination{letter}1.json'
    assert record['schema'] == f'openarmx-{side}-motion-recording-v1'
    assert record['arm'] == side
    assert record['capture_kind'] == 'destination'
    assert record['coordinate_convention'] == 'encoder_to_joint'
    assert record['captured_at_utc'] == record['started_at_utc']
    assert record['sample_count'] == len(record['samples']) == 1
    assert record['duration_s'] == record['samples'][0]['time_s'] == 0
    assert record['joint_order'] == list(joint_names(side))
    sample = record['samples'][0]
    assert sample['motor8_encoder_count'] == 32890
    assert sample['motor8_raw_rad'] == motor8_feedback(32890)['raw_rad']
    assert sample['gripper_opening_m'] == .005
    assert record['capture']['motor_state_commands'] == 0
    np.testing.assert_array_equal(endpoint(path, side=side), [.01 * i for i in range(1, 8)])
    with pytest.raises(ValueError, match='Incompatible'):
        endpoint(path, side='left' if side == 'right' else 'right')


def test_independent_numbering_restarts_gaps_and_unrelated_files(tmp_path):
    right = DestinationStore(tmp_path, 'right')
    first = save(right)
    original = first.read_bytes()
    assert save(right).name == 'destinationR2.json'
    (right.folder / 'destinationR9.json').write_text('preserve me')
    (right.folder / 'destinationL99.json').write_text('wrong side')
    (right.folder / 'notes.json').write_text('{}')
    assert save(DestinationStore(tmp_path, 'right')).name == 'destinationR10.json'
    assert save(DestinationStore(tmp_path, 'left')).name == 'destinationL1.json'
    assert first.read_bytes() == original
    assert (right.folder / 'destinationR9.json').read_text() == 'preserve me'


def test_concurrent_saves_publish_complete_distinct_files(tmp_path):
    def capture(i):
        return save(DestinationStore(tmp_path, 'right'), state('right', i * .001))
    with ThreadPoolExecutor(max_workers=4) as pool:
        paths = list(pool.map(capture, range(24)))
    assert len(set(paths)) == 24
    assert {p.name for p in paths} == {f'destinationR{i}.json' for i in range(1, 25)}
    assert len({json.loads(p.read_text())['samples'][0]['positions_rad'][0] for p in paths}) == 24
    assert not list((tmp_path / 'right').glob('*.tmp'))


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_values_are_not_written(tmp_path, bad):
    store = DestinationStore(tmp_path, 'right')
    values = state('right')
    values['openarmx_right_joint1'] = bad
    with pytest.raises(ValueError):
        save(store, values)
    assert not list(store.folder.iterdir())


def test_failed_publication_can_retry_and_does_not_consume_number(tmp_path, monkeypatch):
    from destination_capture import store as module
    store = DestinationStore(tmp_path, 'left')
    link = module.os.link
    def fail(*args):
        raise PermissionError('test output failure')
    monkeypatch.setattr(module.os, 'link', fail)
    with pytest.raises(PermissionError):
        save(store)
    assert not list(store.folder.iterdir())
    monkeypatch.setattr(module.os, 'link', link)
    assert save(store).name == 'destinationL1.json'


def test_legal_calibrated_angle_outside_original_urdf_is_saved_unchanged(tmp_path):
    values = state('right')
    values['openarmx_right_joint4'] = -.019756008239872003
    path = save(DestinationStore(tmp_path, 'right'), values)
    assert json.loads(path.read_text())['samples'][0]['positions_rad'][3] == values['openarmx_right_joint4']


def test_r4_joint7_is_rejected_without_writing_or_consuming_a_number(tmp_path):
    store = DestinationStore(tmp_path, 'right')
    first = save(store)
    before = {p.name: p.read_bytes() for p in store.folder.iterdir()}
    values = state('right')
    values['openarmx_right_joint7'] = 1.5192945754177156  # Reported R4 failure.
    with pytest.raises(ValueError, match=r'Right J7 out of range.*87\.05.*80\.21'):
        save(store, values)
    assert {p.name: p.read_bytes() for p in store.folder.iterdir()} == before
    assert store.next_number() == 2
    assert save(store).name == 'destinationR2.json'
    assert first.read_bytes() == before[first.name]


@pytest.fixture(scope='module', params=['right', 'left'])
def planner_limits(request, tmp_path_factory):
    # Compare to the actual generated planner model, not a duplicate expected
    # offset formula. This is source/model-only; no camera, CAN or FCL worker.
    from drum_planner.model import generate
    side = request.param
    folder = tmp_path_factory.mktemp('destination-planner-' + side)
    generate(folder, moving_side=side)
    root = ET.parse(folder / 'robot.urdf').getroot()
    bounds = []
    for name in joint_names(side):
        limit = root.find(f"joint[@name='{name}']/limit")
        bounds.append((float(limit.get('lower')), float(limit.get('upper'))))
    return side, bounds


@pytest.mark.parametrize('joint', range(7))
@pytest.mark.parametrize('bound', [0, 1])
def test_exact_planner_boundaries_save_unchanged(tmp_path, planner_limits, joint, bound):
    side, limits = planner_limits
    values = state(side)
    values[joint_names(side)[joint]] = limits[joint][bound]
    path = save(DestinationStore(tmp_path, side), values)
    actual = json.loads(path.read_text())['samples'][0]['positions_rad']
    assert actual == [values[name] for name in joint_names(side)]


@pytest.mark.parametrize('joint', range(7))
@pytest.mark.parametrize('bound,direction', [(0, -np.inf), (1, np.inf)])
def test_even_just_outside_planner_boundaries_is_not_saved(
        tmp_path, planner_limits, joint, bound, direction):
    side, limits = planner_limits
    values = state(side)
    values[joint_names(side)[joint]] = np.nextafter(limits[joint][bound], direction)
    store = DestinationStore(tmp_path, side)
    with pytest.raises(ValueError, match=f'{side.title()} J{joint + 1} out of range'):
        save(store, values)
    assert not list(store.folder.iterdir())
    assert store.next_number() == 1


@pytest.mark.parametrize('side', ['right', 'left'])
def test_only_selected_arm_limits_are_checked(tmp_path, side):
    values = state(side)
    other = 'left' if side == 'right' else 'right'
    values.update({name: 100. for name in joint_names(other)})
    assert save(DestinationStore(tmp_path, side), values).is_file()


def test_freshness_rejects_missing_old_and_future_feedback():
    sample = Snapshot(state('left'), 100., 'utc', .01)
    assert not fresh(None, 100.)
    assert fresh(sample, 100.1)
    assert not fresh(sample, 100.31)
    assert not fresh(sample, 99.)


@pytest.mark.parametrize('side,bus', [('right', 'can0'), ('left', 'can1')])
def test_observer_sends_only_query_packets_on_selected_bus(monkeypatch, side, bus):
    import safe_zone.encoder as encoder
    outgoing, opened, closed = [], [], []

    class Socket:
        def __init__(self):
            self.pending = []
        def bind(self, address):
            opened.append(address)
        def setsockopt(self, *args): pass
        def setblocking(self, value): pass
        def send(self, frame):
            outgoing.append(frame)
            motor = encoder.FRAME.unpack(frame)[0] & 255
            count = 32768 + 10 * motor
            self.pending.append(encoder.FRAME.pack(
                encoder.EFF | (2 << 24) | (motor << 8), 8,
                count.to_bytes(2, 'big') + b'\0' * 6))
            return 16
        def recv(self, size):
            if not self.pending:
                raise BlockingIOError()
            return self.pending.pop(0)
        def close(self): closed.append(True)

    original = encoder.socket
    monkeypatch.setattr(encoder, 'socket', SimpleNamespace(
        PF_CAN=original.PF_CAN, SOCK_RAW=original.SOCK_RAW, CAN_RAW=original.CAN_RAW,
        SOL_CAN_RAW=original.SOL_CAN_RAW, socket=lambda *a: Socket()))
    monkeypatch.setattr(encoder, 'os', SimpleNamespace(
        open=lambda *a: 123, close=lambda fd: None, O_CREAT=0, O_RDWR=0, O_NOFOLLOW=0))
    monkeypatch.setattr(encoder, 'fcntl', SimpleNamespace(flock=lambda *a: None, LOCK_EX=0, LOCK_NB=0))
    monkeypatch.setattr(encoder, 'select', SimpleNamespace(select=lambda r, w, x, t: (r, [], [])))
    observer = encoder.SingleArmObserver(side, bus)
    values = observer.sample()
    observer.close()
    assert opened == [(bus,)]
    assert outgoing == [encoder.request_frame(i) for i in range(1, 9)]
    assert closed == [True]
    for i, name in enumerate(joint_names(side), 1):
        raw = (32768 + 10 * i) / 65535 * 25.14 - 12.57
        assert values[name] == encoder.encoder_to_joint(side, i, raw)
    assert values.motor8_feedback[side]['encoder_count'] == 32848


def test_feedback_recovers_without_restart_and_closes_observers():
    opened, closed = [], []
    class Observer:
        def __init__(self, side, bus):
            opened.append((side, bus))
            self.fail = len(opened) == 1
        def sample(self):
            if self.fail:
                raise RuntimeError('temporary missing feedback')
            return state('left')
        def close(self): closed.append(True)
    reader = Feedback('left', 'can1', Observer)
    reader.thread.start()
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            latest, error = reader.read()
            if latest and error is None:
                break
            time.sleep(.01)
        assert fresh(reader.read()[0])
        assert reader.read()[1] is None
    finally:
        reader.close()
    assert opened == [('left', 'can1'), ('left', 'can1')]
    assert closed == [True, True]
    assert not reader.thread.is_alive()


def test_cli_side_selection_and_hardware_is_opt_in():
    from destination_capture.app import parse_args
    for side, bus in [('right', 'can0'), ('left', 'can1')]:
        args = parse_args(['--' + side])
        assert args.arm == side and args.can == bus and not args.hardware
        assert args.output_root == ROOT / 'destinations'
    for argv in [[], ['--right', '--left']]:
        with pytest.raises(SystemExit): parse_args(argv)
    assert parse_args(['--left', '--left-can', 'mycan']).can == 'mycan'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_display_hides_only_unmeasured_arm_and_preserves_original_model(side):
    from launch_destination import display_description
    original_bytes = (ROOT / 'model/openarmx.urdf').read_bytes()
    original = ET.fromstring(original_bytes)
    view = ET.fromstring(display_description(side))
    other = 'right' if side == 'left' else 'left'
    for old, new in zip(original.findall('link'), view.findall('link')):
        if old.get('name').startswith(f'openarmx_{other}_'):
            assert new.find('visual') is None and new.find('collision') is None
        else:
            assert ET.tostring(old) == ET.tostring(new)
    assert [ET.tostring(j) for j in original.findall('joint')] == [ET.tostring(j) for j in view.findall('joint')]
    assert (ROOT / 'model/openarmx.urdf').read_bytes() == original_bytes


@pytest.mark.parametrize('argv,code', [(['--help'], 0), ([], 2), (['--left', '--right'], 2)])
def test_launcher_help_and_errors_exit_without_windows(argv, code):
    result = subprocess.run([str(ROOT / 'start_destination.sh'), *argv], capture_output=True,
                            text=True, timeout=10)
    assert result.returncode == code
    assert 'process started' not in result.stdout
