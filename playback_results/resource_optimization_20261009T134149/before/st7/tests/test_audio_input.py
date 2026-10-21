"""Offline microphone routing checks; never open audio or robot devices."""
import json
import os
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from cymbal_detector.audio_input import TONOR_SOURCE, find_tonor_source, microphone_input
from cymbal_detector.listener import listen


def sources(text):
    return patch('cymbal_detector.audio_input.subprocess.run',
                 return_value=SimpleNamespace(stdout=text))


def test_tonor_uses_specific_shared_source_and_restores_environment():
    original = {'PULSE_SOURCE': 'original', 'PIPEWIRE_PROPS': 'original props'}
    with patch('sys.platform', 'linux'), patch.dict(os.environ, original, clear=True), \
            sources(f'1\t{TONOR_SOURCE}\tPipeWire\n') as run:
        with microphone_input('TONOR TD510 Dynamic Mic') as (device, source):
            assert device == 'pipewire'
            assert source == TONOR_SOURCE
            assert os.environ['PULSE_SOURCE'] == TONOR_SOURCE
            assert json.loads(os.environ['PIPEWIRE_PROPS'])['target.object'] == TONOR_SOURCE
        assert dict(os.environ) == original
        # Query only: never set-default-source or move another application's input.
        assert run.call_args.args[0] == ['pactl', 'list', 'short', 'sources']
        run.assert_called_once()


def test_environment_restored_on_open_failure():
    with patch('sys.platform', 'linux'), patch.dict(os.environ, {}, clear=True), \
            sources('1 alsa_input.usb-TONOR-new PipeWire\n'):
        with pytest.raises(RuntimeError, match='open failed'):
            with microphone_input('TONOR TD510 Dynamic Mic'):
                raise RuntimeError('open failed')
        assert 'PIPEWIRE_PROPS' not in os.environ
        assert 'PULSE_SOURCE' not in os.environ


@pytest.mark.parametrize('device', ['pipewire', 'pulse', 'Other USB Mic', '3', 3, None])
def test_explicit_non_tonor_routing_unchanged(device):
    with patch.dict(os.environ, {'PIPEWIRE_PROPS': 'X5 route'}), \
            patch('cymbal_detector.audio_input.find_tonor_source') as find:
        with microphone_input(device) as selected:
            assert selected == (device, None)
            assert os.environ['PIPEWIRE_PROPS'] == 'X5 route'
        find.assert_not_called()


def test_non_linux_selection_unchanged():
    with patch('sys.platform', 'darwin'), \
            patch('cymbal_detector.audio_input.find_tonor_source') as find:
        with microphone_input('TONOR TD510 Dynamic Mic') as selected:
            assert selected == ('TONOR TD510 Dynamic Mic', None)
        find.assert_not_called()


def test_stable_source_wins_and_monitor_is_never_selected():
    with sources(f'1 {TONOR_SOURCE}.monitor PipeWire\n'
                 f'2 {TONOR_SOURCE} PipeWire\n3 another-tonor PipeWire\n'):
        assert find_tonor_source() == TONOR_SOURCE


@pytest.mark.parametrize('listing, message', [
    ('1 alsa_input.other PipeWire\n', 'No TONOR'),
    (f'1 {TONOR_SOURCE}.monitor PipeWire\n', 'No TONOR'),
    ('1 tonor-one PipeWire\n2 tonor-two PipeWire\n', 'Multiple TONOR'),
])
def test_missing_or_ambiguous_source_fails_closed(listing, message):
    with sources(listing), pytest.raises(RuntimeError, match=message):
        find_tonor_source()


@pytest.mark.parametrize('error', [FileNotFoundError('pactl'),
                                 subprocess.TimeoutExpired('pactl', 5),
                                 subprocess.CalledProcessError(1, 'pactl')])
def test_unavailable_pipewire_has_actionable_error(error):
    with patch('cymbal_detector.audio_input.subprocess.run', side_effect=error), \
            pytest.raises(RuntimeError, match='PipeWire and pactl'):
        find_tonor_source()


def test_listener_routes_before_backend_import_and_does_not_claim_failed_open(capsys):
    import builtins
    original_import = builtins.__import__
    backend = SimpleNamespace(InputStream=Mock(side_effect=RuntimeError('open failed')))

    def guarded_import(name, *args, **kwargs):
        if name == 'sounddevice':
            assert os.environ['PULSE_SOURCE'] == TONOR_SOURCE
            return backend
        return original_import(name, *args, **kwargs)

    config = {'runtime': dict(block_seconds=.1, update_seconds=.25,
                             window_seconds=6., lookahead_seconds=.4, queue_seconds=2.)}
    detector = SimpleNamespace(sample_rate=16000, event_metadata={'instrument': 'ride'},
                               decoder=SimpleNamespace(as_dict=lambda: {}))
    normality = SimpleNamespace(metadata={'normality_version': 1})
    with patch('sys.platform', 'linux'), sources(f'1 {TONOR_SOURCE} PipeWire\n'), \
            patch('builtins.__import__', side_effect=guarded_import), \
            patch('cymbal_detector.listener.signal.signal'), \
            pytest.raises(RuntimeError, match='open failed'):
        listen(config, detector, normality, 'TONOR TD510 Dynamic Mic')
    assert backend.InputStream.call_args.kwargs['device'] == 'pipewire'
    assert backend.InputStream.call_args.kwargs['samplerate'] == 16000
    statuses = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert statuses == [{'status': 'stopped', 'captured_seconds': 0.0}]
