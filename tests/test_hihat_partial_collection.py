"""No physical devices: plan bounds, serial allowlist and fake-PTY collection."""
import json
import os
import pty
import select
import threading
import time
from unittest.mock import Mock,patch

import pytest

from hihat_collection.partial_close import (validate_plan,LoggedHiHat,Capture,ALLOWED,
                                           no_can,require_unused,zero_confirmed_open)


@pytest.mark.parametrize('value',[-.001,4.001,float('nan'),float('inf'),True,'0.2'])
def test_rejects_invalid_close_pulses(value):
    with pytest.raises(ValueError):validate_plan([dict(close_seconds=value)])


def test_collection_is_bounded_and_never_raises_target_above_90():
    assert validate_plan([dict(close_seconds=v) for v in [0,.04,.1,.3,.65,2.,3.,4.]])
    with pytest.raises(ValueError):validate_plan([dict(close_seconds=.1)]*41)
    assert ALLOWED == {b'Q',b'H',b'S',b'O',b'B',b'A90\n',b'V'} | {f'P{i}\n'.encode() for i in range(1,91)}
    d=LoggedHiHat('unused',lambda *a,**k:None)
    for command in [b'K',b'C',b'J',b'A91\n',b'A115\n',b'A100\n',b'A80\n',b'P0\n',b'P91\n',b'P80BK\n']:
        with pytest.raises(RuntimeError,match='Only hi-hat'):d._write(command)


def test_can_and_competing_owners_are_rejected():
    import socket
    with pytest.raises(RuntimeError,match='CAN'):no_can('socket.__new__',(None,socket.AF_CAN))
    with patch('subprocess.run',return_value=Mock(returncode=0,stdout='12345')):
        with pytest.raises(RuntimeError,match='already has an owner'):require_unused('/fake')


class Firmware:
    def __init__(self):
        self.master,self.slave=pty.openpty();self.port=os.ttyname(self.slave)
        self.stop=threading.Event();self.commands=[];self.pos=0;self.released=1;self.opened=1;self.fault=0;self.target=0
        self.thread=threading.Thread(target=self.run);self.thread.start()
    def run(self):
        angle=False;buffer=b'';prefix=b''
        while not self.stop.is_set():
            if not select.select([self.master],[],[],.01)[0]:continue
            for c in os.read(self.master,4096):
                if angle:
                    buffer+=bytes([c])
                    if c==10:
                        self.commands.append(prefix+buffer);angle=False
                        if prefix==b'A':os.write(self.master,b'ANGLE degrees=90 counts=134\n')
                        else:
                            degrees=int(buffer);self.target=int(28*19.2*degrees/360+.5);self.pos=self.target;self.released=0;self.opened=0
                            os.write(self.master,f'PARTIAL degrees={degrees} counts={self.target}\n'.encode())
                        buffer=b''
                    continue
                command=bytes([c])
                if command in (b'A',b'P'):angle=True;prefix=command;continue
                self.commands.append(command)
                if command==b'B':self.pos=80;self.released=0;self.opened=0;self.target=134
                if command==b'V':os.write(self.master,b'COLLECTION v=1 min=1 max=90 guard=100\n')
                if command==b'O':self.pos=0;self.released=1;self.opened=1;self.target=0
                if command==b'S':
                    self.released=1;self.target=0
                    os.write(self.master,b'STOP -- all motors released.\n')
                if command==b'Q':
                    os.write(self.master,f'HIHAT v=2 min=90 max=115 angle=90 counts=134 pos={self.pos} target={self.target} released={self.released} opened={self.opened} fault={self.fault}\n'.encode())
    def close(self):
        self.stop.set();self.thread.join();os.close(self.master);os.close(self.slave)


def test_real_collector_on_fake_serial_never_opens_arm_or_kicks(tmp_path):
    firmware=Firmware();c=Capture(tmp_path,[dict(close_seconds=.08)],firmware.port)
    real_wait=c.wait
    try:
        with patch.object(c,'start_audio'),patch('hihat_collection.partial_close.require_unused'),\
             patch.object(c,'wait',side_effect=lambda duration,*args:real_wait(min(duration,.08),*args)):
            result=c.run()
        assert result['error'] is None and result['release_ack']
        assert result['completed_trials']==1 and not result['arm_access']
        assert b'B' in firmware.commands and b'O' in firmware.commands
        assert set(firmware.commands)<=ALLOWED
        assert firmware.released==1 and firmware.opened==1 and firmware.pos==0
        assert json.loads((tmp_path/'trials.json').read_text())[0]['peak_degrees']<90
    finally:firmware.close()


def test_initial_fault_does_not_command_close_or_clear_it_by_reset(tmp_path):
    firmware=Firmware();firmware.fault=1
    c=Capture(tmp_path,[dict(close_seconds=.08)],firmware.port)
    try:
        with patch.object(c,'start_audio'),patch('hihat_collection.partial_close.require_unused'):
            with pytest.raises(RuntimeError,match='fault'):c.run()
        assert b'B' not in firmware.commands and b'O' not in firmware.commands
        assert firmware.fault==1 and firmware.released==1
    finally:firmware.close()


def test_zero_refuses_active_or_faulted_pose_before_any_reset():
    firmware=Firmware();firmware.fault=1
    try:
        with patch('hihat_collection.partial_close.require_unused'):
            with pytest.raises(RuntimeError,match='Refusing zero reset'):zero_confirmed_open(firmware.port)
        assert set(firmware.commands)=={b'Q'}
    finally:firmware.close()


@pytest.mark.parametrize('target',[0,91,115,True,20.5,float('nan')])
def test_rejects_invalid_shallow_targets(target):
    with pytest.raises(ValueError):validate_plan([dict(close_seconds=2,target_degrees=target)])


def test_shallow_hold_requires_capability_ack_and_returns_open(tmp_path):
    firmware=Firmware();c=Capture(tmp_path,[dict(close_seconds=2,target_degrees=55,provisional_label='uncertain')],firmware.port)
    real_wait=c.wait
    try:
        with patch.object(c,'start_audio'),patch('hihat_collection.partial_close.require_unused'),\
             patch.object(c,'wait',side_effect=lambda duration,*args:real_wait(min(duration,.08),*args)):
            result=c.run()
        assert result['error'] is None and result['release_ack'] and result['max_target_degrees']==55
        assert b'P55\n' in firmware.commands and b'B' not in firmware.commands
        assert set(firmware.commands)<=ALLOWED and firmware.opened and firmware.released
    finally:firmware.close()


@pytest.mark.parametrize('position,released,should_fail',[(-16,1,False),(-16,0,True),(150,1,True),(150,0,True)])
def test_passive_open_coast_is_not_powered_overtravel(tmp_path,position,released,should_fail):
    c=Capture(tmp_path,[dict(close_seconds=2,target_degrees=84)],'unused')
    c.driver=Mock(state='ready',telemetry={'pos':position,'released':released},telemetry_at=time.monotonic())
    c.driver.ready.return_value=False
    try:
        if should_fail:
            with pytest.raises(RuntimeError,match='travel guard'):c.tick()
            c.driver._write.assert_called_once_with(b'S')
        else:c.tick();c.driver._write.assert_not_called()
    finally:c.events.close()
