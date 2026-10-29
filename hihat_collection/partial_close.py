"""Finite HI-HAT ONLY partial-close capture. No arm/CAN, kick, or firmware flash.

Explicit target_degrees uses the optional P1..90 collection command to HOLD
shallow positions. Legacy plans without that field retain B90/early O behavior.
Command/encoder timing is provenance, not acoustic ground truth. Labels/export
are separate offline work. With no --execute this only checks the supplied plan.
"""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import termios
import time

from camera_playback.hihat import HiHatController, DEFAULT_ESP_PORT, configure_serial

ROOT = Path(__file__).resolve().parents[1]
TONOR = 'alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo'
ALLOWED = {b'Q', b'H', b'S', b'O', b'B', b'A90\n', b'V'} | {f'P{i}\n'.encode() for i in range(1,91)}
COUNTS_PER_DEGREE = 28*19.2/360


def save(path, obj):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(obj,indent=2)+'\n');tmp.replace(path)


def validate_plan(obj):
    if not isinstance(obj,list) or not 1 <= len(obj) <= 40:
        raise ValueError('One bounded capture must have 1..40 trials')
    for item in obj:
        pulse=item['close_seconds']
        if isinstance(pulse,bool) or not isinstance(pulse,(int,float)) or not 0 <= pulse <= 4.:
            raise ValueError('Close interval must be 0..4 seconds')
        if 'target_degrees' in item and (type(item['target_degrees']) is not int or not 1 <= item['target_degrees'] <= 90):
            raise ValueError('Collection target must be an integer 1..90 degrees')
        if item.get('provisional_label','uncertain') not in ('half_close','full_close','uncertain','quiet'):
            raise ValueError('Explicit provisional label must be half_close, full_close, uncertain or quiet')
        if not 4 <= item.get('tail_seconds',4.5) <= 8:
            raise ValueError('Use 4..8 seconds of independent decay per trial')
    return obj


def no_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise RuntimeError('Arm CAN access forbidden in hi-hat-only collection')


def require_unused(port):
    result=subprocess.run(['fuser',str(port)],capture_output=True,text=True)
    if result.returncode not in (0,1) or result.stdout.strip():
        raise RuntimeError('ESP32 port already has an owner; do not run another controller during collection')


def zero_confirmed_open(port):
    """Reset encoder ONLY at operator-confirmed relaxed open pose, no flash.

    Pulse EN using the Espressif HardReset sequence, keeping GPIO0 deasserted.
    https://github.com/espressif/esptool/blob/v5.0.0/esptool/reset.py
    Refuse a fault/active motor rather than reset away its fault evidence.
    """
    require_unused(port)
    fd=os.open(port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
    def lines(seconds):
        end=time.monotonic()+seconds;data=bytearray()
        while time.monotonic()<end:
            if select.select([fd],[],[],.01)[0]:data.extend(os.read(fd,4096))
        return data.decode('utf8','replace')
    def status():
        os.write(fd,b'Q');raw=lines(.18)
        values=[dict(token.split('=') for token in line.split()[1:])
                for line in raw.splitlines() if line.startswith('HIHAT ')]
        if not values:raise RuntimeError('Missing ESP32 status before/after zero reset')
        return {k:int(v) for k,v in values[-1].items()}
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        fcntl.ioctl(fd,termios.TIOCEXCL);configure_serial(fd)
        before=status()
        if before['fault'] or not before['released'] or before['target']!=0 or not before['opened']:
            raise RuntimeError('Refusing zero reset: motor must already be released/open and fault-free')
        os.write(fd,b'S')
        original=struct.unpack('I',fcntl.ioctl(fd,termios.TIOCMGET,struct.pack('I',0)))[0]
        idle=original & ~(termios.TIOCM_DTR|termios.TIOCM_RTS)
        fcntl.ioctl(fd,termios.TIOCMSET,struct.pack('I',idle|termios.TIOCM_RTS))
        time.sleep(.1)
        fcntl.ioctl(fd,termios.TIOCMSET,struct.pack('I',idle))
        boot=lines(.8)
        after=status()
        if 'READY motor_beat' not in boot or after['pos']!=0 or after['fault'] or not after['released']:
            raise RuntimeError('Open-pose encoder reset was not verified; no closing command sent')
        return dict(before=before,after=after,boot=boot,firmware_flashed=False)
    finally:os.close(fd)


class LoggedHiHat(HiHatController):
    def __init__(self,port,log):
        super().__init__(port);self.log=log;self.release_ack=None;self.partial_ack=None;self.collection_capable=False
    def _write(self,command):
        if command not in ALLOWED:raise RuntimeError('Only hi-hat collection commands up to 90 degrees are allowed')
        super()._write(command)
        self.log('tx',command=command.decode())
    def _handle_line(self,line):
        self.log('rx',line=line)
        if line.startswith('STOP -- all motors released'):self.release_ack=time.monotonic()
        if line=='COLLECTION v=1 min=1 max=90 guard=100':self.collection_capable=True
        if line.startswith('PARTIAL '):
            try:
                v={k:int(n) for k,n in (s.split('=') for s in line.split()[1:])}
                if set(v)!={'degrees','counts'} or not 1<=v['degrees']<=90 or v['counts']!=self.counts_for(v['degrees']):
                    raise ValueError('Invalid partial target acknowledgment')
                self.partial_ack=(v['degrees'],v['counts'],time.monotonic())
            except ValueError as exc:self._fail(str(exc))
        super()._handle_line(line)


class Capture:
    def __init__(self,folder,plan,port):
        self.folder,self.plan,self.port=folder,validate_plan(plan),port
        self.events=(folder/'events.jsonl').open('w',buffering=1)
        self.driver=LoggedHiHat(port,self.log);self.audio=None;self.audio_log=None
        self.cancelled=False;self.next_query=0.;self.last_tick=None;self.audio_check=0.
        self.trials=[];self.observed=[];self.seen_stamp=None
    def log(self,kind,**data):
        self.events.write(json.dumps(dict(monotonic=time.monotonic(),kind=kind,**data))+'\n')
    def tick(self,check_cancel=True):
        now=time.monotonic()
        if check_cancel and self.cancelled:raise KeyboardInterrupt('Operator cancelled')
        self.driver.tick(now)
        if self.driver.state=='error':raise RuntimeError(self.driver.detail)
        if self.driver.ready() and now>=self.next_query:
            self.driver.query_status();self.next_query=now+.02
        if self.driver.telemetry is not None:
            if now-self.driver.telemetry_at>.2:raise RuntimeError('Hi-hat encoder telemetry stale')
            v=self.driver.telemetry
            # Firmware releases at the observed zero crossing. The unpowered
            # linkage may coast past it (existing v2 behavior); this is not
            # powered negative travel. Keep both powered-travel guards intact.
            if v['pos']>int(100*COUNTS_PER_DEGREE) or (v['pos'] < -15 and not v['released']):
                self.driver._write(b'S');self.driver._fail('Measured hi-hat travel guard')
                raise RuntimeError('Measured hi-hat travel guard; released, no further trials')
            if self.driver.telemetry_at!=self.seen_stamp:
                self.observed.append(dict(at=self.driver.telemetry_at,**v));self.seen_stamp=self.driver.telemetry_at
        if self.audio is not None and now>=self.audio_check:
            if self.audio.poll() is not None:raise RuntimeError('Audio capture stopped')
            path=self.folder/'audio_status.json'
            if path.exists():
                status=json.loads(path.read_text())
                if status['errors'] or now-status['heartbeat']>1.:raise RuntimeError('Audio capture continuity failure')
            self.audio_check=now+.2
        time.sleep(.002)
    def wait(self,seconds,check_cancel=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.tick(check_cancel)
    def until(self,condition,timeout,check_cancel=True):
        end=time.monotonic()+timeout
        while not condition():
            self.tick(check_cancel)
            if time.monotonic()>end:raise RuntimeError('Hi-hat readiness/return timeout')
    def start_audio(self):
        sources=json.loads(subprocess.check_output(['pactl','-f','json','list','sources']))
        source=next(s for s in sources if s['name']==TONOR)
        name='hihat-partial-'+self.folder.name
        env=dict(os.environ,PULSE_SOURCE=TONOR,PA_ALSA_PLUGHW='1',
                 PIPEWIRE_PROPS=json.dumps({'application.name':name,'target.object':TONOR}))
        self.audio_log=(self.folder/'audio_capture.log').open('w')
        self.audio=subprocess.Popen([str(ROOT.parent/'st7/.venv/bin/python'),'-m','x5_collection.audio_capture',str(self.folder)],
            cwd=ROOT,env=env,stdout=self.audio_log,stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+8
        while not (self.folder/'audio_status.json').exists():
            if self.audio.poll() is not None or time.monotonic()>end:raise RuntimeError('TONOR capture startup failed')
            time.sleep(.05)
        outputs=json.loads(subprocess.check_output(['pactl','-f','json','list','source-outputs']))
        selected=[s for s in outputs if s.get('properties',{}).get('application.name')==name]
        if not selected or any(s['source']!=source['index'] for s in selected):
            raise RuntimeError('Capture was not routed to the specified TONOR microphone')
        save(self.folder/'microphone.json',dict(source=source,streams=selected))
    def open_return(self,check_cancel=True):
        self.driver.calibration_edge(False);sent=time.monotonic()
        self.until(lambda:self.driver.telemetry_at is not None and self.driver.telemetry_at>sent
                   and self.driver.telemetry['released']==1 and self.driver.telemetry['opened']==1,
                   2.5,check_cancel)
        self.wait(.5,check_cancel)
    def run(self):
        error=None;released=False
        try:
            self.start_audio()
            require_unused(self.port)
            if not self.driver.connect():raise RuntimeError(self.driver.detail)
            fcntl.flock(self.driver.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            fcntl.ioctl(self.driver.fd,termios.TIOCEXCL)
            self.until(lambda:self.driver.ready() and self.driver.telemetry is not None,4)
            if self.driver.telemetry['fault'] or not self.driver.telemetry['released'] or not self.driver.telemetry['opened']:
                raise RuntimeError('Initial hi-hat is not fault-free, open and released')
            save(self.folder/'initial_status.json',self.driver.telemetry)
            self.driver.set_calibration_angle(90)
            self.until(lambda:self.driver.angle_ack is not None,1)
            if self.driver.angle_ack[:2]!=(90,134):raise RuntimeError('90 degree target readback mismatch')
            if any('target_degrees' in item for item in self.plan):
                self.driver._write(b'V')
                self.until(lambda:self.driver.collection_capable,1)
            for index,item in enumerate(self.plan,1):
                self.wait(1.5);start=time.monotonic();offset=len(self.observed)
                print(f'HIHAT ONLY trial {index}/{len(self.plan)}: {item["close_seconds"]:.3f}s at {item.get("target_degrees",90)} degrees; provisional label {item.get("provisional_label","uncertain")}',flush=True)
                self.log('trial_start',trial=index,**item)
                if item['close_seconds']:
                    if 'target_degrees' in item:
                        target=item['target_degrees'];self.driver.partial_ack=None
                        self.driver._write(f'P{target}\n'.encode())
                        self.until(lambda:self.driver.partial_ack is not None,.3)
                        if self.driver.partial_ack[:2]!=(target,self.driver.counts_for(target)):
                            raise RuntimeError('Collection target readback mismatch')
                        self.driver.motor2_closed=True
                    else:self.driver.calibration_edge(True)
                    self.wait(max(0,item['close_seconds']-(time.monotonic()-start)))
                    self.open_return()
                self.wait(item.get('tail_seconds',4.5))
                values=self.observed[offset:];record=dict(trial=index,start_monotonic=start,
                    end_monotonic=time.monotonic(),**item,telemetry=values,
                    peak_degrees=max((r['pos']/COUNTS_PER_DEGREE for r in values),default=0.))
                self.trials.append(record);save(self.folder/'trials.json',self.trials)
            self.log('capture_complete')
        except BaseException as exc:
            error=f'{type(exc).__name__}: {exc}';self.log('failure',error=error)
        finally:
            shutdown_error=None
            try:
                if self.driver.fd is not None:
                    if self.driver.ready() and self.driver.telemetry and not self.driver.telemetry['released']:
                        self.open_return(False)
                    self.driver._write(b'S');sent=time.monotonic();end=sent+.6
                    while time.monotonic()<end:
                        self.driver._drain()
                        if self.driver.release_ack and self.driver.release_ack>=sent:released=True;break
                        time.sleep(.005)
                    if not released:raise RuntimeError('Release acknowledgement missing')
            except BaseException as exc:
                shutdown_error=str(exc)
                if self.driver.fd is not None:
                    try:self.driver._write(b'S')
                    except Exception:pass
            self.driver.close()
            if self.audio is not None:
                if self.audio.poll() is None:self.audio.send_signal(signal.SIGINT)
                try:self.audio.wait(timeout=3)
                except subprocess.TimeoutExpired:self.audio.terminate();self.audio.wait(timeout=3)
            if self.audio_log:self.audio_log.close()
            result=dict(error=error,shutdown_error=shutdown_error,release_ack=released,
                completed_trials=len(self.trials),max_target_degrees=max(i.get('target_degrees',90) for i in self.plan),measured_guard_degrees=100,
                arm_access=False,kick_commands=0,firmware_changed=False)
            save(self.folder/'result.json',result);self.events.close()
        if error or shutdown_error:raise RuntimeError(error or shutdown_error)
        return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--folder',type=Path)
    p.add_argument('--execute',action='store_true');p.add_argument('--zero-confirmed-open',action='store_true')
    p.add_argument('--port',default=DEFAULT_ESP_PORT)
    args=p.parse_args();plan=validate_plan(json.loads(args.plan.read_text()))
    if not args.execute:print(json.dumps(dict(trials=len(plan),plan=plan,motor_commands=0)));return
    if args.folder is None:p.error('--folder required for capture')
    sys.addaudithook(no_can)
    folder=args.folder.resolve();folder.mkdir(parents=True,exist_ok=False)
    save(folder/'plan.json',plan)
    helper=Path.home()/'.local/bin/codex-ntfy.py'
    spec=importlib.util.spec_from_file_location('partial_warning',helper);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    delivered=module.post_ntfy('Hi-hat-only audio collection starts in 10 seconds. Keep the mechanism clear.',
                              'Hardware collection','warning')
    save(folder/'notification.json',dict(delivered=delivered))
    if not delivered:raise RuntimeError('Motion notification failed; no motor command sent')
    time.sleep(10)
    if args.zero_confirmed_open:save(folder/'zero_reference.json',zero_confirmed_open(args.port))
    capture=Capture(folder,plan,args.port)
    def stop(*_):capture.cancelled=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print(json.dumps(capture.run(),indent=2))


if __name__=='__main__':main()
