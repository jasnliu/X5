"""Passive Linux lock-owner diagnostics. No sockets, signals or motor commands."""
import errno
import os
from pathlib import Path
import shlex

BUSES=('can1','can0')


class BusOwnershipError(BlockingIOError):
    pass


def owners(*,proc=Path('/proc'),lock_dir=Path('/tmp')):
    identities={}
    for bus in BUSES:
        try:s=(lock_dir/f'openarmx-cartesian-{bus}.lock').stat()
        except FileNotFoundError:continue
        identities[os.major(s.st_dev),os.minor(s.st_dev),s.st_ino]=bus
    try:lines=(proc/'locks').read_text().splitlines()
    except OSError:return []
    found=[]
    for line in lines:
        fields=line.split()
        if len(fields)<8 or fields[1]!='FLOCK' or fields[3]!='WRITE':continue
        try:
            major,minor,inode=fields[5].split(':')
            bus=identities.get((int(major,16),int(minor,16),int(inode)))
            pid=int(fields[4])
        except (ValueError,IndexError):continue
        if bus is None:continue
        item=dict(bus=bus,pid=pid,resume=None)
        try:
            args=(proc/str(pid)/'cmdline').read_bytes().decode().rstrip('\0').split('\0')
            i=args.index('drum_planner.baseline.service')
            if i>0 and args[i-1]=='-m' and len(args)==i+2:
                item['resume']='./plan_drums.sh singlearc --resume '+shlex.quote(args[i+1])
        except (OSError,ValueError,UnicodeError):pass
        found.append(item)
    return found


def busy_error(found=None):
    found=owners() if found is None else found
    detail=', '.join(f"{x['bus']} (PID {x['pid']})" for x in found) or 'can0/can1'
    text='CAN interfaces are already owned by another controller: '+detail+'. '
    resumes=sorted({x['resume'] for x in found if x.get('resume')})
    if resumes:text+='Reconnect to the existing controller instead of starting a second one: '+' ; '.join(resumes)+'. '
    else:text+='Close the other CAN application before starting a new session. '
    return BusOwnershipError(errno.EAGAIN,text+'No motor commands were sent; the owner and lock files were left unchanged.')


def check_available():
    # Advisory early explanation only. Observer still takes the authoritative
    # exclusive locks when it opens CAN, so a racing owner is not bypassed.
    found=owners()
    if found:raise busy_error(found)
