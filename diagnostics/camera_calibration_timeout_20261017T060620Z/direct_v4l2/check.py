import json
from pathlib import Path
import socket
import sys
import time
sys.path.insert(0, '/home/jason/Proyectos3/X5')
def audit(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.PF_CAN:
        raise RuntimeError('No CAN in camera diagnosis')
sys.addaudithook(audit)
import cv2
from camera_calibration.jpeg import JpegDecoder, CorruptJpegError
p=Path(__file__).resolve().parent
cap=cv2.VideoCapture('/dev/video4', cv2.CAP_V4L2)
assert cap.isOpened()
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cap.set(cv2.CAP_PROP_FPS,15); cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
print('Profile',cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT),cap.get(cv2.CAP_PROP_FPS),cap.get(cv2.CAP_PROP_CONVERT_RGB),flush=True)
d=JpegDecoder(); records=[]; start=time.monotonic(); saved=0
try:
 while time.monotonic()-start<20:
  t=time.monotonic(); ok, a=cap.read(); end=time.monotonic()
  if not ok:
   records.append(dict(ok=False,wait=end-t)); break
  b=a.tobytes(); r=dict(ok=True,shape=a.shape,size=len(b),wait=end-t,soi=b[:2].hex(),eoi=b[-2:].hex(),eoi_position=b.rfind(b'\xff\xd9'))
  try:
   im=d.decode(b); r['accepted']=True
   if not records: cv2.imwrite(str(p/'first_clean.png'),im)
  except Exception as e:
   r['accepted']=False; r['error']=str(e)
   if saved<3:
    (p/f'bad_{saved}.jpg').write_bytes(b); saved+=1
  records.append(r)
finally:
 cap.release()
(p/'frames.json').write_text(json.dumps(records,indent=2))
print(json.dumps(dict(frames=len(records),accepted=sum(r.get('accepted',False) for r in records),elapsed=time.monotonic()-start,first=records[:2],last=records[-2:]),indent=2))
