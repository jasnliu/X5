"""DT1-environment camera child: five native pairs, then release the device.

No inference, CAN, or X5 numerical imports. The last pair supplies DT1's RGB
target; all five depths supply the unchanged temporally confirmed scene.
"""
import argparse,json,socket,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT.parent/'dt1'))
sys.dont_write_bytecode=True


def guard(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise RuntimeError('Experiment camera child forbids CAN')


def run(folder,cal_path):
    import numpy as np
    from dt1.capture import Camera
    from dt1.config import load_calibration
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    cal,_,sha=load_calibration(cal_path);camera=Camera(cal,folder);pairs=[]
    began=time.monotonic()
    try:
        camera.start()
        while len(pairs)<5:
            if camera.error:raise RuntimeError(camera.error)
            if time.monotonic()-began>20:raise TimeoutError('Five paired native camera frames timed out')
            pair=camera.snapshot()
            if pair is not None and (not pairs or pair.header['sequence']>pairs[-1].header['sequence']):
                # Drop a discontinuous window, never pretend missing samples exist.
                if pairs and pair.header['received']-pairs[-1].header['received']>.25:pairs=[]
                pairs.append(pair)
            time.sleep(.005)
        camera.close()
        if camera.error:raise RuntimeError(camera.error)
        if camera.thread.is_alive() or (camera.process and camera.process.poll() is None):
            raise RuntimeError('Paired camera failed to release')
        last=pairs[-1];meta=camera.meta
        np.savez_compressed(folder/'pair.npz',color=last.color,depth=last.depth,
            header=json.dumps(last.header),meta=json.dumps(meta),calibration=json.dumps(cal))
        np.savez_compressed(folder/'paired_depths.npz',depth=np.stack([p.depth for p in pairs]))
        (folder/'paired_capture.json').write_text(json.dumps(dict(headers=[p.header for p in pairs],
            camera=meta,calibration_sha256=sha,camera_released=True,elapsed_s=time.monotonic()-began),indent=2)+'\n')
        print('Five native RGB/depth pairs captured; camera CLOSED. Last pair retained for dt1.',flush=True)
    finally:camera.close()


if __name__=='__main__':
    sys.addaudithook(guard)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',required=True);parser.add_argument('--calibration',required=True)
    args=parser.parse_args()
    try:run(args.folder,args.calibration)
    except Exception as exc:
        Path(args.folder).mkdir(parents=True,exist_ok=True)
        (Path(args.folder)/'error.json').write_text(json.dumps(dict(error=str(exc))))
        raise
