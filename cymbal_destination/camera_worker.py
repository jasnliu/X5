"""DT1-environment child: ONE paired snapshot, then camera OFF before inference.

No X5 numerical imports: DT1/Y2 NumPy 2 and X5 SciPy environments stay separate.
No CAN, serial, playback or actuator code. Sibling source/weights are read-only.
"""
import argparse,hashlib,json,os,socket,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
DT1=ROOT.parent/'dt1'
sys.path.insert(0,str(DT1))
sys.dont_write_bytecode=True

def guard(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise RuntimeError('Camera worker forbids CAN')
sys.addaudithook(guard)


def run(folder,calibration,replay=None):
    import numpy as np
    import cv2
    from dt1.config import load_calibration,sibling_paths,Y2
    from dt1.capture import Camera,FramePair,validate_metadata
    from dt1.pipeline import Replay
    from dt1.tracking import RideTracker
    from dt1.geometry import locate_target,above_target
    from dt1.surface import build_surface
    from dt1.segmentation import highlight_cymbal
    from dt1.disc import estimate_disc
    from dt1.app import target_dict,above_dict
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    cal,cal_path,cal_hash=load_calibration(calibration)
    camera=None;began=time.monotonic()
    try:
        if replay:
            source=Replay(replay,cal)
            pair=FramePair(dict(source.header,replay=True),source.color,source.depth);meta=source.meta
        else:
            camera=Camera(cal,folder);camera.start()
            while camera.snapshot() is None:
                if camera.error:raise RuntimeError(camera.error)
                if time.monotonic()-began>20:raise TimeoutError('No paired camera snapshot in 20 seconds')
                time.sleep(.01)
            pair=camera.snapshot();meta=camera.meta
            camera.close()
            if camera.error:raise RuntimeError(camera.error)
            if camera.thread.is_alive() or (camera.process and camera.process.poll() is None):
                raise RuntimeError('Camera did not close before calculation')
        validate_metadata(meta,cal)
        np.savez_compressed(folder/'pair.npz',color=pair.color,depth=pair.depth,
            meta=json.dumps(meta),header=json.dumps(pair.header),calibration=json.dumps(cal))
        (folder/'capture.json').write_text(json.dumps(dict(header=pair.header,camera=meta,
            calibration_sha256=cal_hash,camera_released=True,replay=bool(replay)),indent=2)+'\n')
        print('Paired snapshot saved; camera CLOSED. Detecting frozen target with DT1.',flush=True)
        sibling_paths()
        import torch,realtime_detector as detector
        from lighting_normalization import make_processor
        torch.set_num_threads(2);cv2.setNumThreads(2)
        model=detector.load_model(Y2/'models/snare.pt')
        if model.names.get(0)!='cymbal':raise ValueError('Expected DT1/Y2 cymbal class 0')
        image=make_processor('local').apply(pair.color)
        detections,elapsed=detector.InferenceWorker(model,640,.5,'cpu').infer(image)
        track=RideTracker().update(detections,0.)
        target=locate_target(pair.depth,pair.header['scale_m'],meta,cal['T_base_depth'],track.pixel,track.box,detections)
        mesh=build_surface(pair.depth,pair.header['scale_m'],meta,cal['T_base_depth'],step=4)
        highlight=highlight_cymbal(pair.depth,pair.header['scale_m'],meta,target,mesh)
        disc=estimate_disc(pair.depth,pair.header['scale_m'],meta,cal['T_base_depth'],highlight)
        above=above_target(target,meta,cal['T_base_depth'],disc)
        dependencies=[DT1/'dt1'/name for name in ('capture.py','tracking.py','geometry.py','segmentation.py','disc.py','surface.py')]
        dependencies.extend([Y2/'models/snare.pt',Y2/'realtime_detector.py',Y2/'lighting_normalization.py'])
        out=dict(schema='x5-cymbal-target-v1',valid=above is not None,calibration_sha256=cal_hash,
            target=target_dict(target),above_target=above_dict(above),flat_disc=disc.summary(),
            detections=detections,cymbal_highlight=highlight.summary(),
            single_frozen_pair=True,temporal_smoothing=False,inference_s=elapsed,
            snapshot_explanation='One pair; DT1 selection and geometry unchanged; no temporal box smoothing',
            dependencies_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in dependencies},
            motor_commands=0,can_access=False,elapsed_s=time.monotonic()-began)
        (folder/'target.json').write_text(json.dumps(out,indent=2)+'\n')
        rendered=detector.draw_detections(image,detections)
        if track.box is not None:
            x0,y0,x1,y1=track.box
            for k in (1,2):
                x=round(x0+(x1-x0)*k/3);y=round(y0+(y1-y0)*k/3)
                cv2.line(rendered,(x,round(y0)),(x,round(y1)),(255,180,0),1)
                cv2.line(rendered,(round(x0),y),(round(x1),y),(255,180,0),1)
        for p,color in ((target.pixel,(0,0,255)),(None if above is None else above.pixel,(0,255,0))):
            if p is not None:cv2.circle(rendered,tuple(np.rint(p).astype(int)),5,color,-1)
        cv2.imwrite(str(folder/'rgb.png'),rendered)
        np.savez_compressed(folder/'display.npz',vertices=mesh.vertices,faces=mesh.faces,
                            highlight=highlight.face_mask if highlight.face_mask is not None else np.zeros(len(mesh.faces),bool))
        if above is None:raise ValueError('No valid DT1 green target in this snapshot: '+str(out['target'].get('status')))
        print('Frozen target:',out['above_target']['world_m'],flush=True)
    finally:
        if camera:camera.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',required=True)
    p.add_argument('--calibration',required=True);p.add_argument('--replay');a=p.parse_args()
    for key,name in (('YOLO_CONFIG_DIR','yolo_config'),('MPLCONFIGDIR','matplotlib')):
        directory=Path(a.folder)/name;directory.mkdir(parents=True,exist_ok=True)
        os.environ[key]=str(directory)
    try:run(a.folder,a.calibration,a.replay)
    except Exception as e:
        Path(a.folder).mkdir(parents=True,exist_ok=True)
        (Path(a.folder)/'error.json').write_text(json.dumps(dict(error=str(e))))
        raise
