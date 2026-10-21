"""Read-only saved-WAV/model-output replay plus clearly synthetic overlap probes.

Real Tk + Unix datagrams; no microphone, camera, motor or serial access.
Run after generating diagnostics/snare_visual/snare_file_inference.json.
"""
import json
from pathlib import Path
import socket
import sys
import tempfile
import time
import tkinter as tk
from unittest.mock import patch
import numpy as np
import soundfile as sf
from PIL import ImageGrab

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from camera_playback.audio import AudioSender
from camera_playback.audio_bridge import forward_audio_message
from camera_playback.audio_waveform import WaveformApp
from camera_playback.waveform import COLORS
OUT=ROOT/'diagnostics/snare_visual'

def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.AF_CAN:raise AssertionError('No physical CAN')
    if event=='open' and isinstance(args[0],str) and args[0].startswith(('/dev/tty','/dev/serial','/dev/video','/dev/snd')):
        raise AssertionError('No physical devices')
sys.addaudithook(audit)

def screenshot(root,name):
    root.update();time.sleep(.1);root.update()
    box=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT/name)

root=tk.Tk();app=None;senders=[]
try:
 with tempfile.TemporaryDirectory(prefix='x5-snare-ui-') as tmp:
    p=Path(tmp)
    app=WaveformApp(root,p/'ride',p/'hihat',snare_socket=p/'snare')
    data,sr=sf.read(ROOT/'X5data/snare/recordings/r1.wav',dtype='float32')
    assert sr==16000
    for i in range(0,len(data),320):app.timeline.add_audio(100.+i/sr,data[i:i+320])
    inference=json.loads((OUT/'snare_file_inference.json').read_text())
    assert inference['events']
    for instrument in ('ride','hihat','snare'):
        senders.append(AudioSender(p/instrument,instrument))
    for event in inference['events']:
        message=dict(event,event='HIT',event_monotonic=100.+event['time'])
        forward_audio_message(message,senders[-1],100.+event['time']+.7,'snare')
    with patch('camera_playback.audio_waveform.time.monotonic',return_value=110.):app.tick()
    root.after_cancel(app.after_id);app.after_id=None
    app.window.render(110.,'OFFLINE REPLAY · real snare_v1 output on preserved r1.wav · no live devices')
    screenshot(root,'snare_model_replay.png')
    markers=[dict(m) for m in app.window.drawn_markers]
    assert len(markers)==len(inference['events'])
    for marker,event in zip(markers,inference['events']):
        assert marker['event_at']==100.+event['time'] and marker['color']==COLORS['snare']
    # Overlap probes are NOT claims of classifier detections on this recording.
    onset=100.+inference['events'][0]['time']
    for sender in senders[:2]:sender.hit(onset,onset+.9,.99,80.)
    with patch('camera_playback.audio_waveform.time.monotonic',return_value=110.):app.tick()
    root.after_cancel(app.after_id);app.after_id=None
    app.window.render(110.,'OFFLINE OVERLAP PROBE · snare model hit + synthetic ride/hi-hat markers')
    screenshot(root,'three_instrument_overlap.png')
    overlap=[m for m in app.window.drawn_markers if m['event_at']==onset]
    assert len(overlap)==3 and len({m['x'] for m in overlap})==1
    for instrument in COLORS:
        items=app.window.canvas.find_withtag(instrument)
        labels=[app.window.canvas.itemcget(i,'text') for i in items if app.window.canvas.type(i)=='text']
        assert labels==[{'ride':'Ride','hihat':'Hi-hat','snare':'snare'}[instrument]]
        assert all(app.window.canvas.itemcget(i,'fill')==COLORS[instrument] for i in items)
    before=app.window.drawn_markers[0]['x']
    app.window.render(110.5,'OFFLINE SCROLL PROBE')
    assert app.window.drawn_markers[0]['x']<before
    root.geometry('700x360');root.update()
    app.window.render(110.5,'OFFLINE RESIZE PROBE')
    screenshot(root,'three_instrument_small.png')
    assert len(app.window.drawn_markers)==3
    (OUT/'ui_verification.json').write_text(json.dumps(dict(success=True,physical_io=False,
        real_snare_model_file_inference=True,original_snare_onsets_exact=True,
        coincidence_probe_synthetic_ride_hihat=True,three_colors_and_labels=True,
        labels_share_time_not_vertical_row=True,scroll_and_resize=True,markers=markers),indent=2)+'\n')
    print('PASS: snare_v1 saved-WAV output reaches real graph socket; purple snare labels, three-color overlap, onset placement, scroll and resize')
finally:
 if app:app.close()
 for sender in senders:sender.close()
 root.destroy()
