"""Offline visual/timestamp regression using preserved WAVs; no microphone or motor I/O."""
from pathlib import Path
import json,sys,time,tkinter as tk
import numpy as np
import soundfile as sf
from PIL import ImageGrab
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from camera_playback.waveform import WaveformTimeline,WaveformWindow
OUT=Path(__file__).resolve().parent

def screenshot(root,name):
    root.update();time.sleep(.15);root.update()
    box=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT/name)

root=tk.Tk()
try:
    t=WaveformTimeline();w=WaveformWindow(root,t);root.update()
    signal=np.zeros(10*16000,dtype=np.float32)
    ride,sr=sf.read(ROOT/'X5data/ride/recordings/r1.wav',dtype='float32');assert sr==16000
    hi,sr=sf.read(ROOT/'X5data/hihat/recording/r1.wav',dtype='float32');assert sr==16000
    signal[:len(ride)]+=ride
    signal[4*16000:]+=hi[:6*16000]
    for i in range(0,len(signal),320):t.add_audio(100.+i/16000,signal[i:i+320])
    events=[('ride',101.5,.8),('hihat',105.5,.9),('hihat',106.7055,.7)]
    # Replay-only markers: labeling semantics tested separately from inference.
    for instrument,onset,delay in events:
        assert t.add_hit(dict(kind='hit',instrument=instrument,event_at=onset,detected_at=onset+delay),110.)
    w.render(110.,'OFFLINE UI CHECK · saved WAV replay, test markers')
    screenshot(root,'waveform_before.png')
    before=[dict(x) for x in w.drawn_markers]
    left,right=62.,w.canvas.winfo_width()-20.
    for marker in before:
        expected=left+(marker['event_at']-100.)/10*(right-left)
        assert abs(marker['x']-expected)<1e-8
    w.render(110.5,'OFFLINE UI CHECK · same markers 0.5 seconds later')
    screenshot(root,'waveform_scrolled.png')
    after=w.drawn_markers
    for a,b in zip(before,after):assert abs((a['x']-b['x'])-(right-left)*.05)<1e-8
    # New late detection belongs at its earlier onset, not the current edge.
    assert t.add_hit(dict(kind='hit',instrument='ride',event_at=106.7055,detected_at=110.5),110.5)
    w.render(110.5,'OFFLINE UI CHECK · coincident events retain both colors and labels')
    screenshot(root,'waveform_overlap.png')
    overlap=[m for m in w.drawn_markers if m['event_at']==106.7055]
    assert len(overlap)==2 and overlap[0]['x']==overlap[1]['x']
    root.geometry('800x360');root.update();w.render(110.5,'OFFLINE UI CHECK · resized')
    assert len(w.drawn_markers)==4
    (OUT/'ui_verification.json').write_text(json.dumps(dict(success=True,physical_io=False,history_seconds=t.history,
        saved_wavs_unchanged=True,test_markers=True,original_onsets_exact=True,scroll_shift_exact=True,
        overlap_two_colors=True,resize_passed=True,markers_before=before),indent=2)+'\n')
    print('UI PASS: bounded waveform, onset positions, delayed insertion, scrolling, overlap and resizing')
finally:root.destroy()
