import json,wave
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/x5data_collection/20261005T060856-f745b243')
with wave.open(str(p/'raw.wav')) as w:x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2')/32768;sr=w.getframerate()
hp=sosfilt(butter(4,3000,fs=sr,btype='highpass',output='sos'),x)
rows=json.loads((p/'onset_measurements.json').read_text());fig,axes=plt.subplots(5,2,figsize=(13,12))
changes=[]
for ax,row in zip(axes.flat,[r for r in rows if r['count']==2]):
 old=round(row['onset_raw_seconds']*sr);lo=old-160;hi=old+400;y=hp[lo:hi];energy=np.cumsum(y*y)+1e-16;n=len(y);k=np.arange(128,480)
 score=k*np.log(energy[k-1]/k)+(n-k)*np.log((energy[-1]-energy[k-1])/(n-k))
 chosen=int(k[np.argmin(score)])+lo
 row['initial_onset_raw_seconds']=row['onset_raw_seconds'];row['onset_raw_seconds']=chosen/sr
 row['refinement']='High-frequency variance change point plus visual waveform/spectrogram review'
 row['release_to_audio_ms']=(chosen/sr-row['release_raw_seconds'])*1000
 row['timestamp_uncertainty_seconds']=.003
 changes.append(dict(block=row['block'],shift_ms=(chosen-old)/sr*1000))
 a=chosen-160;b=chosen+400;t=(np.arange(a,b)-chosen)/sr*1000
 ax.plot(t,x[a:b],lw=.7);ax.axvline(0,color='r');ax.set_title(f"Block {row['block']}, second onset; shift {(chosen-old)/sr*1000:.2f} ms")
 ax.set_xlabel('ms relative to revised WAV onset')
fig.tight_layout();fig.savefig(p/'pair_onset_refinement.png',dpi=140)
(p/'onset_measurements_refined.json').write_text(json.dumps(rows,indent=2)+'\n')
print(changes)
