import json
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from review_audio import read_audio,BASE,SR
p=BASE/'20261010T232851-train-4e7f0cb9';x=read_audio(p/'raw.wav');items=json.loads((p/'onset_proposals.json').read_text())
rows=[]
for label,block in [('hihat_close',10),('cymbal_hit',18),('snare_hit',25)]:
 rows.extend([r for r in items if r['label']==label and r['block']==block])
fig,axs=plt.subplots(6,2,figsize=(17,17))
for (ax,bx),r in zip(axs,rows):
 c=r['command_raw_seconds'];a=round((c+.045)*SR);b=round((c+.19)*SR)
 t=(np.arange(a,b)/SR-c)*1000;z=x[a:b];hp=sosfilt(butter(3,1200,fs=SR,btype='highpass',output='sos'),z)
 ax.plot(t,z,lw=.5);bx.plot(t,hp,lw=.6)
 for q in (ax,bx):q.axvline(r['latency_ms'],color='red');q.set_title(f"b{r['block']} {r['label']} command {c:.5f} proposal {r['latency_ms']:.2f}ms");q.set_xticks(np.arange(50,191,10));q.grid(alpha=.3)
fig.tight_layout();fig.savefig(p/'context_attack_review.png',dpi=100)
