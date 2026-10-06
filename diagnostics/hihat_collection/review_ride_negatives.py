import json,wave
from pathlib import Path
import numpy as np
from scipy.signal import spectrogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/hihat_collection/20261005T175549-54ef005a')
ids=json.loads((p/'plan.json').read_text())['plan']['ride_negative_ids']
for page in range(2):
 fig,axes=plt.subplots(10,2,figsize=(14,18))
 for i,id in enumerate(ids[page*10:page*10+10]):
  with wave.open(f'X5data/ride/recordings/{id}.wav') as w:x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768;sr=w.getframerate()
  axes[i,0].plot(np.arange(len(x))/sr,x,lw=.3);axes[i,0].set_title(f'{id}: existing ride-only recording');axes[i,0].set_ylim(-.6,.6)
  f,t,s=spectrogram(x,sr,nperseg=256,noverlap=192)
  axes[i,1].pcolormesh(t,f/1000,10*np.log10(s+1e-14),vmin=-110,vmax=-45,cmap='magma',shading='auto')
 fig.tight_layout();fig.savefig(p/f'ride_negative_review_{page+1}.png',dpi=110);plt.close(fig)
