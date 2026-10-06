import json,wave,csv
from pathlib import Path
import numpy as np
from scipy.signal import spectrogram,butter,sosfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/x5data_collection/20261005T060856-f745b243')
with wave.open(str(p/'raw.wav')) as w:x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768;sr=w.getframerate()
o=json.loads((p/'clock_summary.json').read_text())['offset']
r=[json.loads(z) for z in (p/'strikes.jsonl').read_text().splitlines() if json.loads(z)['kind']=='release']
hp=sosfilt(butter(4,1500,fs=sr,btype='highpass',output='sos'),x)
rows=[]
for number in range(1,31):
 rr=[v for v in r if v['block']==number];a=rr[0]['released_at']-o
 lo=max(0,int((a-.35)*sr));hi=int((rr[-1]['released_at']-o+1.0)*sr)
 y=x[lo:hi];z=hp[lo:hi];tt=np.arange(len(y))/sr+lo/sr-a
 rows.append((number,rr,tt,y,z))
for batch in range(6):
 fig,axes=plt.subplots(5,2,figsize=(13,13))
 for idx,(num,rr,t,y,z) in enumerate(rows[batch*5:batch*5+5]):
  ax=axes[idx,0];ax.plot(t,y,lw=.5);ax.set_title(f'Block {num}: {rr[0]["depth_deg"]} deg, {len(rr)} attempts');ax.set_ylim(-.17,.17)
  for e in rr:ax.axvline(e['released_at']-o-(rr[0]['released_at']-o),color='r',lw=.7)
  f,tt,s=spectrogram(y,sr,nperseg=256,noverlap=224);axes[idx,1].pcolormesh(tt+t[0],f/1000,10*np.log10(s+1e-14),vmin=-100,vmax=-45,shading='auto',cmap='magma');axes[idx,1].set_ylim(0,8)
 fig.tight_layout();fig.savefig(p/f'review_page_{batch+1}.png',dpi=125);plt.close(fig)
# Compare directly to two existing labeled reference recordings; no fitting/model.
fig,axes=plt.subplots(2,2,figsize=(12,6))
for i,name in enumerate(('r1','r10')):
 with wave.open(f'../st7/data/recordings/{name}.wav') as w: yy=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2')/32768
 hit=float(next(csv.DictReader(open(f'../st7/data/timestamps/t{name[1:]}.csv')))['hit_time_seconds']);lo=int((hit-.35)*sr);hi=int((hit+1)*sr);yy=yy[lo:hi];tt=np.arange(len(yy))/sr-.35
 axes[i,0].plot(tt,yy,lw=.5);axes[i,0].axvline(0,color='g');axes[i,0].set_title(name+' annotated hit');f,t,s=spectrogram(yy,sr,nperseg=256,noverlap=224);axes[i,1].pcolormesh(t-.35,f/1000,10*np.log10(s+1e-14),vmin=-100,vmax=-45,shading='auto',cmap='magma')
fig.tight_layout();fig.savefig(p/'reference_comparison.png',dpi=130)
print(p)
