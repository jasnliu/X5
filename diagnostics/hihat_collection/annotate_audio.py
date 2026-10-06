"""Offline acoustic onset proposals, independent of any trained detector."""
import json,wave
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt,spectrogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/hihat_collection/20261005T175549-54ef005a')
assert json.loads((p/'query_final_disabled.json').read_text())['all_16_disabled_fault_free']
with wave.open(str(p/'raw.wav')) as w:
 sr=w.getframerate();x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768
clock=json.loads((p/'clock_summary.json').read_text())['offset']
commands=[json.loads(l) for l in (p/'hihat_commands.jsonl').read_text().splitlines()]
close=[v for v in commands if v['kind']=='beat' and v['command']=='C']
rows={r['block']:r for r in json.loads((p/'examples.json').read_text())}
hp=sosfilt(butter(3,2500,fs=sr,btype='highpass',output='sos'),x)
# 1 ms causal energy avoids acausal ringing before an onset.
env=np.sqrt(np.convolve(hp*hp,np.ones(16)/16,mode='full')[:len(x)])
proposals=[]
for c in close:
 t=c['t']-clock
 a,b=round((t+.05)*sr),round((t+.17)*sr)
 baseline=float(np.median(env[round((t+.015)*sr):round((t+.04)*sr)]))
 peak=float(np.max(env[a:b]));threshold=baseline+.15*(peak-baseline)
 above=(env[a:b]>threshold).astype(int)
 sustained=np.convolve(above,np.ones(48,dtype=int),mode='valid')>=40
 index=a+int(np.flatnonzero(sustained)[0])
 # Waveform threshold is the proposal; inspect every proposed leading edge.
 proposals.append(dict(block=c['block'],command_raw_seconds=t,onset_raw_seconds=index/sr,
                       latency_seconds=index/sr-t,baseline=baseline,peak=peak,threshold=threshold))
(p/'onset_proposals.json').write_text(json.dumps(proposals,indent=2)+'\n')
for batch in range(5):
 selected=proposals[batch*7:(batch+1)*7]
 fig,axes=plt.subplots(len(selected),2,figsize=(14,len(selected)*2.0),squeeze=False)
 for idx,v in enumerate(selected):
  t=v['command_raw_seconds'];onset=v['onset_raw_seconds'];a,b=round((t+.035)*sr),round((t+.18)*sr)
  tt=np.arange(a,b)/sr-t
  ax=axes[idx,0];ax.plot(tt*1000,x[a:b],lw=.7)
  ax.axvline((onset-t)*1000,color='red',lw=1)
  ax.set_title(f"B{v['block']} {rows[v['block']]['kind']}: proposed {onset:.6f} s; delay {(onset-t)*1000:.2f} ms")
  ax.set_xlim(35,180);ax.grid(alpha=.2)
  ax=axes[idx,1];ax.plot(tt*1000,env[a:b],lw=.7);ax.axvline((onset-t)*1000,color='red');ax.axhline(v['threshold'],color='gray',ls=':')
  ax.set_xlim(35,180);ax.grid(alpha=.2)
 fig.tight_layout();fig.savefig(p/f'onset_details_{batch+1}.png',dpi=120);plt.close(fig)
print(json.dumps([dict(block=v['block'],time=v['onset_raw_seconds'],delay_ms=round(v['latency_seconds']*1000,2)) for v in proposals],indent=2))
# Backgrounds include absence of a closure command and spectral/energy review.
fig,axes=plt.subplots(10,2,figsize=(14,18))
for i,block in enumerate(range(31,41)):
 row=rows[block];a,b=round((row['start']-clock)*sr),round((row['end']-clock)*sr);y=x[a:b]
 ax=axes[i,0];ax.plot(np.arange(len(y))/sr,y,lw=.3);ax.set_ylim(-.06,.06);ax.set_title(f'B{block}: background peak {np.max(np.abs(y)):.4f}')
 f,t,s=spectrogram(y,sr,nperseg=256,noverlap=192)
 axes[i,1].pcolormesh(t,f/1000,10*np.log10(s+1e-14),vmin=-110,vmax=-45,cmap='magma',shading='auto')
fig.tight_layout();fig.savefig(p/'background_review.png',dpi=110);plt.close(fig)
