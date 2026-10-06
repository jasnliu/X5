"""Offline evidence only; no trained models or hardware commands."""
from pathlib import Path
import json,wave
import numpy as np
from scipy.signal import butter,sosfilt,spectrogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
paths=json.loads((OUT/'run1_paths.json').read_text());run=Path(paths['run'])
assert json.loads((OUT/'query_after_run1.json').read_text())['all_16_disabled_fault_free']
with wave.open(str(run/'raw.wav')) as w:
 sr=w.getframerate();x=np.frombuffer(w.readframes(w.getnframes()),'<i2').astype(float)/32768
clock=[json.loads(l) for l in (run/'audio_blocks.jsonl').read_text().splitlines()]
offsets=np.array([r['adc_monotonic']-r['frame']/sr for r in clock]);offset=float(np.median(offsets))
rows=[json.loads(l) for l in Path(paths['timeline']).read_text().splitlines()]
cmd={r['index']:r for r in rows if r['kind']=='command'}
matches=[r for r in rows if r['kind']=='match'];hits=[r for r in rows if r['kind']=='onset']
power=sosfilt(butter(3,1500,fs=sr,btype='highpass',output='sos'),x)**2
power=np.convolve(power,np.ones(16)/16,mode='full')[:len(x)]
for batch,indices in enumerate(([1,3,7,19],[51,59,63,65])):
 fig,axes=plt.subplots(4,2,figsize=(15,11))
 for i,index in enumerate(indices):
  c=cmd[index];target=c['target'];lo=round((target-offset-.22)*sr);hi=round((target-offset+.32)*sr)
  y=x[lo:hi];t=(np.arange(lo,hi)/sr+offset-target)*1000
  axes[i,0].plot(t,y,lw=.4)
  axes[i,0].set_title(f'Beat edge {index}: advance {c["advance"]*1000:.0f} ms; waveform')
  f,tt,s=spectrogram(y,sr,nperseg=128,noverlap=112)
  axes[i,1].pcolormesh(tt*1000+t[0],f/1000,10*np.log10(s+1e-14),vmin=-100,vmax=-40,cmap='magma',shading='auto')
  for ax in axes[i]:
   ax.axvline(0,color='gray',ls=':',label='ride grid')
   ax.axvline((c['at']-target)*1000,color='purple',ls='--',label='close command')
   for k,color in [('ride','orange'),('hihat','cyan')]:
    near=[r for r in hits if r['instrument']==k and -.22<r['event_at']-target<.32]
    for n,r in enumerate(near):ax.axvline((r['event_at']-target)*1000,color=color,lw=1.4,label=k+' detector' if n==0 else None)
   ax.set_xlim(-220,320);ax.set_xlabel('ms relative to unmodified ride grid')
  axes[i,0].legend(fontsize=7,loc='upper left');axes[i,1].set_ylim(0,8)
 fig.tight_layout();fig.savefig(OUT/f'waveform_review_{batch+1}.png',dpi=120);plt.close(fig)
commands=list(cmd.values());closed=[r for r in commands if r['closed']]
summary=dict(paths=paths,audio_clock_offset=offset,audio_clock_spread_ms=float(np.ptp(offsets)*1000),
 clipped_samples=int(np.sum(np.abs(x)>=32767/32768)),commands=len(commands),closures=len(closed),
 matches=len(matches),skips=sum(r['kind']=='skip' for r in rows),
 initial_five_detector_error_median_ms=float(np.median([r['error'] for r in matches[:5]])*1000),
 final_five_detector_error_median_ms=float(np.median([r['error'] for r in matches[-5:]])*1000),
 max_command_lateness_ms=max(r['lateness'] for r in commands)*1000,
 final_estimated_advance_ms=[r for r in rows if r['kind']=='stop'][-1]['advance']*1000,
 final_applied_advance_ms=closed[-1]['advance']*1000,
 exact_grid_error_ms=max(abs(r['target']-(commands[0]['target']+r['index']*.6)) for r in commands)*1000,
 paired_advances_identical=all(c['advance']==o['advance'] for c,o in zip(commands[1::2],commands[2::2])))
(OUT/'run1_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
# Independent, offline leading-edge measurements of the dominant closure burst.
# This is supporting waveform evidence, NOT training labels or runtime input.
measurements=[]
env=np.sqrt(power)
for c in closed:
 at=c['at']-offset;a,b=round((at+.035)*sr),round((at+.18)*sr)
 base=float(np.median(env[round((at+.015)*sr):round((at+.035)*sr)]))
 peak=float(np.max(env[a:b]));threshold=base+.15*(peak-base)
 sustain=np.convolve((env[a:b]>threshold).astype(int),np.ones(48,dtype=int),mode='valid')>=40
 if not np.any(sustain):continue
 onset=(a+int(np.flatnonzero(sustain)[0]))/sr+offset
 measurements.append(dict(index=c['index'],onset=onset,latency_ms=(onset-c['at'])*1000,
     relative_to_ride_grid_ms=(onset-c['target'])*1000,advance_ms=c['advance']*1000))
early=[r for r in measurements if r['advance_ms']==0.]
late=[r for r in measurements if r['advance_ms']>=99.]
summary['independent_waveform_review']=dict(
 method='1 ms causal RMS above 1.5 kHz; 15 percent burst rise, sustained 2.5 ms of 3 ms, searched 35-180 ms after close; visually reviewed representative plots.',
 caveat='Dominant closure-burst measurements, not independent classification of the weak overlapping ride; no exact acoustic lock claim.',
 baseline_closures=len(early),late_closures=len(late),
 baseline_grid_error_median_ms=float(np.median([r['relative_to_ride_grid_ms'] for r in early])),
 late_grid_error_median_ms=float(np.median([r['relative_to_ride_grid_ms'] for r in late])),
 closure_latency_median_ms=float(np.median([r['latency_ms'] for r in measurements])),measurements=measurements)
(OUT/'run1_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary['independent_waveform_review'].items() if k!='measurements'},indent=2))
