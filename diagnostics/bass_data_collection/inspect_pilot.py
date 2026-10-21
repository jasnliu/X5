"""Offline acoustic inspection only; no models or training."""
import json,sys,wave
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt,stft
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]);assert json.loads((p/'result.json').read_text())['relaxed_verified']
with wave.open(str(p/'raw.wav')) as w:
 sr=w.getframerate();x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768
clocks=[json.loads(s) for s in (p/'audio_blocks.jsonl').read_text().splitlines()]
offs=np.array([b['adc_monotonic']-b['frame']/sr for b in clocks]);o=float(np.median(offs))
(p/'clock_summary.json').write_text(json.dumps(dict(offset=o,spread_p99_s=float(np.percentile(offs,99)-np.percentile(offs,1))),indent=2))
events=[json.loads(s) for s in (p/'hihat_commands.jsonl').read_text().splitlines()]
kicks=[e for e in events if e['kind']=='beat' and e['command']=='K']
low=sosfilt(butter(3,[40,300],fs=sr,btype='bandpass',output='sos'),x)
fig,axes=plt.subplots(len(kicks),2,figsize=(16,3*len(kicks)),squeeze=False)
for (ax,bx),e in zip(axes,kicks):
 t=e['t']-o;a=round((t-.06)*sr);b=round((t+.5)*sr);tt=(np.arange(a,b)/sr-t)*1000
 ax.plot(tt,x[a:b],lw=.65);ax.plot(tt,low[a:b],lw=.8);ax.axvline(0,color='gray');ax.set_title(f"Block {e['block']} raw command {t:.5f}s; peak {max(abs(x[a:b])):.3f}")
 f,ts,z=stft(x[a:b],fs=sr,nperseg=256,noverlap=224)
 bx.pcolormesh((ts-.06)*1000,f,20*np.log10(np.maximum(abs(z),1e-6)),shading='auto',cmap='magma',vmin=-85,vmax=-20);bx.set_ylim(0,3000)
fig.tight_layout();fig.savefig(p/'pilot_attacks.png',dpi=120)
print('offset',o,'clock spread',np.ptp(offs),'kicks',[(e['block'],e['t']-o) for e in kicks])
