"""Read-only live quality preview, not annotation/export."""
from pathlib import Path
import json,sys
import numpy as np
from scipy.signal import stft
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]);raw=(p/'raw.wav').read_bytes();assert raw[:4]==b'RIFF' and raw[36:40]==b'data'
x=np.frombuffer(raw[44:len(raw)//2*2],'<i2').astype(float)/32768;sr=16000
cl=[]
for line in (p/'audio_blocks.jsonl').read_text().splitlines():
 try:cl.append(json.loads(line))
 except json.JSONDecodeError:pass
o=np.median([r['adc_monotonic']-r['frame']/sr for r in cl])
blocks=json.loads((p/'examples.json').read_text());wanted=[r for r in blocks if r['block'] in [9,10,17,18,25,29,33]]
fig,axs=plt.subplots(len(wanted),2,figsize=(16,3*len(wanted)),squeeze=False)
for (ax,bx),r in zip(axs,wanted):
 a=round((r['start']-o)*sr);b=round((r['end']-o)*sr);y=x[a:b]
 ax.plot(np.arange(len(y))/sr,y,lw=.5);ax.set_title(f"Live completed block {r['block']} {r['kind']}")
 f,t,z=stft(y,fs=sr,nperseg=256,noverlap=192);bx.pcolormesh(t,f,20*np.log10(np.maximum(abs(z),1e-6)),vmin=-85,vmax=-15,cmap='magma',shading='auto')
fig.tight_layout();fig.savefig(p/'live_audio_preview.png',dpi=100)
print('Read-only preview',len(wanted),'clips; raw peak',max(abs(x)))
