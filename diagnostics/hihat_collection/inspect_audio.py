import json,wave
from pathlib import Path
import numpy as np
from scipy.signal import spectrogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/hihat_collection/20261005T175549-54ef005a')
assert json.loads((p/'query_final_disabled.json').read_text())['all_16_disabled_fault_free']
with wave.open(str(p/'raw.wav')) as w:x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768;sr=w.getframerate()
clocks=[json.loads(l) for l in (p/'audio_blocks.jsonl').read_text().splitlines()]
offsets=np.array([v['adc_monotonic']-v['frame']/sr for v in clocks]);o=float(np.median(offsets))
(p/'clock_summary.json').write_text(json.dumps(dict(offset=o,spread_ms=float(np.ptp(offsets)*1000)),indent=2)+'\n')
commands=[json.loads(l) for l in (p/'hihat_commands.jsonl').read_text().splitlines()]
close=[v for v in commands if v['kind']=='beat' and v['command']=='C']
rows=sorted(json.loads((p/'examples.json').read_text()),key=lambda r:r['block'])
for batch in range(6):
 fig,axes=plt.subplots(5,2,figsize=(14,13))
 for idx,row in enumerate(rows[batch*5:batch*5+5]):
  t0=row['close_due']-o;end=t0+(row['closures']-1)*1.2+1.1
  a=int((t0-.55)*sr);b=int(end*sr);y=x[a:b];tt=np.arange(len(y))/sr+a/sr-t0
  ax=axes[idx,0];ax.plot(tt,y,lw=.5);ax.set_title(f"Block {row['block']} {row['kind']}, {row['closures']} closures; ride offset {row.get('ride_command_offset_s')}")
  ax.set_ylim(-.65,.65)
  for v in close:
   if v['block']==row['block']:ax.axvline(v['t']-o-t0,color='green')
  ax.axvline(.6,color='gray',ls=':')
  if 'last_release' in row:ax.axvline(row['last_release']-o-t0,color='orange')
  f,t,s=spectrogram(y,sr,nperseg=256,noverlap=224)
  axes[idx,1].pcolormesh(t+tt[0],f/1000,10*np.log10(s+1e-14),vmin=-105,vmax=-40,shading='auto',cmap='magma')
  axes[idx,1].set_ylim(0,8)
 fig.tight_layout();fig.savefig(p/f'review_page_{batch+1}.png',dpi=115);plt.close(fig)
print('clock',o,'spread ms',np.ptp(offsets)*1000,'clipped samples',int(np.sum(np.abs(x)>=32767/32768)))
print('closure commands',len(close),'examples',len(rows),'peak',np.max(np.abs(x)))
