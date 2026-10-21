import json,wave
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/bass_data_collection/20261010T232155-train-63812811')
o=json.loads((p/'clock_summary.json').read_text())['offset'];sr=16000
with wave.open(str(p/'raw.wav')) as w:x=np.frombuffer(w.readframes(w.getnframes()),'<i2').astype(float)/32768
events=[json.loads(s) for s in (p/'hihat_commands.jsonl').read_text().splitlines()]
k=[e for e in events if e['kind']=='beat' and e['command']=='K']
fig,axs=plt.subplots(5,1,figsize=(14,12))
for ax,e in zip(axs,k):
 t=e['t']-o;a=round((t+.065)*sr);b=round((t+.11)*sr)
 ax.plot((np.arange(a,b)/sr-t)*1000,x[a:b],lw=.7);ax.set_title(f"Block {e['block']}, K {t:.6f}");ax.set_xticks(np.arange(65,111,2));ax.grid()
fig.tight_layout();fig.savefig(p/'pilot_onset_zoom.png',dpi=110)
