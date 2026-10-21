"""Render target-only review without accepting or exporting any proposal."""
import sys,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from review_audio import read_audio,plots,SR
p=Path(sys.argv[1]);r=json.loads((p/'review_proposed.json').read_text());d=json.loads((p/'onset_proposals.json').read_text())
d=[x for x in d if x['label']=='bass_hit']
for row in r['examples']:row['events']=[e for e in row['events'] if e['label']=='bass_hit']
r['annotation_scope']='bass_only';r['method']='UNREVIEWED bass-only acoustic proposals'
(p/'review_bass_proposed.json').write_text(json.dumps(r,indent=2)+'\n')
out=p/'bass_review';out.mkdir(exist_ok=True)
x=read_audio(p/'raw.wav');blocks=json.loads((p/'examples.json').read_text())
plots(out,r['examples'],[],x,blocks)
weak=[e for e in d if e['correlation']<.85 or not 80<=e['latency_ms']<=100]
for page in range(math.ceil(len(weak)/12)):
 subset=weak[page*12:page*12+12];fig,axs=plt.subplots(math.ceil(len(subset)/3),3,figsize=(17,3*math.ceil(len(subset)/3)),squeeze=False)
 for ax,e in zip(axs.flat,subset):
  k=round(e['raw_seconds']*SR);a=k-400;b=k+960;t=(np.arange(a,b)-k)/SR*1000
  ax.plot(t,x[a:b],lw=.6);ax.axvline(0,color='red');ax.set_title(f"b{e['block']} {e['latency_ms']:.1f} ms corr {e['correlation']:.3f}",fontsize=10);ax.grid(alpha=.3)
 fig.tight_layout();fig.savefig(out/f'weak_attacks_{page+1}.png',dpi=105);plt.close(fig)
print(p.name,'bass',len(d),'weak review',len(weak))
