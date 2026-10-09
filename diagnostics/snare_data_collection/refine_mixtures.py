"""Offline correction of two reviewed interference-triggered onset proposals."""
import json,wave
from pathlib import Path
import numpy as np
from scipy.signal import correlate
ROOT=Path(__file__).resolve().parent
train=ROOT/'20261008T005622-development-4ebb64e9'
def load(d):
 with wave.open(str(d/'raw.wav')) as w:return np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)
x=load(train)
rows=json.loads((train/'onset_proposals.json').read_text())
t=next(r['onset_raw_seconds'] for r in rows if r['block']==4)
k=round(t*16000);template=x[k:k+224];template-=template.mean()
for name,block in [('20261008T005622-development-4ebb64e9',26),('20261008T010321-test-40bddeb3',50)]:
 d=ROOT/name;y=load(d);review=json.loads((d/'review_proposed.json').read_text());r=next(r for r in review['examples'] if r['block']==block)
 old=r['onsets_raw_seconds'][0];a=round((old-.01)*16000);b=round((old+.05)*16000)
 segment=y[a:b]
 dot=correlate(segment,template,mode='valid')
 energy=np.convolve(segment**2,np.ones(len(template)),mode='valid')-np.convolve(segment,np.ones(len(template)),mode='valid')**2/len(template)
 similarity=dot/np.sqrt(np.maximum(energy,1)*np.sum(template**2))
 best=int(np.argmax(similarity));new=(a+best)/16000
 print('block',block,'old',old,'new',new,'adjustment_ms',(new-old)*1000,'correlation',similarity[best])
 record=dict(block=block,old_seconds=old,new_seconds=new,adjustment_ms=(new-old)*1000,template_block=4,similarity=float(similarity[best]))
 (d/'mixture_correction.json').write_text(json.dumps(record,indent=2)+'\n')
