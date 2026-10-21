from pathlib import Path
import sys,json
import numpy as np
from scipy.signal import butter,sosfilt
from review_audio import read_audio,matched,SR,BASE
ref=BASE/'20261010T232851-train-4e7f0cb9';x=read_audio(ref/'raw.wav');sos=butter(3,1500,fs=SR,btype='highpass',output='sos')
y=sosfilt(sos,x);origin=167.424;n=round(.016*SR);template=y[round(origin*SR):round(origin*SR)+n];template=template-template.mean()
p=Path(sys.argv[1]);z=sosfilt(sos,read_audio(p/'raw.wav'));items=json.loads((p/'onset_proposals.json').read_text());rows=[]
for r in items:
 if r['label']!='cymbal_hit':continue
 c=r['command_raw_seconds'];t,score=matched(z,template,c+.095,c+.150)
 rows.append(dict(r,match_raw_seconds=t,match_corr=score,match_latency_ms=(t-c)*1000))
print([(r['block'],round(r['match_latency_ms'],2),round(r['match_corr'],3)) for r in rows])
(p/'ride_match.json').write_text(json.dumps(rows,indent=2)+'\n')
