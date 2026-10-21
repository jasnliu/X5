"""Compare acoustic proposals using filtered raw waveforms, never command labels."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt
from review_audio import read_audio,matched,SR,BASE
p=Path(sys.argv[1]);x=read_audio(p/'raw.wav')
source=BASE/'20261010T232155-train-63812811';pilot=read_audio(source/'raw.wav')
sos=butter(3,[40,300],fs=SR,btype='bandpass',output='sos')
y=sosfilt(sos,x);q=sosfilt(sos,pilot)
prefix=.015;origin=54.7075625;a=round((origin-prefix)*SR);template=q[a:a+round(.065*SR)];template-=template.mean()
a=json.loads((p/'onset_proposals.json').read_text());out=[]
for r in a:
 if r['label']!='bass_hit':continue
 command=r['command_raw_seconds'];t,c=matched(y,template,command+.04-prefix,command+.19-prefix);t+=prefix
 out.append(dict(r,filtered_raw_seconds=t,filtered_corr=c,delta_ms=(t-r['raw_seconds'])*1000))
(p/'bass_filtered_comparison.json').write_text(json.dumps(out,indent=2)+'\n')
print('filtered min',min(r['filtered_corr'] for r in out))
print('Shifts >3ms',[(r['block'],round(r['latency_ms'],1),round(r['delta_ms'],1),round(r['filtered_corr'],3)) for r in out if abs(r['delta_ms'])>3])
