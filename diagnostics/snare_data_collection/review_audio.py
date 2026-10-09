"""OFFLINE signal-review aids; refuses execution until successful centered shutdown.
No learned detector; motor times locate broad windows, never become CSV labels.
"""
import argparse,json,wave
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt,stft
from scipy.ndimage import uniform_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('session',type=Path);args=p.parse_args();d=args.session
assert json.loads((d/'result.json').read_text())['success']
assert json.loads((d/'query_final_disabled.json').read_text())['all_16_disabled_fault_free']
with wave.open(str(d/'raw.wav')) as w:
    sr=w.getframerate();x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768
assert sr==16000 and np.max(np.abs(x))<32767/32768
clocks=[json.loads(line) for line in (d/'audio_blocks.jsonl').read_text().splitlines()]
offsets=np.array([r['adc_monotonic']-r['frame']/sr for r in clocks])
o=float(np.median(offsets))
(d/'clock_summary.json').write_text(json.dumps(dict(offset=o,spread_p99_s=float(np.percentile(offsets,99)-np.percentile(offsets,1)),sample_rate=sr),indent=2)+'\n')
examples=json.loads((d/'examples.json').read_text())
hp=sosfilt(butter(3,300,fs=sr,btype='highpass',output='sos'),x)
env=np.sqrt(np.maximum(0,uniform_filter1d(hp*hp,16)))
flux=uniform_filter1d(env,32)-np.roll(uniform_filter1d(env,32),128)
rows=[]; details=[]
for r in examples:
    onsets=[]
    for e in r['snare_events']:
        start=e['started']-o
        lo=round((start+.035)*sr);hi=round((start+.24)*sr)
        rise=lo+int(np.argmax(flux[lo:hi]))
        baseline=float(np.median(env[max(lo,rise-960):max(lo+1,rise-640)]))
        high=float(np.percentile(env[rise:rise+320],75))
        threshold=baseline+.10*(high-baseline)
        left=max(lo,rise-640)
        # First sustained rise, not a later large waveform cycle/valley.
        above=env[left:rise+1]>=threshold
        runs=np.convolve(above.astype(int),np.ones(8,dtype=int),mode='valid')
        candidates=np.flatnonzero(runs==8)
        if not len(candidates):raise RuntimeError('No sustained acoustic rise')
        onset=left+int(candidates[0])
        correction_path=d/'mixture_correction.json'
        if correction_path.exists():
            correction=json.loads(correction_path.read_text())
            if r['block']==correction['block']:
                onset=round(correction['new_seconds']*sr)
        onsets.append(onset/sr)
        details.append(dict(block=r['block'],kind=r['kind'],onset_raw_seconds=onset/sr,
            command_raw_seconds=start,latency_ms=(onset/sr-start)*1000,peak=high,baseline=baseline))
    rows.append(dict(block=r['block'],accepted=False,clip_start_seconds=round((r['start']-o)*sr)/sr,
                     clip_end_seconds=round((r['end']-o)*sr)/sr,onsets_raw_seconds=onsets,
                     notes='Unreviewed signal proposals. Do not export until visual review.'))
for page in range((len(rows)+5)//6):
    fig,axes=plt.subplots(6,2,figsize=(16,16))
    for axesrow,r,example in zip(axes,rows[page*6:page*6+6],examples[page*6:page*6+6]):
        a,b=round(r['clip_start_seconds']*sr),round(r['clip_end_seconds']*sr)
        audio=x[a:b];t=np.arange(len(audio))/sr
        axesrow[0].plot(t,audio,lw=.4)
        for hit in r['onsets_raw_seconds']:axesrow[0].axvline(hit-a/sr,color='red')
        axesrow[0].set_title(f"Block {r['block']} {example['kind']} / {example['snare_hits']} snare")
        f,tt,z=stft(audio,fs=sr,nperseg=256,noverlap=192)
        axesrow[1].pcolormesh(tt,f,20*np.log10(np.maximum(abs(z),1e-6)),shading='auto',vmin=-75,vmax=-15,cmap='magma')
        for hit in r['onsets_raw_seconds']:axesrow[1].axvline(hit-a/sr,color='cyan',lw=1)
        axesrow[1].set_ylim(0,8000)
    fig.tight_layout();fig.savefig(d/f'review_page_{page+1}.png',dpi=105);plt.close(fig)
for page in range((len(details)+8)//9):
    fig,axes=plt.subplots(3,3,figsize=(16,10))
    for ax,r in zip(axes.flat,details[page*9:page*9+9]):
        k=round(r['onset_raw_seconds']*sr);a=k-640;b=k+1280
        t=(np.arange(a,b)-k)/sr*1000
        ax.plot(t,x[a:b],lw=.6);ax.plot(t,env[a:b],lw=.9,color='orange');ax.axvline(0,color='red')
        ax.set_title(f"Block {r['block']} {r['kind']}\ncommand→proposal {r['latency_ms']:.1f} ms")
        ax.set_xlabel('ms from proposed acoustic attack')
    fig.tight_layout();fig.savefig(d/f'onset_details_{page+1}.png',dpi=120);plt.close(fig)
(d/'onset_proposals.json').write_text(json.dumps(details,indent=2)+'\n')
(d/'review_proposed.json').write_text(json.dumps(dict(method='Unreviewed signal proposals',examples=rows),indent=2)+'\n')
print(d,'clips',len(rows),'onsets',len(details),'PCM peak',max(abs(x)),'clock p99 spread',np.percentile(offsets,99)-np.percentile(offsets,1))
print('Proposal latencies ms',[(r['block'],round(r['latency_ms'],1)) for r in details])
