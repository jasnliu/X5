import json,wave
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt
from scipy.ndimage import uniform_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('diagnostics/x5data_collection/20261005T060856-f745b243')
assert json.loads((p/'result.json').read_text())['relaxed_verified']
assert json.loads((p/'query_final_disabled.json').read_text())['all_16_disabled_fault_free']
with wave.open(str(p/'raw.wav')) as w:x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768;sr=w.getframerate()
o=json.loads((p/'clock_summary.json').read_text())['offset']
hp=sosfilt(butter(4,1500,fs=sr,btype='highpass',output='sos'),x)
env=np.sqrt(uniform_filter1d(hp*hp,size=16))
# Non-learned onset aid only. All 30 examples are separately inspected as
# waveform/spectrogram pages. No ST7 output or encoder bottom is a label.
flux=uniform_filter1d(env,size=48)-np.roll(uniform_filter1d(env,size=48),160)
events=[json.loads(t) for t in (p/'strikes.jsonl').read_text().splitlines() if json.loads(t)['kind']=='release']
rows=[]
for e in events:
 release=e['released_at']-o
 lo=int((release+.04)*sr);hi=int((release+.22)*sr)
 rise=lo+int(np.argmax(flux[lo:hi]))
 baseline=float(np.median(env[rise-480:rise-240]))
 high=float(np.percentile(env[rise:rise+480],75))
 threshold=baseline+.12*(high-baseline)
 left=max(lo,rise-240)
 crossing=np.flatnonzero(env[left:rise+1]>=threshold)
 assert len(crossing)
 onset=left+int(crossing[0])
 pre=np.sqrt(np.mean(hp[onset-1600:onset-320]**2))
 post=np.sqrt(np.mean(hp[onset+160:onset+2400]**2))
 rows.append(dict(block=e['block'],count=e['count'],onset_raw_seconds=onset/sr,
                  release_raw_seconds=release,release_to_audio_ms=(onset/sr-release)*1000,
                  highband_snr_db=float(20*np.log10(post/(pre+1e-10))),
                  timestamp_uncertainty_seconds=.003))
for page in range(4):
 fig,axes=plt.subplots(5,2,figsize=(13,12))
 for ax,row in zip(axes.flat,rows[page*10:page*10+10]):
  k=round(row['onset_raw_seconds']*sr);lo=k-160;hi=k+400;tt=(np.arange(lo,hi)-k)/sr*1000
  ax.plot(tt,x[lo:hi],lw=.7);ax.axvline(0,color='r');ax.set_title(f"Block {row['block']} impact {row['count']}; SNR {row['highband_snr_db']:.1f} dB")
  ax.set_xlabel('ms relative to proposed WAV onset')
 fig.tight_layout();fig.savefig(p/f'onset_detail_{page+1}.png',dpi=120);plt.close(fig)
(p/'onset_measurements.json').write_text(json.dumps(rows,indent=2)+'\n')
print('release-to-audio ms range',min(r['release_to_audio_ms'] for r in rows),max(r['release_to_audio_ms'] for r in rows))
print('HF SNR range',min(r['highband_snr_db'] for r in rows),max(r['highband_snr_db'] for r in rows))
print('double spacings ms',[(rows[i+1]['onset_raw_seconds']-r['onset_raw_seconds'])*1000 for i,r in enumerate(rows[:-1]) if r['block']>20 and r['count']==1])
