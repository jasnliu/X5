"""OFFLINE annotation aids. Raw acoustic templates/energy only; no trained models.
Motor times locate broad windows. All proposals remain unaccepted until review.
"""
import argparse,json,wave
from pathlib import Path
import numpy as np
from scipy.signal import correlate,butter,sosfilt,stft
from scipy.ndimage import uniform_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parent
SR=16000
COLORS=dict(bass_hit='red',hihat_close='limegreen',cymbal_hit='deepskyblue',snare_hit='darkorange')

def read_audio(p):
 with wave.open(str(p)) as w:
  assert (w.getframerate(),w.getnchannels(),w.getsampwidth())==(SR,1,2)
  return np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768

def clock(session):
 rows=[json.loads(s) for s in (session/'audio_blocks.jsonl').read_text().splitlines()]
 offsets=np.array([r['adc_monotonic']-r['frame']/SR for r in rows]);o=float(np.median(offsets))
 (session/'clock_summary.json').write_text(json.dumps(dict(offset=o,spread_p99_s=float(np.percentile(offsets,99)-np.percentile(offsets,1)),sample_rate=SR),indent=2)+'\n')
 return o

def bass_template():
 p=BASE/'20261010T232155-train-63812811';x=read_audio(p/'raw.wav')
 # Attack origin chosen by direct inspection of pilot_onset_zoom.png.
 origin=54.7075625;n=round(.022*SR);a=round(origin*SR)
 return x[a:a+n]-np.mean(x[a:a+n])

def matched(x,template,lo,hi):
 n=len(template); a=round(lo*SR);b=round(hi*SR)
 y=x[a:b+n];dot=correlate(y,template,mode='valid',method='direct')
 sums=np.convolve(y,np.ones(n),mode='valid');squares=np.convolve(y*y,np.ones(n),mode='valid')
 norm=np.sqrt(np.maximum(1e-15,squares-sums*sums/n)*np.dot(template,template))
 score=dot/norm;j=int(np.argmax(score));return (a+j)/SR,float(score[j])

def energy_onset(x,command,label):
 settings={'hihat_close':(1800,.035,.40),'cymbal_hit':(2300,.025,.32),'snare_hit':(700,.07,.23)}
 cutoff,earliest,latest=settings[label]
 a=max(0,round((command-.1)*SR));b=min(len(x),round((command+latest+.06)*SR))
 y=sosfilt(butter(3,cutoff,btype='highpass',fs=SR,output='sos'),x[a:b])
 env=np.sqrt(np.maximum(0,uniform_filter1d(y*y,24)))
 flux=uniform_filter1d(env,24)-np.roll(uniform_filter1d(env,24),160)
 lo=round((command+earliest)*SR)-a;hi=round((command+latest)*SR)-a
 peak=lo+int(np.argmax(flux[lo:hi]));left=max(lo,peak-200)
 baseline=float(np.median(env[max(0,peak-480):max(1,peak-200)]))
 high=float(np.percentile(env[peak:peak+160],75));threshold=baseline+.15*max(0.,high-baseline)
 candidates=np.flatnonzero(np.convolve((env[left:peak+1]>threshold).astype(int),np.ones(8,dtype=int),mode='valid')==8)
 onset=left+int(candidates[0]) if len(candidates) else peak
 return (a+onset)/SR,dict(energy_peak=high,baseline=baseline,flux=float(flux[peak]))

def propose(session):
 result=json.loads((session/'result.json').read_text());assert result['success'] and result['relaxed_verified']
 assert json.loads((session/'query_final_disabled.json').read_text())['all_16_disabled_fault_free']
 x=read_audio(session/'raw.wav');o=clock(session);template=bass_template()
 commands=[json.loads(s) for s in (session/'hihat_commands.jsonl').read_text().splitlines()]
 blocks=json.loads((session/'examples.json').read_text());review=[];details=[]
 for block in blocks:
  events=[]
  for e in commands:
   if e['kind']!='beat' or e['block']!=block['block'] or e['command'] not in ('K','B'):continue
   command=e['t']-o;label='bass_hit' if e['command']=='K' else 'hihat_close'
   if label=='bass_hit':
    onset,corr=matched(x,template,command+.04,command+.19);quality=dict(correlation=corr)
   else:onset,quality=energy_onset(x,command,label)
   event=dict(label=label,raw_seconds=onset,command_raw_seconds=command,latency_ms=(onset-command)*1000,**quality)
   events.append(dict(label=label,raw_seconds=onset));details.append(dict(block=block['block'],kind=block['kind'],**event))
  for e in block['arm_requests']:
   label={'ride':'cymbal_hit','snare':'snare_hit'}[e['instrument']];command=e['requested']-o
   onset,quality=energy_onset(x,command,label)
   events.append(dict(label=label,raw_seconds=onset));details.append(dict(block=block['block'],kind=block['kind'],label=label,raw_seconds=onset,command_raw_seconds=command,latency_ms=(onset-command)*1000,**quality))
  review.append(dict(block=block['block'],accepted=False,clip_start_seconds=round((block['start']-o)*SR)/SR,
   clip_end_seconds=round((block['end']-o)*SR)/SR,events=sorted(events,key=lambda e:e['raw_seconds']),notes='Unreviewed acoustic proposals; not motor timestamps.'))
 (session/'onset_proposals.json').write_text(json.dumps(details,indent=2)+'\n')
 (session/'review_proposed.json').write_text(json.dumps(dict(method='UNREVIEWED raw-template correlation and acoustic energy proposals',auditory_review_performed=False,examples=review),indent=2)+'\n')
 return review,details,x,blocks

def plots(session,review,details,x,blocks):
 lookup={b['block']:b for b in blocks}
 for page in range((len(review)+5)//6):
  subset=review[page*6:page*6+6];fig,axes=plt.subplots(len(subset),2,figsize=(17,3*len(subset)),squeeze=False)
  for (ax,bx),row in zip(axes,subset):
   b=lookup[row['block']];a,z=round(row['clip_start_seconds']*SR),round(row['clip_end_seconds']*SR);audio=x[a:z];t=np.arange(len(audio))/SR
   ax.plot(t,audio,lw=.4);ax.set_title(f"Block {b['block']} {b['kind']} {b['bpm']} BPM; peak {max(abs(audio)):.3f}")
   f,tt,s=stft(audio,fs=SR,nperseg=256,noverlap=192)
   bx.pcolormesh(tt,f,20*np.log10(np.maximum(abs(s),1e-6)),shading='auto',vmin=-85,vmax=-15,cmap='magma');bx.set_ylim(0,8000)
   for event in row['events']:
    hit=event['raw_seconds']-a/SR;color=COLORS[event['label']]
    ax.axvline(hit,color=color,alpha=.75,lw=.7);bx.axvline(hit,color=color,alpha=.8,lw=.7)
  fig.tight_layout();fig.savefig(session/f'review_page_{page+1}.png',dpi=110);plt.close(fig)
 # Expanded events where raw matching is weak or other instruments overlap.
 for label in COLORS:
  rows=[r for r in details if r['label']==label]
  for page in range((len(rows)+11)//12):
   fig,axes=plt.subplots(4,3,figsize=(16,11))
   for ax,r in zip(axes.flat,rows[page*12:page*12+12]):
    k=round(r['raw_seconds']*SR);a=max(0,k-480);b=min(len(x),k+960)
    ax.plot((np.arange(a,b)-k)/SR*1000,x[a:b],lw=.6);ax.axvline(0,color=COLORS[label])
    title=f"b{r['block']} {label} {r['latency_ms']:.1f} ms"
    if 'correlation' in r:title+=f" corr {r['correlation']:.3f}"
    ax.set_title(title,fontsize=9);ax.grid(alpha=.2)
   fig.tight_layout();fig.savefig(session/f'attack_{label}_{page+1}.png',dpi=110);plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--review',type=Path);a=p.parse_args()
 review,details,x,blocks=propose(a.session)
 if a.review:review=json.loads(a.review.read_text())['examples']
 plots(a.session,review,details,x,blocks)
 print(a.session.name,'clips',len(review),'labels',len(details))
 for label in COLORS:
  rows=[r for r in details if r['label']==label]
  if rows:print(label,'latency range',min(r['latency_ms'] for r in rows),max(r['latency_ms'] for r in rows),'min corr',min(r.get('correlation',1) for r in rows))

if __name__=='__main__':main()
