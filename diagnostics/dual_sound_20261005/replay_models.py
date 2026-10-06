"""Offline real-model inference only, on existing WAVs; never playback sound."""
import json,sys,time
from pathlib import Path
import torch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT.parent/'st7'))
from cymbal_detector.config import load_config
from cymbal_detector.detector import CymbalDetector
from cymbal_detector.data import load_audio
from cymbal_detector.normality import load_configured_normality,add_normality_scores

torch.set_num_threads(1)
models={}
for name,version in [('ride','v2'),('hihat','hihat_v1')]:
 config=load_config(ROOT.parent/'st7/models'/version/'config.yaml')
 models[name]=(CymbalDetector(config),load_configured_normality(config,None))
report={}
for number in (1,21,41):
 path=ROOT/f'X5data/hihat/recording/r{number}.wav'
 audio=load_audio(path);item={}
 for name,(detector,normality) in models.items():
  started=time.perf_counter()
  events=add_normality_scores(audio,detector.detect_waveform(audio),normality)
  item[name]=dict(events=events,elapsed_s=time.perf_counter()-started)
 report[path.name]=item
assert len(report['r1.wav']['hihat']['events'])==2
assert len(report['r21.wav']['hihat']['events'])==1
assert not report['r41.wav']['hihat']['events']
assert report['r41.wav']['ride']['events']
(Path(__file__).parent/'replay_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
