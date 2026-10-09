"""Offline reviewed snare-onset export. Commands locate windows, never labels."""
import argparse
import csv
import fcntl
import hashlib
import io
import json
import math
from pathlib import Path
import re
import wave

import numpy as np
from x5_collection.plan import CSV_FIELDS

ROOT=Path(__file__).resolve().parents[1]


def wav_read(path):
    with wave.open(str(path)) as source:
        if (source.getframerate(),source.getnchannels(),source.getsampwidth())!=(16000,1,2):
            raise ValueError('Expected 16 kHz mono PCM16 WAV')
        return np.frombuffer(source.readframes(source.getnframes()),dtype='<i2')


def export(session, review, output=ROOT/'X5data/snare', ride=ROOT/'X5data/ride'):
    session,output=map(Path,(session,output))
    result=json.loads((session/'result.json').read_text())
    disabled=json.loads((session/'query_final_disabled.json').read_text())
    accessory=json.loads((session/'hihat_result.json').read_text())
    audio=json.loads((session/'audio_result.json').read_text())
    centers=json.loads((session/'both_center_before_relax.json').read_text())
    if not(result['success'] and result['relaxed_verified'] and disabled['all_16_disabled_fault_free']):
        raise ValueError('Export requires successful centered relaxation')
    for side in ('left','right'):
        p=centers[side]
        if not all(math.isfinite(p[k]) for k in ('max_error_deg','span_deg','settled_s')) or p['max_error_deg']>.2 or p['span_deg']>.12 or p['settled_s']<.6:
            raise ValueError('Both arms require strict settled center proof')
    if not accessory['release_ack'] or accessory.get('error') or accessory.get('shutdown_error') or audio['errors']:
        raise ValueError('Export requires clean capture and accessory release')
    if not review.get('method'):raise ValueError('Explicit audio review provenance required')
    examples={r['block']:r for r in json.loads((session/'examples.json').read_text())}
    raw=wav_read(session/'raw.wav')
    if len(review['examples'])!=len(examples) or {r['block'] for r in review['examples']}!=set(examples):
        raise ValueError('Review must cover every captured block exactly once')
    output.mkdir(parents=True,exist_ok=True)
    with (output/'.export.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        manifest_path=output/'manifest.json'
        manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else dict(
            version=1,target='snare_hit',grouping='physical-session',training_performed=False,records=[])
        if manifest['target']!='snare_hit':raise ValueError('Wrong dataset target')
        old=manifest['records']
        if any(r.get('collection_session',r['group'])==session.name for r in old):raise ValueError('Session already exported; refusing duplicates')
        numbers=[int(re.fullmatch(r'r(\d+)',r['id']).group(1)) for r in old]
        number=max(numbers,default=0)
        planned=[]
        for item in review['examples']:
            if item.get('accepted') is not True:raise ValueError('Uncertain clips require review/recollection, not fabricated labels')
            block=examples[item['block']]
            onsets=[float(t) for t in item.get('onsets_raw_seconds',[])]
            if len(onsets)!=block['snare_hits']:
                raise ValueError('Reviewed snare onset count does not match block')
            if any(not np.isfinite(t) for t in onsets) or any(b-a<.08 for a,b in zip(onsets,onsets[1:])):
                raise ValueError('Nonfinite, unordered or duplicate onsets')
            start,end=float(item['clip_start_seconds']),float(item['clip_end_seconds'])
            if not np.isfinite([start,end]).all() or not 0<=start<end<=len(raw)/16000:
                raise ValueError('Clip outside raw audio')
            a,b=round(start*16000),round(end*16000)
            if any(not a/16000<=t<b/16000 for t in onsets):raise ValueError('Onset outside clip')
            clip=raw[a:b]
            source_ride=None
            if 'source_ride_id' in item:
                if block['kind']!='ride_only' or block['snare_hits'] or onsets or block['split']=='test':
                    raise ValueError('Ride replacement is only permitted for development ride-only negatives')
                sources=json.loads((Path(ride)/'manifest.json').read_text())['records']
                source_ride=next(r for r in sources if r['id']==item['source_ride_id'])
                source_path=Path(ride)/source_ride['wav']
                if hashlib.sha256(source_path.read_bytes()).hexdigest()!=source_ride['wav_sha256']:
                    raise ValueError('Ride replacement source hash mismatch')
                clip=wav_read(source_path)
            if not len(clip) or np.any(np.abs(clip.astype(np.int32))>=32767):
                raise ValueError('Empty or clipped audio requires review')
            number+=1
            wav=output/'recordings'/f'r{number}.wav'; labels=output/'timestamps'/f't{number}.csv'
            if wav.exists() or labels.exists():raise ValueError('Refusing to overwrite existing dataset files')
            hits=[round(t-a/16000,6) for t in onsets]
            stream=io.StringIO(newline=''); writer=csv.writer(stream); writer.writerow(CSV_FIELDS)
            for t in hits:writer.writerow((wav.name,f'{t:.6f}','snare_hit'))
            record=dict(id=wav.stem,wav=str(wav.relative_to(output)),csv=str(labels.relative_to(output)),
                kind=block['kind'],hits=hits,group=session.name,collection_session=session.name,block=block['block'],
                split=block['split'],raw_offset_seconds=a/16000,duration=len(clip)/16000,
                commanded_snare_degrees=block.get('snare_degrees'),
                commanded_ride_degrees=block.get('ride_depth_deg'),hihat_degrees=90,
                source_capture=str(session.resolve()),annotation_method=review['method'],
                label_status=review.get('label_status','draft_visual_signal_review'),
                human_verified=False,auditory_review_performed=bool(review.get('auditory_review_performed',False)),
                notes=item.get('notes',''))
            if source_ride:
                record.update(group=source_ride['group'],source_ride_id=source_ride['id'],
                    source_ride_sha256=source_ride['wav_sha256'],source_capture=str(source_path.resolve()),
                    raw_offset_seconds=None,commanded_ride_degrees=source_ride.get('commanded_depth_deg'),
                    replaced_weak_capture_block=block['block'])
            planned.append((wav,labels,clip,stream.getvalue().encode(),record))
        for name in ('recordings','timestamps'):(output/name).mkdir(exist_ok=True)
        for wav,labels,clip,csvbytes,record in planned:
            with wav.open('xb') as f:
                with wave.open(f,'wb') as dest:
                    dest.setparams((1,2,16000,0,'NONE','not compressed'))
                    dest.writeframes(clip.astype('<i2').tobytes())
            with labels.open('xb') as f:f.write(csvbytes)
            record['wav_sha256']=hashlib.sha256(wav.read_bytes()).hexdigest()
            record['csv_sha256']=hashlib.sha256(csvbytes).hexdigest()
        records=[p[-1] for p in planned]
        manifest['records']=old+records
        temporary=manifest_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(manifest,indent=2)+'\n'); temporary.replace(manifest_path)
    return records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=Path);p.add_argument('review',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'X5data/snare')
    args=p.parse_args()
    records=export(args.session,json.loads(args.review.read_text()),args.output)
    print(json.dumps(dict(recordings=len(records),snare_hits=sum(len(r['hits']) for r in records)),indent=2))


if __name__=='__main__':main()
