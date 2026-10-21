"""Export reviewed acoustic onsets; motor command times are not annotations."""
import argparse
from collections import Counter
import csv
import fcntl
import hashlib
import io
import json
import math
from pathlib import Path
import wave

import numpy as np
from hihat_collection.export import wav_read
from x5_collection.plan import CSV_FIELDS
from .plan import PULSE, make_plan

ROOT=Path(__file__).resolve().parents[1]
LABELS={'bass_hit'}


def read_json(path): return json.loads(Path(path).read_text())
def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def export(session, review, output=ROOT/'X5data/bass'):
    session,output=Path(session),Path(output)
    result=read_json(session/'result.json')
    disabled=read_json(session/'query_final_disabled.json')
    accessory=read_json(session/'hihat_result.json')
    audio=read_json(session/'audio_result.json')
    centers=read_json(session/'both_center_before_relax.json')
    if not(result['success'] and result['relaxed_verified'] and disabled['all_16_disabled_fault_free']) or result.get('hardware_emergency'):
        raise ValueError('Successful centered shutdown required')
    for side in ('left','right'):
        p=centers[side]
        if not all(math.isfinite(p[k]) for k in ('max_error_deg','span_deg','settled_s')) or p['max_error_deg']>.2 or p['span_deg']>.12 or p['settled_s']<.6:
            raise ValueError('Strict dual-arm center proof required')
    if (accessory.get('error') or accessory.get('shutdown_error') or not accessory['release_ack']
            or not accessory['encoder_position_verified'] or accessory['bass_pulse']!=PULSE or audio['errors']):
        raise ValueError('Clean audio, fixed pulse and verified accessory return required')
    if not review.get('method'): raise ValueError('Audio review provenance required')
    examples={r['block']:r for r in read_json(session/'examples.json')}
    if len(review['examples'])!=len(examples) or {r['block'] for r in review['examples']}!=set(examples):
        raise ValueError('Review every captured block exactly once')
    raw=wav_read(session/'raw.wav'); planned=[]
    for item in review['examples']:
        if item.get('accepted') is not True: raise ValueError('Uncertain audio must be reviewed, not fabricated')
        block=examples[item['block']]; n=block['block']
        start,end=float(item['clip_start_seconds']),float(item['clip_end_seconds'])
        if not np.isfinite([start,end]).all() or not 0<=start<end<=len(raw)/16000:
            raise ValueError('Invalid clip boundaries')
        a,b=round(start*16000),round(end*16000); clip=raw[a:b]
        if not len(clip) or np.any(np.abs(clip.astype(np.int32))>=32767):
            raise ValueError('Empty or clipped audio must be recollected')
        events=item['events']; counts=Counter(e['label'] for e in events)
        expected=dict(bass_hit=block['bass_hits'])
        if set(counts)-LABELS: raise ValueError('Unknown label')
        for label,count in expected.items():
            if counts[label]!=count and not (label=='bass_hit' and counts[label]>count and item.get('extra_bass_impacts_verified') is True):
                raise ValueError(f'{label}: verify every impact or recollect missing hits')
        annotations=[]
        for event in sorted(events,key=lambda e:float(e['raw_seconds'])):
            t=float(event['raw_seconds'])
            if not math.isfinite(t) or not a/16000<=t<b/16000: raise ValueError('Onset outside clip')
            annotations.append(dict(label=event['label'],time=round(t-a/16000,6)))
        for label in LABELS:
            hits=[e['time'] for e in annotations if e['label']==label]
            if any(y-x<.02 for x,y in zip(hits,hits[1:])):raise ValueError('Duplicate onset annotation')
        wav=output/'recordings'/f'r{n}.wav'; labels=output/'timestamps'/f't{n}.csv'
        stream=io.StringIO(newline='');writer=csv.writer(stream);writer.writerow(CSV_FIELDS)
        for e in annotations: writer.writerow((wav.name,f"{e['time']:.6f}",e['label']))
        record=dict(id=wav.stem,wav=str(wav.relative_to(output)),csv=str(labels.relative_to(output)),
            kind=block['kind'],split=block['split'],group=session.name,collection_session=session.name,
            block=n,bpm=block['bpm'],bass_pulse=PULSE,hits=[e['time'] for e in annotations if e['label']=='bass_hit'],
            annotations=annotations,raw_offset_seconds=a/16000,duration=len(clip)/16000,
            annotation_method=review['method'],label_status='assistant_visual_signal_review',
            annotation_scope='bass_only',exhaustively_labeled_instruments=['bass'],
            accompaniment_labels_complete=False,
            left_return_policy='reverse_then_center' if result.get('left_reverse_verified') else 'centered_before_new_reverse_requirement',
            auditory_review_performed=bool(review.get('auditory_review_performed',False)),human_verified=False,
            source_capture=str(session.resolve()),notes=item.get('notes',''))
        planned.append((wav,labels,clip,stream.getvalue(),record))
    output.mkdir(parents=True,exist_ok=True)
    for name in ('recordings','timestamps'): (output/name).mkdir(exist_ok=True)
    with (output/'.export.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        manifest_path=output/'manifest.json'
        manifest=read_json(manifest_path) if manifest_path.exists() else dict(version=1,target='bass_hit',
            grouping='physical-session',training_performed=False,annotation_scope='bass_only',records=[])
        if manifest['target']!='bass_hit' or manifest['training_performed'] is not False:
            raise ValueError('Wrong output dataset')
        oldids={r['id'] for r in manifest['records']}
        if oldids & {r['id'] for *_,r in planned}: raise ValueError('Block already exported')
        if any(p.exists() for w,c,*_ in planned for p in (w,c)):raise ValueError('Refusing to overwrite files')
        created=[]
        try:
            for wav,labels,clip,content,record in planned:
                with wav.open('xb') as stream:
                    created.append(wav)
                    with wave.open(stream,'wb') as dest:
                        dest.setparams((1,2,16000,0,'NONE','not compressed'));dest.writeframes(clip.astype('<i2').tobytes())
                with labels.open('x',newline='') as dest: created.append(labels);dest.write(content)
                record.update(wav_sha256=digest(wav),csv_sha256=digest(labels))
            manifest['annotation_scope']='bass_only'
            manifest['records'].extend(r for *_,r in planned)
            manifest['records'].sort(key=lambda r:int(r['id'][1:]))
            temporary=manifest_path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(manifest,indent=2)+'\n');temporary.replace(manifest_path)
        except BaseException:
            for path in created: path.unlink()
            raise
    return [r for *_,r in planned]


def verify(output=ROOT/'X5data/bass'):
    output=Path(output); manifest=read_json(output/'manifest.json'); records=manifest['records']
    plan={r['block']:r for r in make_plan()}
    assert manifest['target']=='bass_hit' and manifest['training_performed'] is False
    assert manifest['annotation_scope']=='bass_only'
    assert len(records)==60 and {r['id'] for r in records}=={f'r{i}' for i in range(1,61)}
    assert {p.name for p in (output/'recordings').iterdir()}=={f'r{i}.wav' for i in range(1,61)}
    assert {p.name for p in (output/'timestamps').iterdir()}=={f't{i}.csv' for i in range(1,61)}
    groups={};hashes=set();counts=Counter()
    for r in records:
        n=int(r['id'][1:]); wav=output/r['wav']; csvpath=output/r['csv']
        assert wav.name==f'r{n}.wav' and csvpath.name==f't{n}.csv'
        assert digest(wav)==r['wav_sha256'] and digest(csvpath)==r['csv_sha256']
        assert r['wav_sha256'] not in hashes;hashes.add(r['wav_sha256'])
        assert r['kind']==plan[n]['kind'] and r['split']==plan[n]['split'] and r['bass_pulse']==PULSE
        assert r['annotation_scope']=='bass_only' and r['exhaustively_labeled_instruments']==['bass']
        assert r['accompaniment_labels_complete'] is False
        assert groups.setdefault(r['group'],r['split'])==r['split']
        raw=wav_read(wav); assert len(raw)>0 and not np.any(np.abs(raw.astype(np.int32))>=32767)
        with csvpath.open() as src:
            reader=csv.DictReader(src); assert tuple(reader.fieldnames)==CSV_FIELDS;rows=list(reader)
        assert len(rows)==len(r['annotations'])
        for row,event in zip(rows,r['annotations']):
            t=float(row['hit_time_seconds']); assert 0<=t<len(raw)/16000
            assert row['recording_file']==wav.name and row['label']==event['label'] and t==event['time']
            assert row['label'] in LABELS
            counts[row['label']]+=1
        assert sum(x['label']=='bass_hit' for x in rows)>=plan[n]['bass_hits']
    return dict(recordings=60,timestamp_files=60,labels=dict(counts),sessions=len(groups),
                category_counts=dict(Counter(r['kind'] for r in records)),training_performed=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path,nargs='?')
    p.add_argument('--review',type=Path);p.add_argument('--verify',action='store_true')
    a=p.parse_args()
    if a.verify: print(json.dumps(verify(),indent=2));return
    if not a.session or not a.review:p.error('session and --review required')
    print(json.dumps(export(a.session,read_json(a.review)),indent=2))

if __name__=='__main__': main()
