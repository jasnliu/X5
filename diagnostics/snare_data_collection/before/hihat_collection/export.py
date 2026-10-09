"""Offline export of reviewed hi-hat onsets and target-specific negatives."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import wave
import numpy as np

from x5_collection.plan import CSV_FIELDS

ROOT=Path(__file__).resolve().parents[1]


def wav_read(path):
    with wave.open(str(path)) as source:
        if (source.getframerate(),source.getnchannels(),source.getsampwidth())!=(16000,1,2):
            raise ValueError('Expected 16 kHz mono PCM16 WAV')
        return np.frombuffer(source.readframes(source.getnframes()),dtype='<i2')


def export(session,review,output=ROOT/'X5data/hihat',ride=ROOT/'X5data/ride'):
    session,output,ride=map(Path,(session,output,ride))
    result=json.loads((session/'result.json').read_text())
    disabled=json.loads((session/'query_final_disabled.json').read_text())
    accessory=json.loads((session/'hihat_result.json').read_text())
    audio=json.loads((session/'audio_result.json').read_text())
    shutdown_notes=[]
    recovered=False
    recovery_path=session/'post_capture_recovery.json'
    if recovery_path.exists():
        recovery=json.loads(recovery_path.read_text())
        proof=json.loads((session/'recovery_verified.json').read_text())
        last_capture=max(r['end'] for r in json.loads((session/'examples.json').read_text()))
        recovered=(recovery['capture_completed_before_shutdown_issue']
                   and proof['all_16_disabled'] and proof['center']['t']>last_capture
                   and proof['center']['max_error_deg']<=.2
                   and proof['center']['span_deg']<=.12
                   and proof['center']['settled_s']>=.6)
        if recovered:shutdown_notes=recovery['issues']
    if not ((result['success'] and result['relaxed_verified']) or recovered) or not disabled['all_16_disabled_fault_free']:
        raise ValueError('Export requires successful verified center and arm relaxation')
    post_capture_watchdog=(recovered and accessory['error']=='RuntimeError: Collection parent heartbeat lost'
                           and accessory['open_command_at']>last_capture and not accessory['shutdown_error'])
    if (not accessory['release_ack'] or accessory.get('shutdown_error')
            or (accessory['error'] and not post_capture_watchdog) or audio['errors']):
        raise ValueError('Export requires hi-hat release ACK and clean audio')
    if not review.get('method'):raise ValueError('Explicit audio review provenance required')
    examples={r['block']:r for r in json.loads((session/'examples.json').read_text())}
    source_records={r['id']:r for r in json.loads((ride/'manifest.json').read_text())['records']}
    raw=wav_read(session/'raw.wav')
    records=[];planned=[]
    for item in review['examples']:
        if not item.get('accepted'):continue
        number=len(records)+1
        onsets=[float(t) for t in item.get('onsets_raw_seconds',[])]
        if any(not np.isfinite(t) for t in onsets) or any(b-a<.08 for a,b in zip(onsets,onsets[1:])):
            raise ValueError('Nonfinite or unordered/duplicate onsets')
        if 'ride_id' in item:
            if onsets:raise ValueError('Ride-only negatives cannot carry hi-hat labels')
            source=source_records[item['ride_id']]
            path=ride/source['wav'];clip=wav_read(path)
            if hashlib.sha256(path.read_bytes()).hexdigest()!=source['wav_sha256']:
                raise ValueError('Ride source hash mismatch')
            metadata=dict(kind='ride_negative',hits=[],group=source['group'],
                          source_ride_id=source['id'],source_ride_sha256=source['wav_sha256'])
        else:
            block=examples[item['block']]
            if len(onsets)!=block['closures']:
                raise ValueError('A mixed closure remains positive; annotation count must match reviewed block')
            start,end=float(item['clip_start_seconds']),float(item['clip_end_seconds'])
            if not np.isfinite([start,end]).all() or not 0<=start<end<=len(raw)/16000:
                raise ValueError('Clip outside raw audio')
            a,b=round(start*16000),round(end*16000)
            if any(not a/16000<=t<b/16000 for t in onsets):raise ValueError('Onset outside clip')
            clip=raw[a:b]
            metadata=dict(kind=block['kind'],hits=[round(t-a/16000,6) for t in onsets],
                          group=session.name,block=block['block'],raw_offset_seconds=a/16000,
                          commanded_ride_depth_deg=block.get('depth_deg'),
                          measured_ride_peak_deg=block.get('last_peak_deg'),
                          ride_command_offset_s=block.get('ride_command_offset_s'))
        if len(clip)==0 or np.mean(np.abs(clip.astype(np.int32))>=32767)>.001:
            raise ValueError('Empty or clipped audio requires review')
        wav=output/'recording'/f'r{number}.wav';csvpath=output/'timestamp'/f't{number}.csv'
        if wav.exists() or csvpath.exists():raise ValueError('Refusing to overwrite existing hi-hat data')
        labels=io.StringIO(newline='');writer=csv.writer(labels);writer.writerow(CSV_FIELDS)
        for t in metadata['hits']:writer.writerow((wav.name,f'{t:.6f}','hihat_close'))
        record=dict(id=wav.stem,wav=str(wav.relative_to(output)),csv=str(csvpath.relative_to(output)),
                    **metadata,duration=len(clip)/16000,split='unassigned',
                    annotation_method=review['method'],label_status='draft_visual_signal_review',
                    human_verified=False,auditory_review_performed=False,notes=item.get('notes',''))
        records.append(record);planned.append((wav,csvpath,clip,labels.getvalue().encode(),record))
    if not records:raise ValueError('No reviewed examples to export')
    if (output/'manifest.json').exists():raise ValueError('Refusing to replace existing manifest')
    for name in ('recording','timestamp'):(output/name).mkdir(parents=True,exist_ok=True)
    for wav,csvpath,clip,labels,record in planned:
        with wav.open('xb') as binary:
            with wave.open(binary,'wb') as dest:
                dest.setparams((1,2,16000,0,'NONE','not compressed'))
                dest.writeframes(clip.astype('<i2').tobytes())
        with csvpath.open('xb') as binary:binary.write(labels)
        record['wav_sha256']=hashlib.sha256(wav.read_bytes()).hexdigest()
        record['csv_sha256']=hashlib.sha256(labels).hexdigest()
    manifest=dict(version=1,target='hihat_close',grouping='physical-session',training_performed=False,
                  source_capture=str(session.resolve()),shutdown_notes=shutdown_notes,records=records)
    with (output/'manifest.json').open('x') as dest:dest.write(json.dumps(manifest,indent=2)+'\n')
    return records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=Path);p.add_argument('review',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'X5data/hihat')
    args=p.parse_args()
    records=export(args.session,json.loads(args.review.read_text()),args.output)
    print(json.dumps(dict(recordings=len(records),closures=sum(len(r['hits']) for r in records)),indent=2))


if __name__=='__main__':main()
