"""Append-only trial data and content-addressed source snapshots. No trial deletion."""
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import threading
import uuid
from .config import ROOT, TARGET_DEG, ZONE_DEG
from .scoring import identity, summarize


def save_json(path, data):
    # Used for new, unique files only. Never replace a previous trial/analysis.
    with Path(path).open('x') as f:
        json.dump(data, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')


class Store:
    def __init__(self, root, backend, rules, limits, extra=None):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8]
        self.path = Path(root).expanduser().resolve()/stamp
        self.path.mkdir(parents=True, exist_ok=False)
        self.backend, self.rules, self.limits = backend, rules, limits
        self.trials = []
        self.error = None
        self.queue = queue.Queue(maxsize=64)
        source = self.path/'source'
        manifest = {}
        files = list((ROOT/'strike_lab').rglob('*.py')) + [ROOT/'experiment.sh', ROOT/'launch_experiment.py',
                ROOT/'camera_playback/mit_strike.py', ROOT/'centering/motors.py',
                ROOT/'safe_zone/encoder.py', ROOT/'model/openarmx.urdf', ROOT/'config/experiment.rviz']
        for p in sorted(files):
            if not p.is_file():
                continue
            rel = p.relative_to(ROOT); target = source/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p,target)
            manifest[str(rel)] = hashlib.sha256(target.read_bytes()).hexdigest()
        self.source_id = identity(manifest)
        save_json(self.path/'session.json', dict(backend=backend, rules=asdict(rules), limits=asdict(limits),
                  target_deg=TARGET_DEG, zone_deg=ZONE_DEG, source_id=self.source_id,
                  source_manifest=manifest, initial_tuning_not_calibrated=True, extra=extra or {}))
        self.thread = threading.Thread(target=self._write, name='experiment-archive', daemon=True)
        self.thread.start()

    def _write(self):
        handles = {}
        try:
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        return
                    kind, trial, data = item
                    directory = self.path/trial
                    if kind == 'rows':
                        if trial not in handles:
                            f = (directory/'trace.csv').open('x', newline='')
                            writer = csv.DictWriter(f,fieldnames=list(data[0]))
                            writer.writeheader();handles[trial]=(f,writer)
                        f,writer=handles[trial];writer.writerows(data);f.flush()
                    elif kind == 'finish':
                        if trial in handles:
                            f,_ = handles.pop(trial);f.flush();os.fsync(f.fileno());f.close()
                        save_json(directory/'score.json',data)
                finally:
                    self.queue.task_done()
        except BaseException as exc:
            self.error = repr(exc)
        finally:
            for f,_ in handles.values():
                f.close()

    def check(self):
        if self.error:
            raise RuntimeError('Trial archive failed: '+self.error)

    def begin(self, method, parameters, stage, anchor, bias, parent=None):
        self.check()
        trial = f'{len(self.trials)+1:05d}-{method}-{uuid.uuid4().hex[:6]}'
        directory = self.path/trial;directory.mkdir()
        meta = dict(method=method, parameters=parameters, parameters_id=identity(parameters),
                    backend=self.backend, stage=stage, anchor_rad=anchor, bias_torque=bias,
                    target_deg=TARGET_DEG, source_id=self.source_id, parent_trial=parent)
        save_json(directory/'metadata.json',meta)
        self.trials.append(dict(id=trial,metadata=meta))
        return trial

    def enqueue(self, item):
        self.check()
        try:
            if self.backend=='simulation':self.queue.put(item,timeout=5)
            else:self.queue.put_nowait(item)
        except queue.Full:raise RuntimeError('Archive queue full; refusing to lose trial data') from None

    def rows(self, trial, rows):
        self.check()
        if rows:
            self.enqueue(('rows',trial,list(rows)))

    def finish(self, trial, score):
        self.check()
        self.enqueue(('finish',trial,score))
        next(t for t in self.trials if t['id']==trial)['score']=score

    def report(self):
        complete=[t for t in self.trials if 'score' in t]
        analysis=self.path/('analysis-'+uuid.uuid4().hex[:8]+'.json')
        data=summarize(complete,self.rules.depth_match_deg)
        save_json(analysis,data)
        lines=['# Strike lab results', '', data['disclaimer'], '',
               '| Method | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |',
               '|---|---|---:|---:|---:|---:|---:|---:|']
        for g in data['groups']:
            vals=[g.get(k) for k in ('lower_zone_ms_median','lower_zone_ms_p95','peak_depth_deg_median','cycle_ms_median')]
            lines.append(f"| {g['method']} | {g['stage']} | {g['attempts']} | {g['passed']} | "+' | '.join('—' if v is None else f'{v:.3f}' for v in vals)+' |')
        lines += ['', '## Comparisons', '```json',json.dumps(data['comparisons'],indent=2),'```',
                  '', 'Failed, interrupted, and rejected attempts remain in their individual trial directories.']
        analysis.with_suffix('.md').write_text('\n'.join(lines)+'\n')
        return analysis

    def close(self):
        if self.thread.is_alive():
            self.queue.put(None,timeout=5)
            self.thread.join(timeout=10)
        self.check()
        if self.thread.is_alive():raise RuntimeError('Archive did not finish flushing')


def load_trials(path):
    trials=[]
    for p in sorted(Path(path).glob('*/metadata.json')):
        score_path=p.parent/'score.json'
        if score_path.exists():
            trials.append(dict(id=p.parent.name,metadata=json.loads(p.read_text()),
                               score=json.loads(score_path.read_text())))
    return trials


def rescore_session(path, rules, limits):
    from .scoring import score
    directory=Path(path)/('rescore-'+uuid.uuid4().hex[:8]);directory.mkdir()
    trials=[]
    for p in sorted(Path(path).glob('*/metadata.json')):
        trace=p.parent/'trace.csv'
        if not trace.exists():continue
        with trace.open() as f:
            rows=list(csv.DictReader(f))
        for row in rows:
            for k in row:
                if k=='phase':continue
                row[k]=row[k]=='True' if k=='limited' else float(row[k])
        original=p.parent/'score.json'
        old=json.loads(original.read_text()) if original.exists() else {}
        abort=next((r for r in old.get('reasons',[]) if r.startswith('aborted:')),None)
        if not original.exists():abort='original trial did not finish'
        result=score(rows,rules,limits,abort)
        save_json(directory/(p.parent.name+'.json'),result)
        trials.append(dict(metadata=json.loads(p.read_text()),score=result))
    save_json(directory/'summary.json',summarize(trials,rules.depth_match_deg))
    save_json(directory/'rules.json',dict(rules=asdict(rules),limits=asdict(limits)))
    return directory
