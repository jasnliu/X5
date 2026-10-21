"""Opt-in physical Run acceptance; real perception/recordings/hi-hat calibration.

After hi-hat timing locks, finish returning and verify BOTH arms centered/disabled.
No snare calibration or snare strikes are expected. This mode never enters swing.
Caller warns of motion.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

class CalibrationEvidence:
    def __init__(self, app):
        self.app = app
        self.directory = Path(__file__).resolve().parents[1]/'playback_results'/'calibration_verification'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True)
        self.stream = (self.directory/'telemetry.jsonl').open('w', buffering=1)
        self.created = time.monotonic()
        self.started = self.finished = self.stopping = False
        self.faults, self.phases, self.disabled_proofs = [], [], {}
        self.accepted_search_hit = self.last_phase = self.last_request = None
        self.snare_strikes = 0
        for name in ('fail', '_program_failure'):
            original = getattr(app, name)
            def failed(message, original=original, name=name):
                self.faults.append(dict(at=time.monotonic(), source=name, message=str(message)))
                return original(message)
            setattr(app, name, failed)
        original_disable = app.bus.disable_centered_side
        def disable(side):
            result = original_disable(side)
            self.disabled_proofs[side] = dict(at=time.monotonic(),
                q=[app.bus.states[side, i][0] for i in range(1,8)])
            return result
        app.bus.disable_centered_side = disable
        print('PHYSICAL CALIBRATION EVIDENCE: '+str(self.directory), flush=True)
        app.root.after(50, self.tick)

    def audio(self, message):
        if not self.finished:
            self.stream.write(json.dumps(dict(message, kind='audio'))+'\n')

    def selected(self, message, degrees, released_at, returned_at):
        self.accepted_search_hit = dict(message, degrees=degrees,
                                       released_at=released_at, returned_at=returned_at)

    def finish(self):
        a = self.app
        disabled = (a.bus.fresh() and len(a.bus.states)==16
                    and all(s[1]==0 for s in a.bus.states.values()))
        passed = (self.started and not self.faults and a.dual.left_done
                  and self.accepted_search_hit is not None
                  and a.hihat_sync.calibrated_advance is not None
                  and self.snare_strikes == 0
                  and set(self.disabled_proofs)=={'left','right'} and disabled)
        result = dict(success=passed, physical=True, swing_skipped=True,
                      snare_outcome='disabled', snare_calibrated=False,
                      snare_strikes=self.snare_strikes,
                      hihat_advance=a.hihat_sync.calibrated_advance,
                      snare_advance=0.,
                      accepted_search_hit=self.accepted_search_hit,
                      faults=self.faults, phases=self.phases,
                      disabled_proofs=self.disabled_proofs, all_16_disabled=disabled)
        (self.directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print('PHYSICAL CALIBRATION RESULT: '+json.dumps(result), flush=True)
        self.finished = True
        self.stream.close()
        a.request_safe_close()

    def tick(self):
        if self.finished:
            return
        a, now = self.app, time.monotonic()
        try:
            worker = getattr(a.bus, 'left_joint6_session', None)
            s = None if worker is None else worker.status
            ride = None if a.hybrid_session is None else a.hybrid_session.status
            if s is not None:
                self.snare_strikes = max(self.snare_strikes, s.strike_index)
            if a.phase != self.last_phase:
                self.last_phase = a.phase
                self.phases.append(dict(at=now, phase=a.phase, status=a.status.get()))
                print('CALIBRATION VERIFY PHASE: '+json.dumps(self.phases[-1]), flush=True)
            self.stream.write(json.dumps(dict(at=now, phase=a.phase, status=a.status.get(),
                states={f'{side}{i}': list(v) for (side,i),v in a.bus.states.items()},
                snare=None if s is None else asdict(s),
                ride=None if ride is None else asdict(ride)))+'\n')
            left_return = a.dual.left_return
            if left_return is not None and left_return.failure:
                row = dict(source='left_return', message=left_return.failure)
                if row not in self.faults:
                    self.faults.append(row)
            # Only ordinary safe-stop is exposed; no emergency disable shortcut.
            path = self.directory/'request.json'
            if path.exists() and (raw := path.read_text()) != self.last_request:
                self.last_request = raw
                if json.loads(raw)['action'] != 'center':
                    raise RuntimeError('Only controlled center requests are permitted')
                self.stopping = True
                self.faults.append(dict(message='External early stop requested'))
                a.center_relax()
            if not self.started and not self.faults:
                if (a.phase == 'READY' and a.dual.ready and a.playback_trajectory is not None
                        and a.bus.fresh() and a._camera_ready_for_start()
                        and a._sound_ready_for_start() and a._hihat_ready_for_start()):
                    a._refresh_buttons()
                    if a.continue_button.cget('state') == 'normal':
                        self.started = True
                        a.continue_button.invoke()
            if self.started and a.phase == 'RELAXED' and not a.bus.active:
                self.finish()
                return
            if now-self.created > 180 and not self.stopping:
                self.stopping = True
                self.faults.append(dict(message='Verification exceeded 180 seconds'))
                a.center_relax()
        except Exception as exc:
            a.fail('Calibration evidence: '+str(exc))
        a.root.after(25, self.tick)
