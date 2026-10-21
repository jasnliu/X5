"""Opt-in physical acceptance of NORMAL dual-arm playback, without any beat.

Uses the real Run button, readiness/perception, snare mode preparation, recording
workers and return controllers. No fake encoders or altered motor limits. A
failure leaves the existing safe recovery/hold in charge, never forces relax.
The caller must send the motion warning before launching this auto-run mode.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import traceback
import uuid


class PlaybackEvidence:
    def __init__(self, app):
        self.app = app
        self.directory = Path(__file__).resolve().parents[1]/'playback_results'/'dual_playback_verification'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True)
        self.stream = (self.directory/'telemetry.jsonl').open('w', buffering=1)
        self.created = time.monotonic()
        self.started = self.endpoint = self.finished = False
        self.snare_armed = False
        self.faults = []
        self.return_fault = None
        self.phases = []
        self.last_phase = None
        self.last_request = None
        self.last_log = 0.
        self.disabled_proofs = {}
        for name in ('fail', '_program_failure'):
            original = getattr(app, name)
            def failed(message, original=original, name=name):
                row = dict(at=time.monotonic(), source=name, phase=app.phase,
                           message=str(message), traceback=traceback.format_exc())
                if not self.faults or self.faults[-1]['message'] != str(message):
                    self.faults.append(row)
                    print('PLAYBACK VERIFY FAULT: '+json.dumps(row), flush=True)
                    self.save(False)
                return original(message)
            setattr(app, name, failed)
        def no_beat(*_args, **_kwargs):
            raise RuntimeError('Strike forbidden in --verify-playback')
        app._begin_strike_attempt = app._begin_continuous_striking = no_beat
        app._begin_playback_alignment = self.recording_endpoint
        original_disable = app.bus.disable_centered_side
        def disable(side):
            result = original_disable(side)
            self.disabled_proofs[side] = dict(at=time.monotonic(),
                q=[app.bus.states[side, i][0] for i in range(1,8)])
            return result
        app.bus.disable_centered_side = disable
        print('PHYSICAL PLAYBACK EVIDENCE: '+str(self.directory), flush=True)
        app.root.after(50, self.tick)

    def recording_endpoint(self, now=None):
        self.endpoint = True
        print('PLAYBACK VERIFY: both animations complete; skipping alignment/beat, Center + Relax', flush=True)
        self.app.center_relax()

    def save(self, success):
        a = self.app
        result = dict(success=success, physical=True, beat_skipped=True, started=self.started,
            right_endpoint_reached=self.endpoint, left_recording_complete=a.dual.left_done,
            snare_mode_prepared=self.snare_armed, faults=self.faults, phases=self.phases,
            disabled_proofs=self.disabled_proofs,
            all_16_disabled=a.bus.fresh() and len(a.bus.states)==16 and all(s[1]==0 for s in a.bus.states.values()))
        (self.directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        return result

    def tick(self):
        if self.finished:
            return
        a, now = self.app, time.monotonic()
        try:
            s = getattr(a.bus, 'left_joint6_session', None)
            status = None if s is None else s.status
            self.snare_armed |= status is not None and status.armed
            left_return = a.dual.left_return
            failure = None if left_return is None else left_return.failure
            if failure and failure != self.return_fault:
                self.return_fault = failure
                self.faults.append(dict(at=now, source='LeftReturn', phase=a.phase, message=failure))
                print('PLAYBACK VERIFY FAULT: '+json.dumps(self.faults[-1]), flush=True)
                self.save(False)
            if a.phase != self.last_phase:
                self.last_phase = a.phase
                row = dict(at=now, phase=a.phase, status=a.status.get())
                self.phases.append(row)
                print('PLAYBACK VERIFY PHASE: '+json.dumps(row), flush=True)
            if now-self.last_log >= .025:
                self.last_log = now
                row = dict(at=now, phase=a.phase, status=a.status.get(), active=a.bus.active,
                    states={f'{side}{i}': list(v) for (side,i),v in a.bus.states.items()},
                    snare=None if status is None else asdict(status))
                self.stream.write(json.dumps(row)+'\n')
            # Recovery requests are deliberately restricted to the existing
            # centered-stop command or closing AFTER disabled confirmation.
            path = self.directory/'request.json'
            if path.exists():
                raw = path.read_text()
                if raw != self.last_request:
                    self.last_request = raw
                    action = json.loads(raw)['action']
                    if action == 'center': a.center_relax()
                    elif action == 'close' and not a.bus.active and all(s[1]==0 for s in a.bus.states.values()):
                        self.finished = True
                        self.save(False); self.stream.close(); a.root.quit(); return
                    else: raise RuntimeError('Verification request refused')
            if not self.started and not self.faults:
                if (a.phase == 'READY' and a.dual.ready and a.playback_trajectory is not None
                        and a.bus.fresh() and a._camera_ready_for_start()
                        and a._sound_ready_for_start() and a._hihat_ready_for_start()):
                    a._refresh_buttons()
                    if a.continue_button.cget('state') == 'normal':
                        self.started = True
                        a.continue_button.invoke()
                elif now-self.created > 300:
                    self.faults.append(dict(message='Readiness timeout before Run'))
                    self.save(False)
            if self.started and a.phase == 'RELAXED' and not a.bus.active:
                disabled = a.bus.fresh() and all(s[1]==0 for s in a.bus.states.values())
                passed = (not self.faults and self.endpoint and a.dual.left_done
                          and self.snare_armed and set(self.disabled_proofs)=={'left','right'} and disabled)
                result = self.save(passed)
                print('PHYSICAL PLAYBACK RESULT: '+json.dumps(result), flush=True)
                self.finished = True
                self.stream.close()
                a.request_safe_close()
                return
        except Exception as exc:
            a.fail('Playback evidence: '+str(exc))
        a.root.after(20, self.tick)
