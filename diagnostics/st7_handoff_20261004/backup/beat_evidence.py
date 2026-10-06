"""Opt-in, bounded physical acceptance run of the ordinary hardware workflow.

Never substitutes encoders, audio, camera decisions, or the strike controller.
The caller must arrange the motion warning and clear the workspace first.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid


class BeatEvidence:
    def __init__(self, app, seconds=5.):
        self.app, self.seconds = app, seconds
        self.directory = Path(__file__).resolve().parents[1] / 'playback_results' / 'swing_verification' / (
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True)
        self.log = open(self.directory/'telemetry.jsonl', 'w', buffering=1)
        self.started = self.stopping = False
        self.created = time.monotonic()
        self.last_phase = None
        self.last_bottom = 0
        self.bottoms, self.hits, self.faults = [], [], []
        for name in ('fail', '_program_failure'):
            original = getattr(app, name)
            def wrapped(message, original=original, name=name):
                self.faults.append(dict(t=time.monotonic(), source=name, message=str(message)))
                return original(message)
            setattr(app, name, wrapped)
        print('PHYSICAL VERIFICATION EVIDENCE: '+str(self.directory), flush=True)
        app.root.after(50, self.tick)

    def audio(self, message):
        self.hits.append(dict(message))
        self.log.write(json.dumps(dict(message, kind='audio'))+'\n')

    def finish(self):
        duration = self.bottoms[-1]['at']-self.bottoms[0]['at'] if len(self.bottoms)>1 else 0.
        hits = [h for h in self.hits if self.bottoms and
                self.bottoms[0]['at']-.15 <= h['event_at'] <= self.bottoms[-1]['at']+.15]
        matched = sum(any(abs(h['event_at']-b['at']) <= .15 for h in hits) for b in self.bottoms)
        audio_span = max(h['event_at'] for h in hits)-min(h['event_at'] for h in hits) if len(hits)>1 else 0.
        center = getattr(self.app.bus, 'last_center_evidence', None)
        disabled = (self.app.bus.fresh() and all(self.app.bus.states['right', i][1] == 0 for i in range(1, 9)))
        motion_passed = duration >= self.seconds and not self.faults and center is not None and disabled
        audio_passed = len(hits) >= 8 and matched >= 8 and audio_span >= 4.
        passed = motion_passed and audio_passed
        result = dict(success=passed, physical=True, duration_s=duration,
                      motion_success=motion_passed, st7_repeat_confirmation=audio_passed,
                      audio_review_needed=bool(motion_passed and not audio_passed),
                      encoder_strikes=len(self.bottoms), audio_hits=len(hits),
                      audio_matched_strikes=matched, audio_span_s=audio_span,
                      faults=self.faults, center=center, relaxed_verified=disabled,
                      bottoms=self.bottoms, audio=hits)
        (self.directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print('PHYSICAL SWING RESULT: '+json.dumps(result), flush=True)
        self.log.close()
        self.app.request_safe_close()

    def tick(self):
        a, now = self.app, time.monotonic()
        try:
            s = a.hybrid_session.status if a.hybrid_session is not None else None
            row = dict(t=now, phase=a.phase, status=a.status.get(), q=a.arm().tolist(),
                       feedback_fresh=a.bus.fresh(), active=a.bus.active,
                       audio_finalized=a.audio_receiver.finalized_at,
                       hybrid=None if s is None else asdict(s))
            self.log.write(json.dumps(row)+'\n')
            if a.phase != self.last_phase:
                self.last_phase = a.phase
                print('VERIFICATION PHASE: '+json.dumps(dict(t=now, phase=a.phase, status=a.status.get())), flush=True)
            if not self.started:
                if (a.phase == 'READY' and a.playback_trajectory is not None and a.bus.fresh()
                        and a._camera_ready_for_start() and a._sound_ready_for_start()
                        and a._hihat_ready_for_start()):
                    self.started = True
                    a._refresh_buttons()
                    a.continue_button.invoke()  # Exactly the normal visible RUN button.
                elif now-self.created > 90:
                    self.faults.append(dict(message='Readiness timeout; no run'))
                    self.finish()
                    return
            if a.continuous_strike_active and s is not None and s.swing:
                if s.bottoms > self.last_bottom:
                    self.last_bottom = s.bottoms
                    self.bottoms.append(dict(at=s.bottom_at, count=s.count,
                        lateness=s.last_lateness, peak=s.peak_drop, partial_returns=s.partial_returns,
                        q=s.sample.position, target_degrees=a.continuous_strike_degrees))
                if self.bottoms and self.bottoms[-1]['at']-self.bottoms[0]['at'] >= self.seconds and not self.stopping:
                    self.stopping = True
                    print('VERIFICATION: five seconds of swing measured; Center + Relax requested', flush=True)
                    a.center_relax()
            if self.started and a.phase == 'RELAXED':
                self.finish()
                return
            if now-self.created > 180 and not self.stopping:
                self.stopping = True
                self.faults.append(dict(message='Verification exceeded 180 seconds'))
                a.center_relax()
        except Exception as exc:
            self.faults.append(dict(message='Evidence monitor: '+str(exc)))
            self.stopping = True
            a.fail('Evidence monitor: '+str(exc))
        a.root.after(20, self.tick)
