"""One scoring implementation for every method. Raw zone timing is never smoothed."""
from dataclasses import asdict
from collections import Counter
import hashlib
import json
import math
import numpy as np
from .config import TARGET_DEG, ZONE_DEG, Rules, Limits


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]


def zone_measurements(times, depths, boundary=ZONE_DEG):
    """Piecewise-linear integral of depth > boundary, including ALL reentries."""
    total = 0.
    entries, exits, brackets = [], [], []
    for ta, tb, a, b in zip(times, times[1:], depths, depths[1:]):
        if tb <= ta:
            raise ValueError('Encoder timestamps must strictly increase')
        if a > boundary and b > boundary:
            total += tb-ta
        elif a <= boundary < b:
            cross = ta+(tb-ta)*(boundary-a)/(b-a)
            entries.append(cross); brackets.append(tb-ta)
            total += tb-cross
        elif a > boundary >= b:
            cross = ta+(tb-ta)*(boundary-a)/(b-a)
            exits.append(cross); brackets.append(tb-ta)
            total += cross-ta
    return total, entries, exits, sum(brackets)


def score(rows, rules=Rules(), limits=Limits(), abort=None):
    reasons = []
    # The raw log also preserves duplicate feedback used by intervening commands.
    # It must NOT turn duplicate feedback into new measurement samples.
    samples = []
    for r in rows:
        if not samples or r['sample_at'] > samples[-1]['sample_at']:
            samples.append(r)
        elif r['sample_at'] < samples[-1]['sample_at']:
            reasons.append('out_of_order_feedback')
    result = dict(target_deg=TARGET_DEG, zone_deg=ZONE_DEG,
                  rules_id=identity(asdict(rules)), limits_id=identity(asdict(limits)),
                  passed=False, reasons=[], samples=len(samples))
    if abort:
        reasons.append('aborted: '+abort)
    if len(samples) < 5:
        result['reasons'] = reasons+['insufficient_feedback']
        return result
    t = np.array([r['sample_at'] for r in samples])
    depth = np.array([r['depth_deg'] for r in samples])
    velocity = np.array([-r['velocity'] for r in samples])
    if not np.all(np.isfinite(np.concatenate((t, depth, velocity)))):
        result['reasons'] = reasons+['nonfinite_feedback']
        return result
    total, entries, exits, uncertainty = zone_measurements(t, depth)
    peak = float(np.max(depth)); low = int(np.argmax(depth))
    if abs(peak-TARGET_DEG) > rules.acceptance_deg:
        reasons.append('depth_outside_acceptance')
    if not entries or not exits or depth[-1] > ZONE_DEG:
        reasons.append('incomplete_zone_crossing')
    def crossing_speed(at):
        i=int(np.searchsorted(t,at));lo=max(0,i-3);hi=min(len(t),i+4)
        return float(np.polyfit(t[lo:hi]-at,np.radians(depth[lo:hi]),1)[0])
    entry_speed = crossing_speed(entries[0]) if entries else 0.
    exit_speed = -crossing_speed(exits[-1]) if exits else 0.
    if entry_speed < rules.minimum_entry_speed:
        reasons.append('insufficient_entry_speed')
    approach = float(entries[0]-t[0]) if entries else float(t[low]-t[0])
    if approach > rules.maximum_approach_seconds:
        reasons.append('slow_staged_approach')
    # Small encoder fluctuations are not extra physical reversals. Meaningful
    # backtracking >0.08 degrees before/after the main turnaround is rejected.
    if (np.max(np.maximum.accumulate(depth[:low+1])-depth[:low+1]) > .08
            or np.max(depth[low:]-np.minimum.accumulate(depth[low:])) > .08):
        reasons.append('secondary_reversal')
    # Reject creeping/staging before 9 degrees, excluding initial launch and
    # normal braking in the virtual zone. Uses a short measured-position window.
    longest_pause = 0.
    for i in range(low):
        j = int(np.searchsorted(t, t[i]+rules.maximum_prezone_pause))
        if j <= low and j < len(t) and 2 < depth[i] < 8.95:
            if np.ptp(depth[i:j+1]) < .08:
                longest_pause = max(longest_pause, float(t[j]-t[i]))
    if longest_pause >= rules.maximum_prezone_pause:
        reasons.append('prezone_pause')
    if float(np.max(np.diff(t))) > rules.maximum_feedback_gap:
        reasons.append('feedback_gap')
    cycle = float(t[-1]-t[0])
    if cycle > rules.maximum_cycle_seconds:
        reasons.append('slow_cycle')
    first=max(0,int(np.searchsorted(t,t[-1]-rules.settle_seconds,side='right'))-1)
    recent = depth[first:]
    returned = (abs(depth[-1]) <= rules.return_tolerance_deg
                and len(recent) >= 3 and t[-1]-t[first]>=rules.settle_seconds-1e-9
                and np.max(np.abs(recent)) <= rules.return_tolerance_deg
                and np.ptp(recent) <= rules.settle_range_deg)
    if not returned:
        reasons.append('return_not_settled')
    overshoot = max(0., -float(np.min(depth[low:])))
    if overshoot > rules.maximum_upper_overshoot_deg:
        reasons.append('upper_overshoot')
    # Polynomial fit to measured drive velocity over up to 11 feedback samples.
    # Only smoothness diagnostics use this; crossing/depth scores use raw q.
    accelerations, jerks = [], []
    for i in range(5, len(t)-5):
        dt = t[i-5:i+6]-t[i]
        coef = np.polynomial.polynomial.polyfit(dt, velocity[i-5:i+6], 2)
        accelerations.append(abs(float(coef[1])))
        jerks.append(abs(float(2*coef[2])))
    acc99 = float(np.percentile(accelerations, 99)) if accelerations else 0.
    jerk99 = float(np.percentile(jerks, 99)) if jerks else 0.
    peak_speed = float(np.max(np.abs(velocity)))
    if peak_speed > limits.velocity*1.1:
        reasons.append('speed_spike')
    if acc99 > limits.acceleration*1.2 or jerk99 > limits.jerk*1.2:
        reasons.append('motion_not_smooth')
    clip_fraction = sum(bool(r.get('limited')) for r in rows)/max(1, len(rows))
    if clip_fraction > .05:
        reasons.append('excessive_command_limiting')
    temperatures=[r['temperature_c'] for r in rows if r.get('temperature_c') is not None]
    result['maximum_temperature_c']=max(temperatures) if temperatures else None
    result.update(passed=not reasons, reasons=sorted(set(reasons)), peak_depth_deg=peak,
                  lower_zone_ms=total*1000, timing_bracket_ms=uncertainty*1000,
                  entry_speed_rad_s=entry_speed, exit_speed_rad_s=exit_speed,
                  approach_ms=approach*1000, cycle_ms=cycle*1000,
                  upper_overshoot_deg=overshoot, zone_entries=len(entries),
                  peak_speed_rad_s=peak_speed, acceleration_p99=acc99, jerk_p99=jerk99,
                  command_limited_fraction=clip_fraction,
                  peak_estimated_torque=max(abs(r['estimated_torque']) for r in rows),
                  rms_estimated_torque=math.sqrt(sum(r['estimated_torque']**2 for r in rows)/len(rows)))
    return result


def summarize(trials, depth_match=.10, completed_campaigns=()):
    """Never mix parameters, real/sim data, scoring rules, or trial stages."""
    groups = {}
    for trial in trials:
        meta, s = trial['metadata'], trial['score']
        key = (meta['method'], meta['parameters_id'], meta['backend'],
               s['rules_id'], s['limits_id'], meta['stage'])
        groups.setdefault(key, []).append(trial)
    summaries = []
    for key, batch in groups.items():
        accepted = [x['score'] for x in batch if x['score']['passed']]
        item = dict(zip(('method','parameters_id','backend','rules_id','limits_id','stage'),key))
        item.update(attempts=len(batch), passed=len(accepted), failures=len(batch)-len(accepted))
        item['failure_reasons']=dict(Counter(r for t in batch for r in t['score']['reasons']))
        # Include failed strokes wherever the measurement exists. Missing traces
        # remain failures, not zero-duration strokes or silently dropped trials.
        for field in ('lower_zone_ms','peak_depth_deg','cycle_ms','upper_overshoot_deg',
                      'entry_speed_rad_s','timing_bracket_ms','peak_estimated_torque','rms_estimated_torque'):
            vals = [t['score'][field] for t in batch if field in t['score']]
            item[field+'_measured_attempts']=len(vals)
            if vals:
                item[field+'_median'] = float(np.median(vals))
                item[field+'_p95'] = float(np.percentile(vals,95))
        depths=[t['score']['peak_depth_deg'] for t in batch if 'peak_depth_deg' in t['score']]
        if depths:
            item['depth_std_deg'] = float(np.std(depths))
            item['depth_min_deg'],item['depth_max_deg']=min(depths),max(depths)
        temperatures=[t['score']['maximum_temperature_c'] for t in batch
                      if t['score'].get('maximum_temperature_c') is not None]
        item['maximum_temperature_c']=max(temperatures) if temperatures else None
        summaries.append(item)
    # A provisional ranking within each comparison group, NOT a global winner
    # from a lucky successful subset. Every trial in an eligible batch passed.
    comparisons = {}
    for s in summaries:
        key = (s['backend'],s['rules_id'],s['limits_id'],s['stage'])
        if s['passed'] and not s['failures']:
            comparisons.setdefault(key,[]).append(s)
    winners = []
    for key, candidates in comparisons.items():
        candidates.sort(key=lambda s:s['lower_zone_ms_median'])
        depths = [s['peak_depth_deg_median'] for s in candidates]
        matched = max(depths)-min(depths) <= depth_match
        top = candidates[0]
        uncertainty = max(s['timing_bracket_ms_median'] for s in candidates)
        tied = [s for s in candidates if s['lower_zone_ms_median']-top['lower_zone_ms_median'] <= uncertainty]
        endurance=next((g for g in summaries if g['method']==top['method']
                         and g['parameters_id']==top['parameters_id'] and g['backend']==top['backend']
                         and g['rules_id']==top['rules_id'] and g['limits_id']==top['limits_id']
                         and g['stage']=='endurance'),None)
        endurance_ok=bool(endurance and endurance['attempts']>=100 and not endurance['failures'])
        # A partial/stopped campaign must not crown the first method tested.
        completed=any(c.get('stage')=='all' and len(set(c.get('methods',[])))>1
                      and top['method'] in c['methods'] for c in completed_campaigns)
        winners.append(dict(comparison=list(key), depth_matched=matched, endurance_complete=endurance_ok,
                            campaign_complete=completed,
                            winner=(top['method'] if matched and len(tied)==1 and key[0]=='hardware'
                                    and key[3]=='validate' and all(s['attempts']>=100 for s in candidates)
                                    and endurance_ok and completed else None),
                            provisional_leader=(top['method'] if matched and len(tied)==1 else None),
                            candidates=[s['method'] for s in candidates],
                            tied=[s['method'] for s in tied],
                            note=('Depth matching required before declaring a winner' if not matched
                                  else 'Differences within measurement brackets are treated as ties'),
                            validation_complete=all(s['attempts']>=100 for s in candidates)))
    return dict(groups=summaries, comparisons=winners,
                disclaimer='Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.')
