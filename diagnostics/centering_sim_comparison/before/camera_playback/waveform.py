"""Bounded capture-time waveform and onset markers, with no motor dependencies."""
from collections import deque
from dataclasses import dataclass
import math
import tkinter as tk

import numpy as np


HISTORY_SECONDS = 10.
COLORS = {'ride': '#dc2626', 'hihat': '#2563eb'}
LABELS = {'ride': 'Ride cymbal', 'hihat': 'Hi-hat'}
MAX_MARKERS = 256


def capture_start(callback_monotonic, current_time, adc_time):
    """Same ADC-to-monotonic mapping used by ST7, not callback/receipt time."""
    values = (callback_monotonic, current_time, adc_time)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Invalid microphone clock')
    age = current_time - adc_time
    if current_time <= 0 or adc_time <= 0 or not -.01 <= age <= 1.:
        raise ValueError('Microphone did not supply a valid ADC capture clock')
    return callback_monotonic - age


@dataclass(frozen=True)
class Marker:
    instrument: str
    event_at: float
    detected_at: float


class WaveformTimeline:
    def __init__(self, history=HISTORY_SECONDS):
        if not math.isfinite(history) or not 2 <= history <= 30:
            raise ValueError('Waveform history must be 2 through 30 seconds')
        self.history = float(history)
        # Each capture block is 20 ms; the extra second accommodates jitter.
        self.blocks = deque(maxlen=math.ceil((history+1)/.02))
        self.markers = deque(maxlen=MAX_MARKERS)
        self.latest_sample = None
        self.counts = dict(ride=0, hihat=0)
        self.delays = dict(ride=None, hihat=None)

    def add_audio(self, started_at, samples, sample_rate=16000):
        samples = np.asarray(samples, dtype=np.float32).reshape(-1)
        if (not math.isfinite(started_at) or sample_rate <= 0
                or len(samples) == 0 or not np.isfinite(samples).all()):
            raise ValueError('Invalid waveform capture block')
        # Retain a 1 ms min/max envelope, not every raw sample. This preserves
        # transient peaks while bounding both memory and redraw work.
        stride = max(1, int(sample_rate/1000))
        indices = np.arange(0, len(samples), stride)
        low, high = np.minimum.reduceat(samples, indices), np.maximum.reduceat(samples, indices)
        times = started_at + (indices + np.minimum(stride, len(samples)-indices)/2)/sample_rate
        self.blocks.append((times, low, high))
        self.latest_sample = started_at + len(samples)/sample_rate

    def add_hit(self, message, now):
        instrument = message.get('instrument')
        event, detected = message.get('event_at'), message.get('detected_at')
        if (message.get('kind') != 'hit' or instrument not in COLORS
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                           and math.isfinite(v) for v in (event, detected))
                or event > detected + .05 or detected > now + .05
                or event < now-self.history):
            return False
        marker = Marker(instrument, float(event), float(detected))
        if any(m.instrument == instrument and m.event_at == event for m in self.markers):
            return False
        self.markers.append(marker)
        self.counts[instrument] += 1
        self.delays[instrument] = max(0., detected-event)
        return True

    def prune(self, now):
        cutoff = now-self.history
        while self.blocks and self.blocks[0][0][-1] < cutoff:
            self.blocks.popleft()
        # Different detector delays mean arrival order is not onset order.
        self.markers = deque((m for m in self.markers if m.event_at >= cutoff),
                             maxlen=MAX_MARKERS)

    def fraction(self, timestamp, now):
        return (timestamp-(now-self.history))/self.history

    def envelope(self, now, width):
        """Per-screen-column peaks; missing columns remain gaps, not silence."""
        self.prune(now)
        low = np.full(width, np.inf)
        high = np.full(width, -np.inf)
        for times, minimum, maximum in self.blocks:
            columns = np.floor((times-(now-self.history))*width/self.history).astype(int)
            mask = (columns >= 0) & (columns < width)
            np.minimum.at(low, columns[mask], minimum[mask])
            np.maximum.at(high, columns[mask], maximum[mask])
        return low, high


class WaveformWindow:
    """Separate visual window. Its close button never stops the beat program."""
    def __init__(self, root, timeline):
        self.root, self.timeline = root, timeline
        root.title('TONOR — live sound timeline (visual only)')
        root.geometry('1120x500')
        root.minsize(700, 360)
        root.configure(bg='#f8fafc')
        tk.Label(root, text=f'TONOR MICROPHONE  ·  LAST {timeline.history:g} SECONDS',
                 bg='#f8fafc', fg='#0f172a', font=('Sans', 13, 'bold'),
                 anchor='w', padx=18, pady=10).pack(fill='x')
        legend = tk.Frame(root, bg='#f8fafc')
        legend.pack(fill='x', padx=18)
        for instrument in ('ride', 'hihat'):
            tk.Label(legend, text='│ '+LABELS[instrument], fg=COLORS[instrument],
                     bg='#f8fafc', font=('Sans', 11, 'bold')).pack(side='left', padx=(0, 24))
        tk.Label(legend, text='Markers arrive after detection, at the original sound time.',
                 bg='#f8fafc', fg='#475569', font=('Sans', 10)).pack(side='left')
        self.canvas = tk.Canvas(root, bg='white', highlightthickness=1,
                                highlightbackground='#cbd5e1')
        self.canvas.pack(fill='both', expand=True, padx=18, pady=(12, 8))
        self.status = tk.StringVar(value='Starting TONOR capture…  |  Visual only; no audio saved')
        tk.Label(root, textvariable=self.status, bg='#f8fafc', fg='#475569',
                 font=('Sans', 10), anchor='w', padx=18, pady=8).pack(fill='x')
        self.drawn_markers = []  # Read-only inspection aid for UI regression tests.

    def render(self, now, detail=''):
        c, timeline = self.canvas, self.timeline
        w, h = max(100, c.winfo_width()), max(100, c.winfo_height())
        left, right, top, bottom = 62., w-20., 94., h-35.
        width = max(1, int(right-left))
        low, high = timeline.envelope(now, width)
        valid = np.isfinite(low)
        peak = max(float(np.max(np.abs(low[valid]), initial=0)),
                   float(np.max(np.abs(high[valid]), initial=0)))
        scale = max(.005, peak*1.1)  # Visual gain only; never an audio threshold.
        middle, gain = (top+bottom)/2, (bottom-top)/2/scale
        c.delete('all')
        for part in range(6):
            x = left + (right-left)*part/5
            c.create_line(x, top, x, bottom, fill='#e2e8f0')
            label = 'now' if part == 5 else f'−{timeline.history*(1-part/5):g}s'
            c.create_text(x, bottom+18, text=label, fill='#64748b', font=('Sans', 9))
        for value in (-scale, 0., scale):
            y = middle-value*gain
            c.create_line(left, y, right, y, fill='#cbd5e1' if value == 0 else '#edf2f7')
            c.create_text(left-7, y, text=f'{value:+.3f}', anchor='e', fill='#64748b', font=('Sans', 8))
        # Separate contiguous runs so a dropped audio block never gets joined
        # to the next block as if it were a continuous waveform.
        columns = np.flatnonzero(valid)
        for group in np.split(columns, np.flatnonzero(np.diff(columns) > 1)+1):
            if not len(group):
                continue
            coordinates = np.column_stack((left+group, middle-low[group]*gain,
                                            left+group, middle-high[group]*gain)).ravel().tolist()
            c.create_line(*coordinates, fill='#334155', width=1, tags='waveform')
        self.drawn_markers = []
        # Draw blue dashed over red if onsets coincide; both remain visible.
        for instrument in ('ride', 'hihat'):
            for m in timeline.markers:
                if m.instrument != instrument:
                    continue
                fraction = timeline.fraction(m.event_at, now)
                if not 0 <= fraction <= 1:
                    continue
                x = left + fraction*(right-left)
                color, row = COLORS[instrument], (22 if instrument == 'ride' else 66)
                c.create_line(x, top-5, x, bottom, fill=color, width=2,
                              dash=() if instrument == 'ride' else (5, 3), tags=instrument)
                # Compact vertical labels stay readable at 200 ms swung-hit
                # spacing; full horizontal names remain in the fixed legend.
                c.create_text(x, row, text='Ride' if instrument == 'ride' else 'Hi-hat',
                              fill=color, angle=90, font=('Sans', 9, 'bold'), tags=instrument)
                self.drawn_markers.append(dict(instrument=instrument, event_at=m.event_at,
                                               detected_at=m.detected_at, x=x, color=color))
        delays = '  ·  '.join(f'{LABELS[k]} delay {timeline.delays[k]:.2f}s'
                             for k in COLORS if timeline.delays[k] is not None)
        self.status.set('  |  '.join(s for s in (detail, delays, 'Visual only · no audio saved') if s))
