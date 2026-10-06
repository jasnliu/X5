"""Offline acoustic verification; reads the captured TONOR WAV, never hardware."""
import json
from pathlib import Path
import wave
import numpy as np
from scipy.signal import butter, sosfilt

ROOT=Path(__file__).resolve().parents[2]
D=Path(__file__).resolve().parent
R=ROOT/'playback_results/swing_verification/20261005T031425-a625f9f5/result.json'
r=json.loads(R.read_text())
with wave.open(str(D/'physical_run_6.wav')) as w:
    rate=w.getframerate(); x=np.frombuffer(w.readframes(w.getnframes()),'<i2').astype(float)/32768
start=float((D/'audio_run6_start.txt').read_text())
y=sosfilt(butter(4,1200,fs=rate,btype='highpass',output='sos'),x)
hop=80
power=(y[:len(y)//hop*hop].reshape(-1,hop)**2).mean(axis=1)
times=(np.arange(len(power))+.5)*hop/rate
# Positive energy changes over 15 ms locate attack, not the long ringing tail.
flux=np.maximum(0,power-np.roll(power,3))
rows=[]
for b in r['bottoms']:
    expected=b['at']-start
    valid=np.flatnonzero((times>=expected-.13)&(times<=expected+.025))
    i=int(valid[np.argmax(flux[valid])]); onset=times[i]-.0025
    before=power[max(0,i-10):max(0,i-3)]
    after=power[i:i+4]
    ratio=float(max(after)/max(float(np.median(before)),1e-10))
    rows.append(dict(strike=b['count'],encoder_bottom_s=expected,
        audio_attack_s=onset,offset_ms=(onset-expected)*1000,
        high_frequency_power_rise_db=10*np.log10(ratio),
        high_frequency_rms=float(np.sqrt(max(after)))))
# Compare each metallic decay with the first ST7-confirmed swing hit.
# This is supporting evidence, not a replacement classifier or a live threshold.
spectra=[]
window_samples=round(.05*rate)
frequencies=np.fft.rfftfreq(window_samples,1/rate)
for row in rows:
    begin=round((row['audio_attack_s']+.015)*rate)
    fft_power=abs(np.fft.rfft(x[begin:begin+window_samples]*np.hanning(window_samples)))**2
    bands=np.array([np.sqrt(fft_power[(frequencies>=lo)&(frequencies<lo+500)].sum())
                    for lo in range(1500,7500,500)])
    spectra.append(bands/max(float(np.linalg.norm(bands)),1e-12))
for row,spectrum in zip(rows,spectra):
    row['spectral_cosine_to_first_confirmed_hit']=float(np.dot(spectrum,spectra[0]))
report=dict(source_result=str(R.relative_to(ROOT)),raw_audio=str((D/'physical_run_6.wav').relative_to(ROOT)),
    method='TONOR 16 kHz PCM; high-pass 1200 Hz; positive 15 ms energy change within -130/+25 ms of each encoder bottom. No detector threshold or motor limit changes.',
    note='Recorder startup time is approximate; repeated per-stroke offsets are reported, not claimed as ADC calibration.',
    fault_free_duration_s=r['duration_s'],encoder_strokes=r['encoder_strikes'],faults=r['faults'],
    attacks=rows,minimum_rise_db=min(z['high_frequency_power_rise_db'] for z in rows),
    spectral_method='Cosine similarity of 500 Hz spectral amplitude bands from 1500 to 7500 Hz, Hann-windowed 15-65 ms after each attack; reference is first ST7-confirmed swing hit.',
    minimum_spectral_cosine=min(z['spectral_cosine_to_first_confirmed_hit'] for z in rows),
    mean_offset_ms=float(np.mean([z['offset_ms'] for z in rows])),
    center=r['center'],relaxed_verified=r['relaxed_verified'])
report['acoustic_attacks_confirmed']=int(sum(z['high_frequency_power_rise_db']>=6 for z in rows))
report['success']=(report['acoustic_attacks_confirmed']==14 and r['duration_s']>=5
    and not r['faults'] and r['relaxed_verified'] and r['center'] is not None)
(D/'physical_acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
