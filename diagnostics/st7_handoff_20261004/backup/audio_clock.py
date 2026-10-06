"""Map PortAudio ADC sample time to the host monotonic clock, not print time."""
import math


def block_start_monotonic(callback_monotonic, current_time, adc_time):
    values = (callback_monotonic, current_time, adc_time)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Invalid microphone clock')
    age = current_time - adc_time
    if current_time <= 0 or adc_time <= 0 or not -.01 <= age <= 1.:
        raise ValueError('Microphone did not supply a valid ADC capture clock')
    return callback_monotonic - age


def event_monotonic(block_end, samples_seen, sample_rate, event_seconds):
    return block_end - (samples_seen / sample_rate - event_seconds)
