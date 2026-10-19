"""Bounded local PCM measurements and controlled acoustic probe assessment."""
import array
import math
import os
from pathlib import Path
import stat
import sys
import wave


def _positive(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def _samples(pcm, sample_rate, channels):
    if not isinstance(sample_rate, int) or isinstance(sample_rate, bool) or not 8000 <= sample_rate <= 96000:
        raise ValueError('sample_rate must be an integer between 8000 and 96000')
    if channels not in (1, 2) or isinstance(channels, bool):
        raise ValueError('channels must be 1 or 2')
    if not isinstance(pcm, bytes) or not pcm or len(pcm) % (2 * channels):
        raise ValueError('PCM must contain complete nonempty s16le frames')
    if len(pcm) > 32 * 1024 * 1024:
        raise ValueError('PCM exceeds measurement limit')
    samples = array.array('h', pcm)
    if sys.byteorder != 'little':
        samples.byteswap()
    return samples


def analyze_pcm(pcm, sample_rate=16000, channels=1):
    samples = _samples(pcm, sample_rate, channels)
    mean = sum(samples) / len(samples)
    rms = math.sqrt(sum((sample - mean) ** 2 for sample in samples) / len(samples))
    clipped = sum(sample <= -32768 or sample >= 32767 for sample in samples)
    return {
        'sample_rate': sample_rate,
        'channels': channels,
        'sample_format': 's16le',
        'frames': len(samples) // channels,
        'duration_s': len(samples) / channels / sample_rate,
        'dc_offset': mean,
        'ac_rms': rms,
        'ac_dbfs': 20 * math.log10(rms / 32768) if rms else None,
        'peak': max(map(abs, samples)),
        'clipped_samples': clipped,
        'clipping_fraction': clipped / len(samples),
        'microphone_verified': False,
        'speech_verified': False,
    }


def assess_probe(baseline_pcm, probe_pcm, *, sample_rate=16000, tone_hz=880,
                 min_snr_db=10, min_signal_dbfs=-50, min_duration_s=.25,
                 max_clipping_fraction=.001):
    _positive(tone_hz, 'tone_hz')
    _positive(min_snr_db, 'min_snr_db')
    _positive(min_duration_s, 'min_duration_s')
    if tone_hz >= sample_rate / 2:
        raise ValueError('tone_hz must be below Nyquist frequency')
    if not isinstance(min_signal_dbfs, (int, float)) or not math.isfinite(min_signal_dbfs) or not -120 <= min_signal_dbfs <= 0:
        raise ValueError('min_signal_dbfs must be between -120 and 0')
    if not isinstance(max_clipping_fraction, (int, float)) or not math.isfinite(max_clipping_fraction) or not 0 <= max_clipping_fraction < 1:
        raise ValueError('invalid clipping fraction')
    baseline = analyze_pcm(baseline_pcm, sample_rate)
    probe = analyze_pcm(probe_pcm, sample_rate)
    samples = _samples(probe_pcm, sample_rate, 1)
    omega = 2 * math.pi * tone_hz / sample_rate
    cosine = sum((sample - probe['dc_offset']) * math.cos(omega * i) for i, sample in enumerate(samples))
    sine = sum((sample - probe['dc_offset']) * math.sin(omega * i) for i, sample in enumerate(samples))
    amplitude = 2 * math.hypot(cosine, sine) / len(samples)
    signal_rms = math.sqrt(max(0, probe['ac_rms'] ** 2 - baseline['ac_rms'] ** 2))
    snr = 20 * math.log10(max(signal_rms, .5) / max(baseline['ac_rms'], .5))
    reasons = []
    if min(baseline['duration_s'], probe['duration_s']) < min_duration_s:
        reasons.append('insufficient_duration')
    if probe['ac_dbfs'] is None or probe['ac_dbfs'] < min_signal_dbfs:
        reasons.append('insufficient_signal')
    if snr < min_snr_db:
        reasons.append('insufficient_snr')
    if amplitude < max(100, probe['ac_rms'] * .5):
        reasons.append('reference_not_detected')
    if probe['clipping_fraction'] > max_clipping_fraction:
        reasons.append('clipping')
    return {
        'baseline': baseline,
        'probe': probe,
        'tone_hz': tone_hz,
        'reference_amplitude': amplitude,
        'snr_db': snr,
        'reasons': reasons,
        'microphone_verified': not reasons,
        'speech_verified': False,
        'verification_scope': 'controlled_tone_response_only',
    }


def analyze_wav(path, *, max_duration_s=30):
    _positive(max_duration_s, 'max_duration_s')
    if max_duration_s > 300:
        raise ValueError('measurement duration limit exceeds 300 seconds')
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0)
    try:
        fd = os.open(Path(path), flags)
    except OSError:
        raise ValueError('WAV must be an accessible regular file, not a symlink') from None
    with os.fdopen(fd, 'rb') as file:
        info = os.fstat(file.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 32 * 1024 * 1024:
            raise ValueError('WAV is not a bounded regular file')
        try:
            with wave.open(file, 'rb') as source:
                channels = source.getnchannels()
                rate = source.getframerate()
                frames = source.getnframes()
                if source.getsampwidth() != 2 or source.getcomptype() != 'NONE':
                    raise ValueError('WAV must contain uncompressed 16-bit PCM')
                if not rate or frames / rate > max_duration_s:
                    raise ValueError('WAV exceeds measurement duration')
                pcm = source.readframes(frames)
                if len(pcm) != frames * channels * 2:
                    raise ValueError('WAV contains truncated PCM')
        except (wave.Error, EOFError):
            raise ValueError('invalid WAV file') from None
    return analyze_pcm(pcm, rate, channels)


def write_reference_wav(path, *, duration_s=.75, frequency=880, amplitude=900 / 32768,
                        sample_rate=16000):
    _positive(duration_s, 'duration_s')
    _positive(frequency, 'frequency')
    _positive(amplitude, 'amplitude')
    if duration_s > 2 or duration_s < .1:
        raise ValueError('reference duration must be between .1 and 2 seconds')
    if sample_rate != 16000 or frequency >= sample_rate / 2 or amplitude > .25:
        raise ValueError('reference must be 16 kHz with bounded frequency and amplitude <= .25')
    count = round(duration_s * sample_rate)
    fade = min(round(sample_rate * .02), count // 2)
    samples = array.array('h')
    for i in range(count):
        envelope = min(1, i / fade, (count - 1 - i) / fade)
        samples.append(round(32768 * amplitude * envelope * math.sin(2 * math.pi * frequency * i / sample_rate)))
    if sys.byteorder != 'little':
        samples.byteswap()
    fd = os.open(Path(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'wb') as file:
        with wave.open(file, 'wb') as output:
            output.setnchannels(1)
            output.setsampwidth(2)
            output.setframerate(sample_rate)
            output.writeframes(samples.tobytes())
    return {'frequency': frequency, 'duration_s': count / sample_rate,
            'sample_rate': sample_rate, 'amplitude': amplitude}
