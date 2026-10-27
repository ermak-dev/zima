"""Shared PCM validation, independent of recognition backends."""
import math


class ConfigurationRequired(ValueError):
    pass


def _finite(value, name, low=0, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if value < low or (high is not None and value > high):
        raise ValueError(f'{name} outside supported range')
    return float(value)


def validate_chunk(chunk):
    if (chunk.sample_rate, chunk.channels, chunk.sample_format) != (16000, 1, 's16le'):
        raise ValueError('expected mono 16 kHz S16LE')
    if not isinstance(chunk.pcm, bytes) or not chunk.pcm or len(chunk.pcm) % 2:
        raise ValueError('expected nonempty complete S16LE samples')
    if not isinstance(chunk.stream_id, str) or not chunk.stream_id:
        raise ValueError('missing stream id')
    if not isinstance(chunk.sequence, int) or chunk.sequence < 0:
        raise ValueError('invalid sequence')
    if not isinstance(chunk.stamp_ns, int) or chunk.stamp_ns < 0:
        raise ValueError('invalid timestamp')
