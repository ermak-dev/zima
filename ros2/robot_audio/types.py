"""PCM boundary shared by capture, recognition and ROS adapters."""
from dataclasses import dataclass


@dataclass(frozen=True)
class AudioChunk:
    stream_id: str
    sequence: int
    stamp_ns: int
    pcm: bytes
    sample_rate: int = 16000
    channels: int = 1
    sample_format: str = 's16le'
    discontinuity: bool = False
    estimated_timestamp: bool = True

    def __post_init__(self):
        if not isinstance(self.stream_id, str) or not 1 <= len(self.stream_id) <= 128:
            raise ValueError('stream_id must contain 1..128 characters')
        for name in ('sequence', 'stamp_ns'):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value < 2**64:
                raise ValueError(f'{name} must be an unsigned 64-bit integer')
        if type(self.pcm) is not bytes:
            raise TypeError('PCM must be immutable bytes')
        if not self.pcm or len(self.pcm) % 2 or len(self.pcm) > 32000:
            raise ValueError('PCM must contain 1..16000 complete mono samples')
        if (self.sample_rate, self.channels, self.sample_format) != (16000, 1, 's16le'):
            raise ValueError('audio boundary requires 16000 Hz mono s16le')
        if type(self.discontinuity) is not bool or type(self.estimated_timestamp) is not bool:
            raise TypeError('audio flags must be boolean')

    @property
    def frames(self):
        return len(self.pcm) // 2

    @property
    def duration_s(self):
        return self.frames / self.sample_rate
