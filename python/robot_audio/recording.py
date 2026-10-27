"""Explicit short recordings consume the shared normalized source, never another HTTP stream."""
import math
import os
from pathlib import Path
import wave

from .types import AudioChunk


class PcmRecorder:
    def __init__(self, path, *, seconds):
        if type(seconds) not in (int, float) or not math.isfinite(seconds) or not .1 <= seconds <= 30:
            raise ValueError('recording duration must be .1..30 seconds')
        self.path = Path(path).expanduser()
        self.target_frames = round(seconds * 16000)
        self.frames = 0
        self.complete = False
        self._stream_id = self._sequence = None
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        self._inode = os.fstat(fd).st_ino
        self._file = os.fdopen(fd, 'wb')
        self._writer = wave.open(self._file, 'wb')
        self._writer.setnchannels(1)
        self._writer.setsampwidth(2)
        self._writer.setframerate(16000)

    def feed(self, chunk):
        if self.complete:
            return True
        if not isinstance(chunk, AudioChunk):
            self.close()
            raise ValueError('recording requires normalized PCM chunks')
        if self._stream_id is not None and (chunk.stream_id != self._stream_id or
                chunk.sequence != self._sequence + 1 or chunk.discontinuity):
            self.close()
            raise ValueError('audio stream changed or lost samples during recording')
        self._stream_id, self._sequence = chunk.stream_id, chunk.sequence
        count = min(chunk.frames, self.target_frames - self.frames)
        self._writer.writeframesraw(chunk.pcm[:count * 2])
        self.frames += count
        if self.frames == self.target_frames:
            self.complete = True
            self._writer.close()
            self._file.close()
        return self.complete

    def close(self):
        if self._file.closed:
            return
        self._writer.close()
        self._file.close()
        if not self.complete:
            try:
                if self.path.lstat().st_ino == self._inode:
                    self.path.unlink()
            except FileNotFoundError:
                pass
