"""SpeexDSP cancellation with explicit, bounded cross-device reference alignment."""
from collections import deque
import ctypes
import ctypes.util
from dataclasses import dataclass
import threading

from .wake import ConfigurationRequired, _finite, validate_chunk


class SpeexEchoCanceller:
    def __init__(self, *, frame_size=320, sample_rate=16000, filter_length=3200, library=None):
        if frame_size != 320 or sample_rate != 16000:
            raise ValueError('AEC requires 20 ms mono 16 kHz frames')
        if not isinstance(filter_length, int) or not frame_size <= filter_length <= sample_rate * 2:
            raise ValueError('invalid echo-filter length')
        name = library or ctypes.util.find_library('speexdsp')
        if not name:
            raise ConfigurationRequired('libspeexdsp is unavailable')
        try:
            self._lib = ctypes.CDLL(name)
        except OSError as error:
            raise ConfigurationRequired('libspeexdsp is unavailable') from error
        self.frame_size = frame_size
        self.sample_rate = sample_rate
        self._array_type = ctypes.c_int16 * frame_size
        pointer = ctypes.POINTER(ctypes.c_int16)
        self._lib.speex_echo_state_init.argtypes = [ctypes.c_int, ctypes.c_int]
        self._lib.speex_echo_state_init.restype = ctypes.c_void_p
        self._lib.speex_echo_state_destroy.argtypes = [ctypes.c_void_p]
        self._lib.speex_echo_state_destroy.restype = None
        self._lib.speex_echo_state_reset.argtypes = [ctypes.c_void_p]
        self._lib.speex_echo_state_reset.restype = None
        self._lib.speex_echo_ctl.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        self._lib.speex_echo_ctl.restype = ctypes.c_int
        self._lib.speex_echo_cancellation.argtypes = [ctypes.c_void_p, pointer, pointer, pointer]
        self._lib.speex_echo_cancellation.restype = None
        self._lock = threading.RLock()
        self._state = self._lib.speex_echo_state_init(frame_size, filter_length)
        if not self._state:
            raise RuntimeError('cannot allocate echo canceller')
        rate = ctypes.c_int(sample_rate)
        if self._lib.speex_echo_ctl(self._state, 24, ctypes.byref(rate)) != 0:
            self.close()
            raise RuntimeError('cannot configure echo sample rate')

    def process(self, microphone, reference):
        with self._lock:
            if not self._state:
                raise RuntimeError('echo canceller is closed')
            if not isinstance(microphone, bytes) or not isinstance(reference, bytes):
                raise ValueError('AEC input must be S16LE bytes')
            if len(microphone) != self.frame_size * 2 or len(reference) != self.frame_size * 2:
                raise ValueError('AEC input must be exactly one 20 ms frame')
            import sys
            if sys.byteorder != 'little':
                raise RuntimeError('native Speex adapter requires a little-endian host')
            mic = self._array_type.from_buffer_copy(microphone)
            ref = self._array_type.from_buffer_copy(reference)
            output = self._array_type()
            self._lib.speex_echo_cancellation(self._state, mic, ref, output)
            return bytes(output)

    def reset(self):
        with self._lock:
            if not self._state:
                raise RuntimeError('echo canceller is closed')
            self._lib.speex_echo_state_reset(self._state)

    def close(self):
        if not hasattr(self, '_lock'):
            return
        with self._lock:
            state = getattr(self, '_state', None)
            if state:
                self._state = None
                self._lib.speex_echo_state_destroy(state)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()


@dataclass(frozen=True)
class EchoResult:
    pcm: bytes
    mode: str
    reason: str
    allow_wake: bool
    allow_barge_in: bool = False
    playback_id: str = ''


class EchoController:
    def __init__(self, engine, *, delay_ms=0, max_drift_ms=20,
                 calibrated=False, max_reference_s=2):
        self.engine = engine
        self.delay_ns = int(_finite(delay_ms, 'acoustic delay', 0, 2000) * 1e6)
        self.max_drift_ns = int(_finite(max_drift_ms, 'reference drift', 0, 100) * 1e6)
        self.calibrated = calibrated is True
        self._max_frames = int(_finite(max_reference_s, 'reference duration', .02, 5) / .02)
        self._references = deque(maxlen=self._max_frames)
        self._reference_stream = None
        self._reference_sequence = None
        self._playback_id = None
        self._capture_stream = None
        self._capture_sequence = None

    @property
    def reference_frames(self):
        return len(self._references)

    def reset(self):
        self._references.clear()
        self._reference_stream = None
        self._reference_sequence = None
        self._playback_id = None
        self._capture_stream = None
        self._capture_sequence = None
        self.engine.reset()

    def set_calibration(self, *, delay_ms, verified=False):
        delay = int(_finite(delay_ms, 'acoustic delay', 0, 2000) * 1e6)
        self.reset()
        self.delay_ns = delay
        self.calibrated = verified is True

    def push_reference(self, chunk, *, playback_id, actual_playback=False):
        validate_chunk(chunk)
        if len(chunk.pcm) != 640:
            raise ValueError('reference must contain exactly 20 ms')
        if actual_playback is not True or not isinstance(playback_id, str) or not playback_id:
            raise ValueError('only acknowledged playback can be an echo reference')
        if self._reference_stream == chunk.stream_id and self._reference_sequence is not None:
            if chunk.sequence <= self._reference_sequence:
                return
        changed = self._playback_id is not None and (
            playback_id != self._playback_id or chunk.stream_id != self._reference_stream or
            chunk.sequence != self._reference_sequence + 1)
        if changed or chunk.discontinuity:
            self._references.clear()
            self.engine.reset()
        self._playback_id = playback_id
        self._reference_stream, self._reference_sequence = chunk.stream_id, chunk.sequence
        self._references.append((chunk.stamp_ns, chunk.pcm, playback_id))

    def _fallback(self, chunk, reason, playback_active):
        self.engine.reset()
        return EchoResult(chunk.pcm, 'half_duplex', reason, not playback_active)

    def process_capture(self, chunk, *, playback_active):
        validate_chunk(chunk)
        if len(chunk.pcm) != 640:
            raise ValueError('capture must contain exactly 20 ms')
        gap = self._capture_stream is not None and (
            self._capture_stream != chunk.stream_id or chunk.sequence != self._capture_sequence + 1)
        self._capture_stream, self._capture_sequence = chunk.stream_id, chunk.sequence
        if gap or chunk.discontinuity:
            self._references.clear()
            return self._fallback(chunk, 'audio_gap', playback_active)
        if not playback_active:
            return EchoResult(chunk.pcm, 'idle', '', True)
        if not self.calibrated:
            return self._fallback(chunk, 'calibration_required', True)
        if not self._references:
            return self._fallback(chunk, 'reference_missing', True)
        expected_ns = chunk.stamp_ns - self.delay_ns
        reference = min(self._references, key=lambda item: abs(item[0] - expected_ns))
        if abs(reference[0] - expected_ns) > self.max_drift_ns:
            return self._fallback(chunk, 'reference_drift', True)
        while self._references and self._references[0][0] <= reference[0]:
            self._references.popleft()
        clean = self.engine.process(chunk.pcm, reference[1])
        return EchoResult(clean, 'full_duplex', '', True, True, reference[2])

    def playback_finished(self, playback_id):
        if playback_id == self._playback_id:
            self._references.clear()
            self._reference_stream = None
            self._reference_sequence = None
            self._playback_id = None
            self.engine.reset()

    def close(self):
        self._references.clear()
        self.engine.close()
