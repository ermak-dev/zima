"""Bounded local speech recognition and an explicit voice-session state machine."""
import array
from collections import deque
from concurrent.futures import Future
from dataclasses import dataclass
import math
from pathlib import Path
import re
import sys
import threading
import time
import uuid

from .wake import ConfigurationRequired, _finite, validate_chunk
from .phrases import strip_wake_prefix


def _samples(pcm):
    if not isinstance(pcm, bytes) or not pcm or len(pcm) % 2:
        raise ValueError('expected complete S16LE samples')
    values = array.array('h', pcm)
    if sys.byteorder != 'little':
        values.byteswap()
    return values


class RmsVad:
    """Energy-based VAD baseline; it does not distinguish speech from other noise."""
    def __init__(self, threshold_dbfs=-40):
        self.threshold_dbfs = _finite(threshold_dbfs, 'VAD threshold', -100, 0)

    def __call__(self, pcm):
        values = _samples(pcm)
        rms = math.sqrt(sum(value * value for value in values) / len(values))
        return 20 * math.log10(max(rms, 1e-12) / 32768) >= self.threshold_dbfs


class RecognizerBusy(RuntimeError):
    pass


class BoundedRecognizer:
    def __init__(self, recognize):
        self._recognize = recognize
        self._lock = threading.Lock()
        self._busy = False
        self._closed = False

    def submit(self, pcm):
        with self._lock:
            if self._busy or self._closed:
                raise RecognizerBusy('recognizer is busy or closed')
            self._busy = True
        future = Future()

        def run():
            try:
                if future.set_running_or_notify_cancel():
                    try:
                        future.set_result(self._recognize(pcm))
                    except Exception as error:
                        future.set_exception(error)
            finally:
                with self._lock:
                    self._busy = False
        threading.Thread(target=run, name='local-audio-asr', daemon=True).start()
        return future

    def close(self):
        with self._lock:
            self._closed = True


class FasterWhisperASR:
    def __init__(self, model_path, *, language='ru', device='cpu', compute_type='int8', cpu_threads=2):
        model_path = Path(model_path).expanduser()
        if not model_path.is_dir():
            raise ValueError('ASR requires an existing local model directory')
        if language not in ('ru', 'en'):
            raise ValueError('unsupported ASR language')
        if device not in ('cpu', 'cuda') or not isinstance(cpu_threads, int) or not 1 <= cpu_threads <= 64:
            raise ValueError('invalid ASR compute configuration')
        try:
            from faster_whisper import WhisperModel
            import numpy
        except ImportError as error:
            raise ConfigurationRequired('install the optional faster-whisper dependency') from error
        self._numpy = numpy
        self.language = language
        self._model = WhisperModel(str(model_path.resolve()), device=device,
                                   compute_type=compute_type, cpu_threads=cpu_threads,
                                   local_files_only=True)

    def __call__(self, pcm):
        if not pcm or len(pcm) % 2 or len(pcm) > 32 * 32000:
            raise ValueError('ASR input must be bounded mono 16 kHz PCM')
        audio = self._numpy.frombuffer(pcm, dtype='<i2').astype(self._numpy.float32) / 32768
        segments, _ = self._model.transcribe(audio, language=self.language, beam_size=1,
                                             vad_filter=False, condition_on_previous_text=False)
        text = ' '.join(segment.text.strip() for segment in segments).strip()
        if len(text) > 8192:
            raise ValueError('ASR result exceeds transcript limit')
        return text


@dataclass(frozen=True)
class Transcript:
    session_id: str
    stream_id: str
    text: str
    stamp_ns: int
    final: bool = True
    error: str = ''


class SpeechSession:
    def __init__(self, recognizer, *, vad=None, wake_phrase='', silence_s=.6,
                 wait_s=3, max_command_s=12, asr_timeout_s=15, echo_tail_s=.3,
                 clock=time.monotonic, on_transcript=None):
        self.recognizer = recognizer
        self.vad = vad if vad is not None else RmsVad()
        self.wake_phrase = wake_phrase
        self.silence_s = _finite(silence_s, 'silence timeout', .05, 5)
        self.wait_s = _finite(wait_s, 'speech wait', .1, 15)
        self.max_command_s = _finite(max_command_s, 'command timeout', .1, 30)
        self.asr_timeout_s = _finite(asr_timeout_s, 'ASR timeout', .1, 120)
        self.echo_tail_s = _finite(echo_tail_s, 'echo tail', 0, 5)
        self.clock = clock
        self.on_transcript = on_transcript
        self.state = 'waiting'
        self.last_error = ''
        self.transcripts = []
        self._seen = deque(maxlen=256)
        self._future = None
        self._session_id = None
        self._stream_id = None
        self._sequence = None
        self._stamp_ns = 0
        self._audio = bytearray()
        self._heard_speech = False
        self._started = 0.0
        self._last_speech = 0.0
        self._asr_started = 0.0
        self._playback_id = None
        self._hold_until = 0.0

    @property
    def session_id(self):
        return self._session_id

    @property
    def playback_active(self):
        return self._playback_id is not None

    def _now(self, now):
        value = self.clock() if now is None else now
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('invalid monotonic time')
        return value

    def _clear_command(self):
        if self._future is not None:
            self._future.cancel()
        self._future = None
        self._audio.clear()
        self._session_id = None
        self._stream_id = None
        self._sequence = None
        self._heard_speech = False

    def _fail(self, reason):
        self._clear_command()
        self.state = 'fault'
        self.last_error = reason

    def reset(self):
        self._clear_command()
        self.state = 'speaking' if self.playback_active else 'waiting'
        self.last_error = ''

    def cancel(self):
        self._clear_command()
        self.state = 'speaking' if self.playback_active else 'waiting'
        self.last_error = 'cancelled'

    def source_lost(self):
        self._fail('source_lost')

    def on_wake(self, event, *, now=None):
        now = self._now(now)
        if event.event_id in self._seen:
            return False
        if self.state != 'waiting' or self.playback_active or now < self._hold_until:
            return False
        if not event.event_id or not event.stream_id or len(event.pre_roll_pcm) % 2:
            raise ValueError('invalid wake event')
        self._seen.append(event.event_id)
        self._session_id = uuid.uuid4().hex
        self._stream_id, self._sequence = event.stream_id, event.sequence
        self._stamp_ns = event.stamp_ns
        self._audio = bytearray(event.pre_roll_pcm[-64000:])
        self._heard_speech = bool(self._audio and self.vad(bytes(self._audio)))
        self._started = self._last_speech = now
        self.last_error = ''
        self.state = 'listening'
        return True

    def feed(self, chunk, *, now=None):
        validate_chunk(chunk)
        now = self._now(now)
        if self.state not in ('listening', 'transcribing'):
            return
        if chunk.stream_id != self._stream_id or chunk.discontinuity:
            self._fail('audio_gap')
            return
        if chunk.sequence <= self._sequence:
            return
        if chunk.sequence != self._sequence + 1:
            self._fail('audio_gap')
            return
        self._sequence, self._stamp_ns = chunk.sequence, chunk.stamp_ns
        if self.state == 'listening':
            limit = int((self.max_command_s + 2) * 32000)
            if len(self._audio) + len(chunk.pcm) > limit:
                self._fail('audio_limit')
                return
            self._audio.extend(chunk.pcm)
            if self.vad(chunk.pcm):
                self._heard_speech = True
                self._last_speech = now
            self.tick(now=now)

    def _transcribe(self, now):
        try:
            self._future = self.recognizer.submit(bytes(self._audio))
        except RecognizerBusy:
            self._fail('asr_busy')
            return
        self._audio.clear()
        self._asr_started = now
        self.state = 'transcribing'

    def _strip_wake(self, text):
        return strip_wake_prefix(text, self.wake_phrase, allow_asr_substitution=True)

    def tick(self, *, now=None):
        now = self._now(now)
        if self.state == 'listening':
            if not self._heard_speech and now - self._started >= self.wait_s:
                self._fail('no_speech')
            elif self._heard_speech and (now - self._last_speech >= self.silence_s or
                                         now - self._started >= self.max_command_s):
                self._transcribe(now)
            return []
        if self.state != 'transcribing':
            return []
        if now - self._asr_started >= self.asr_timeout_s:
            self._fail('asr_timeout')
            return []
        if not self._future.done():
            return []
        try:
            text = self._future.result()
            if not isinstance(text, str) or len(text) > 8192:
                raise ValueError('invalid transcript')
            text = self._strip_wake(text)
        except Exception:
            self._fail('asr_failed')
            return []
        self._future = None
        if not text:
            self._fail('empty_command')
            return []
        transcript = Transcript(self._session_id, self._stream_id, text, self._stamp_ns)
        self.transcripts.append(transcript)
        if len(self.transcripts) > 32:
            del self.transcripts[:-32]
        self.state = 'processing'
        if self.on_transcript is not None:
            self.on_transcript(transcript)
        return [transcript]

    def response_finished(self, session_id):
        if session_id == self._session_id:
            self._clear_command()
            self.state = 'speaking' if self.playback_active else 'waiting'

    def playback_started(self, playback_id, *, now=None):
        self._now(now)
        if not isinstance(playback_id, str) or not playback_id:
            raise ValueError('missing playback id')
        if self._playback_id == playback_id:
            return
        self._clear_command()
        self._playback_id = playback_id
        self.state = 'speaking'

    def playback_finished(self, playback_id, *, now=None):
        now = self._now(now)
        if playback_id != self._playback_id:
            return
        self._playback_id = None
        self._hold_until = now + self.echo_tail_s
        self.state = 'waiting'

    def barge_in(self, event, cancel_playback, *, now=None):
        now = self._now(now)
        if not self.playback_active or event.event_id in self._seen:
            return False
        if cancel_playback(self._playback_id) is not True:
            return False
        self._playback_id = None
        self._hold_until = now
        self._clear_command()
        self.state = 'waiting'
        return self.on_wake(event, now=now)

    def close(self):
        self.cancel()
        self.recognizer.close()
