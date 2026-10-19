"""CPU wake-word detection; model artifacts are supplied explicitly and locally."""
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import uuid


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


def wake_artifact_hashes(manifest):
    hashes = {}
    for key in ('model_file', 'melspec_model_file', 'embedding_model_file'):
        digest = hashlib.sha256()
        with Path(manifest[key]).open('rb') as handle:
            for block in iter(lambda: handle.read(65536), b''):
                digest.update(block)
        hashes[key] = digest.hexdigest()
    return hashes


def load_wake_manifest(path, *, require_russian=False, allow_experimental=False):
    path = Path(path).expanduser().resolve()
    try:
        if path.stat().st_size > 65536:
            raise ConfigurationRequired('wake manifest too large')
        data = json.loads(path.read_text())
        if data.get('schema_version') != 1:
            raise ConfigurationRequired('unsupported wake manifest schema')
        for key in ('phrase', 'language', 'license'):
            if not isinstance(data.get(key), str) or not data[key].strip():
                raise ConfigurationRequired(f'explicit {key} is required')
        if len(data['phrase']) > 120:
            raise ConfigurationRequired('wake phrase too long')
        for key in ('model_file', 'melspec_model_file', 'embedding_model_file'):
            value = data.get(key)
            if not isinstance(value, str) or '://' in value:
                raise ConfigurationRequired(f'local {key} is required')
            model = (path.parent / value).resolve()
            if not model.is_file() or not 0 < model.stat().st_size <= 256 * 1024 * 1024:
                raise ConfigurationRequired(f'missing or oversized local {key}')
            data[key] = str(model)
        if require_russian:
            if data['language'] != 'ru' or (data.get('verified') is not True and allow_experimental is not True):
                raise ConfigurationRequired('verified Russian wake model is required')
            data['experimental'] = data.get('verified') is not True
            report_path = data.get('evaluation_report')
            if not isinstance(report_path, str):
                raise ConfigurationRequired('held-out evaluation report is required')
            report_file = path.parent / report_path
            if report_file.stat().st_size > 2 * 1024 * 1024:
                raise ConfigurationRequired('wake evaluation report is too large')
            report = json.loads(report_file.read_text())
            if (report.get('accepted') is not True or report.get('split') != 'test' or
                    report.get('phrase') != data['phrase'] or report.get('language') != data['language']):
                raise ConfigurationRequired('matching held-out wake evaluation is required')
            if 'pronunciation' in data:
                from .phrases import PhraseSpec
                PhraseSpec(data['phrase'], data['pronunciation'], data.get('negative_pronunciations', ()))
                if (report.get('pronunciation') != data['pronunciation'] or
                        report.get('negative_pronunciations', []) != data.get('negative_pronunciations', [])):
                    raise ConfigurationRequired('wake pronunciation differs from the evaluated phrase')
            if report.get('artifacts_sha256') != wake_artifact_hashes(data):
                raise ConfigurationRequired('wake artifacts differ from the evaluated model')
            data['evaluation_threshold'] = _finite(report.get('threshold'), 'evaluated threshold', 0, 1)
        data['manifest_path'] = str(path)
        return data
    except ConfigurationRequired:
        raise
    except (OSError, ValueError, TypeError, AttributeError) as error:
        raise ConfigurationRequired('invalid or unavailable wake model manifest') from error


class OpenWakeWordBackend:
    def __init__(self, manifest):
        for key in ('model_file', 'melspec_model_file', 'embedding_model_file'):
            if not Path(manifest.get(key, '')).is_file():
                raise ConfigurationRequired('explicit local wake artifacts are required')
        try:
            from openwakeword.model import Model
            import numpy
        except ImportError as error:
            raise ConfigurationRequired('install the optional openwakeword/onnxruntime dependencies') from error
        self._numpy = numpy
        self._key = manifest.get('model_name', Path(manifest['model_file']).stem)
        self._model = Model(wakeword_models=[manifest['model_file']],
                            inference_framework='onnx', vad_threshold=0,
                            melspec_model_path=manifest['melspec_model_file'],
                            embedding_model_path=manifest['embedding_model_file'])

    def predict(self, pcm):
        if len(pcm) != 2560:
            raise ValueError('wake inference requires exactly 80 ms')
        scores = self._model.predict(self._numpy.frombuffer(pcm, dtype='<i2').copy())
        if self._key not in scores:
            raise ConfigurationRequired('wake model output name does not match the manifest')
        return float(scores[self._key])

    def reset(self):
        self._model.reset()


@dataclass(frozen=True)
class WakeEvent:
    event_id: str
    phrase: str
    model: str
    score: float
    stream_id: str
    sequence: int
    stamp_ns: int
    pre_roll_pcm: bytes


class WakeDetector:
    frame_bytes = 2560

    def __init__(self, backend, *, phrase, model='local', threshold=.5,
                 vad_threshold=0, debounce_s=1.5, pre_roll_s=2):
        if not isinstance(phrase, str) or not phrase.strip():
            raise ConfigurationRequired('choose an explicit wake phrase')
        self.backend = backend
        self.phrase = phrase.strip()
        self.model = model
        self.threshold = _finite(threshold, 'threshold', 0, 1)
        self.vad_threshold = _finite(vad_threshold, 'vad threshold', 0, 1)
        self.debounce_s = _finite(debounce_s, 'debounce', 0, 60)
        self._max_pre_roll = int(_finite(pre_roll_s, 'pre-roll', .08, 2) * 32000)
        self._pending = bytearray()
        self._pre_roll = bytearray()
        self._stream_id = None
        self._sequence = None
        self._last_event_ns = None
        self.enabled = True

    @property
    def pre_roll_pcm(self):
        return bytes(self._pre_roll)

    def reset(self):
        self._pending.clear()
        self._pre_roll.clear()
        self._stream_id = None
        self._sequence = None
        self._last_event_ns = None
        self.backend.reset()

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        if self.enabled != enabled:
            self.reset()
            self.enabled = enabled

    def feed(self, chunk, *, speech_probability=1.0):
        validate_chunk(chunk)
        speech_probability = _finite(speech_probability, 'speech probability', 0, 1)
        if not self.enabled:
            return []
        if self._stream_id == chunk.stream_id and self._sequence is not None and chunk.sequence <= self._sequence:
            return []
        gap = self._stream_id is not None and (
            self._stream_id != chunk.stream_id or chunk.sequence != self._sequence + 1)
        if gap or chunk.discontinuity:
            self.reset()
        self._stream_id, self._sequence = chunk.stream_id, chunk.sequence
        self._pending.extend(chunk.pcm)
        events = []
        consumed = 0
        prefix_size = len(self._pending) - len(chunk.pcm)
        while len(self._pending) >= self.frame_bytes:
            frame = bytes(self._pending[:self.frame_bytes])
            del self._pending[:self.frame_bytes]
            self._pre_roll.extend(frame)
            if len(self._pre_roll) > self._max_pre_roll:
                del self._pre_roll[:-self._max_pre_roll]
            consumed += self.frame_bytes
            frame_end_ns = chunk.stamp_ns + int((consumed - prefix_size) / 32000 * 1e9)
            score = self.backend.predict(frame)
            if not math.isfinite(score) or not 0 <= score <= 1:
                continue
            if score < self.threshold or speech_probability < self.vad_threshold:
                continue
            if self._last_event_ns is not None and frame_end_ns - self._last_event_ns < self.debounce_s * 1e9:
                continue
            self._last_event_ns = frame_end_ns
            events.append(WakeEvent(uuid.uuid4().hex, self.phrase, self.model, score,
                                    chunk.stream_id, chunk.sequence, frame_end_ns,
                                    bytes(self._pre_roll)))
        return events
