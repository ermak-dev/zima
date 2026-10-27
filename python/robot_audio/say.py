"""Local text-to-speech played through the robot's existing playback helper.

No cloud provider: Piper synthesizes chunk by chunk, every chunk
becomes a 16 kHz mono PCM16 WAV, and playback goes through the same SSH
helper transport as the audio service. Piper is imported lazily, so its
absence is a configuration error naming the fix, never a silent fallback.
"""
import json
import os
import stat
import subprocess
import urllib.error
import urllib.parse
import urllib.request
import wave
import io
from pathlib import Path
import threading

from .chunking import split_for_speech
from .playback import AudioError

# The helper accepts at most 30 s of audio; stay below it so a long chunk
# still fits after any rounding.
MAX_SEGMENT_S = 28
OUTPUT_RATE = 16000
PIPER_VOICE_DIR = Path.home() / '.local/share/piper-voices'


def find_voice(language='ru'):
    """Locate an installed piper voice; nothing is ever downloaded here.

    Raises with an actionable message rather than falling back silently: a
    missing voice must look like a missing voice, not like a dead speaker.
    """
    matches = sorted(PIPER_VOICE_DIR.glob(f'{language}_*.onnx'))
    if matches:
        return matches[0]
    raise AudioError(
        f'no piper voice matching {language}_*.onnx in {PIPER_VOICE_DIR}. '
        'A voice is fetched manually (huggingface.co/rhasspy/piper-voices) and '
        'is never downloaded automatically; install piper-tts too: pip install piper-tts'
    )


YANDEX_TTS_URL = 'https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize'


class YandexTTS:
    """Yandex SpeechKit synthesis; raw LPCM 16 kHz mono is the helper's format.

    Only the text being spoken is sent to the provider. Credentials are an
    API key plus optional folder id, read from a 0600 env file; they are never
    embedded in the URL, logged or retried automatically.
    """

    def __init__(self, api_key, folder_id='', voice='filipp', opener=None,
                 timeout_s=15, max_chars=2000, url=YANDEX_TTS_URL):
        if not api_key or len(api_key) > 8192:
            raise AudioError('configuration_required')
        self.api_key, self.folder_id = api_key, folder_id
        self.voice, self.timeout_s, self.max_chars = voice, timeout_s, max_chars
        self.sample_rate = 16000
        self._url = url or YANDEX_TTS_URL
        self._open = opener or urllib.request.build_opener(
            urllib.request.ProxyHandler({})).open

    def synth(self, text):
        if not text or len(text) > self.max_chars:
            raise AudioError('invalid_text_length')
        body = {'text': text, 'voice': self.voice, 'format': 'lpcm',
                'sampleRateHertz': '16000'}
        # With Api-Key authorization the folder is derived from the key; the
        # live API answers 400 "unexpected field: folder_id" if it is sent.
        request = urllib.request.Request(
            self._url, data=urllib.parse.urlencode(body).encode(),
            headers={'Authorization': f'Api-Key {self.api_key}'})
        try:
            with self._open(request, timeout=self.timeout_s) as response:
                pcm = response.read(4_000_001)
        except urllib.error.HTTPError as exc:
            raise AudioError(f'speechkit http {exc.code}') from None
        except (urllib.error.URLError, OSError):
            raise AudioError('speechkit_unreachable') from None
        if not pcm or len(pcm) % 2 or len(pcm) > 4_000_000:
            raise AudioError('tts produced empty or oversized audio')
        return pcm


def load_yandex_credentials(env_path):
    """Read YANDEX_API_KEY/YANDEX_FOLDER from a private 0600 file; nothing else.

    Deliberately not a generic dotenv loader: this file belongs to another
    project, so only the two named variables are read and the key is never
    printed in errors.
    """
    path = Path(env_path).expanduser()
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd, 'r', encoding='utf-8') as handle:
            metadata = os.fstat(handle.fileno())
            # Another project's file: we cannot demand 0600 of it, but we can
            # refuse one that anyone else could have tampered with.
            if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.getuid()
                    or metadata.st_mode & 0o002):
                raise AudioError('yandex env file must be a regular file, '
                                 'owned by this user and not world-writable')
            values = {}
            for line in handle.read(65536).splitlines():
                name, sep, value = line.strip().partition('=')
                if sep and name in ('YANDEX_API_KEY', 'YANDEX_FOLDER'):
                    values[name] = value.strip().strip('\'"')
    except FileNotFoundError:
        raise AudioError('yandex env file does not exist') from None
    except OSError:
        raise AudioError('yandex env file is unreadable or unsafe '
                         '(missing, or a symlink)') from None
    if not values.get('YANDEX_API_KEY'):
        raise AudioError('YANDEX_API_KEY is not set in the yandex env file')
    return values['YANDEX_API_KEY'], values.get('YANDEX_FOLDER', '')


class PiperTTS:
    """Resident Piper voice; the model is loaded once and reused per chunk."""

    def __init__(self, model_path=None, load_voice=None, language='ru'):
        model = Path(model_path).expanduser() if model_path else find_voice(language)
        config_path = Path(f'{model}.json')
        if not model.is_file() or not config_path.is_file():
            raise AudioError(f'piper voice not found: {model} (+ .json); '
                             'fetch a voice first, none is downloaded automatically')
        try:
            config = json.loads(config_path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            raise AudioError('unreadable piper voice config') from None
        self.sample_rate = int(config.get('audio', {}).get('sample_rate', 22050))
        if load_voice is None:
            def load_voice(path):
                try:
                    from piper import PiperVoice
                except ImportError as exc:
                    raise AudioError('piper is not installed in this Python '
                                     'environment; install piper-tts first') from exc
                return PiperVoice.load(str(path))
        try:
            self._voice = load_voice(model)
        except ImportError as exc:
            raise AudioError(f'piper is not importable: {exc}; '
                             'pip install piper-tts in this Python environment') from exc

    def synth(self, text):
        return b''.join(chunk.audio_int16_bytes
                        for chunk in self._voice.synthesize(text))


def resample_pcm(pcm, rate):
    """Convert raw mono PCM16 to the helper's 16 kHz rate (ffmpeg, bounded)."""
    if rate == OUTPUT_RATE:
        return pcm
    try:
        return subprocess.run(
            ['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
             '-f', 's16le', '-ar', str(rate), '-ac', '1', '-i', 'pipe:0',
             '-ar', str(OUTPUT_RATE), '-ac', '1', '-f', 's16le', 'pipe:1'],
            input=pcm, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            timeout=30, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        raise AudioError('audio_conversion_failed') from None


def build_wav(pcm, rate=OUTPUT_RATE):
    """Wrap raw mono PCM16 into a WAV container the helper accepts."""
    result = io.BytesIO()
    with wave.open(result, 'wb') as audio:
        audio.setnchannels(1); audio.setsampwidth(2); audio.setframerate(rate)
        audio.writeframes(pcm)
    return result.getvalue()


def _segments(pcm, max_s=MAX_SEGMENT_S):
    limit = int(max_s * OUTPUT_RATE) * 2
    for pos in range(0, len(pcm), limit):
        yield pcm[pos:pos + limit]


def speak(tts, transport, text, *, volume=.35, first_max=48, rest_max=160,
          wav_builder=None):
    """Speak text chunk by chunk; each chunk plays before the next synthesizes.

    Any non-completed playback outcome stops the utterance: continuing to talk
    after an unknown output state could collide with another speaker on the
    robot, which shares one playback lock.
    """
    if wav_builder is None:
        wav_builder = build_wav
    for chunk in split_for_speech(text, first_max=first_max, rest_max=rest_max):
        pcm = tts.synth(chunk)
        if not pcm:
            raise AudioError('tts produced empty audio')
        for segment in _segments(resample_pcm(pcm, tts.sample_rate)):
            outcome, message = transport.play_wav(wav_builder(segment, OUTPUT_RATE), volume)
            if outcome != 'completed':
                raise AudioError(outcome, message)


class HelperTransport:
    """play_wav adapter over the existing asset store and SSH helper transport."""

    def __init__(self, store, transport):
        self.store, self.transport = store, transport

    def play_wav(self, wav, volume):
        asset = self.store.put(wav)
        result = self.transport.play(asset, volume, threading.Event())
        return result.outcome, result.message
