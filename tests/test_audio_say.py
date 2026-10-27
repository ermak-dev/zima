#!/usr/bin/env python3
"""FEATURE: local text-to-speech that speaks on the robot through the existing helper.

SPEC: no ROS, no robot, no model download; Piper is imported only lazily, so
its absence is a configuration error naming the fix, never a silent fallback.
"""
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from robot_audio.playback import AudioError
from robot_audio.say import (YandexTTS, load_yandex_credentials, PiperTTS, speak)


def wav_data(seconds=.1, rate=22050):
    b = io.BytesIO()
    with wave.open(b, 'wb') as f:
        f.setnchannels(1); f.setsampwidth(2); f.setframerate(rate)
        f.writeframes(b'\x01\x02' * int(seconds * rate))
    return b.getvalue()


class FakePiperVoice:
    """Mimics piper.PiperVoice.synthesize: per-chunk audio objects, resident model."""
    def __init__(self):
        self.texts = []

    def synthesize(self, text):
        self.texts.append(text)
        class Chunk:
            audio_int16_bytes = b'\x00\x40' * 441  # 0.02 s at 22050 Hz
        return [Chunk()]


class PiperTTSTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        # A 22050 Hz voice config, like ru_RU voices shipped with piper.
        config = self.dir / 'voice.onnx.json'
        config.write_text('{"audio": {"sample_rate": 22050}}')
        self.model = self.dir / 'voice.onnx'
        self.model.write_bytes(b'weights')
        self.voice = FakePiperVoice()

    def make_engine(self):
        return PiperTTS(self.model, load_voice=lambda model: self.voice)

    def test_resident_model_is_loaded_once_and_used_per_chunk(self):
        engine = self.make_engine()
        engine.synth('Раз.')
        engine.synth('Два.')
        self.assertEqual(self.voice.texts, ['Раз.', 'Два.'])

    def test_sample_rate_comes_from_the_model_config(self):
        engine = self.make_engine()
        self.assertEqual(engine.sample_rate, 22050)

    def test_missing_model_names_the_path(self):
        with self.assertRaisesRegex(AudioError, str(self.dir)):
            PiperTTS(self.dir / 'absent.onnx', load_voice=lambda model: self.voice)

    def test_missing_piper_import_is_a_configuration_error(self):
        def no_piper(model):
            raise ImportError('No module named piper')
        with self.assertRaisesRegex(AudioError, 'pip install'):
            PiperTTS(self.model, load_voice=no_piper)

    def test_synth_returns_pcm_at_the_voice_rate(self):
        engine = self.make_engine()
        pcm = engine.synth('Привет.')
        self.assertEqual(len(pcm), 441 * 2)


class YandexTTSTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.requests = []
        self.responses = []  # list of (status, bytes)

    def opener(self, status=200, body=b'\x01\x02' * 800):
        parent = self
        def open_(request, timeout=None):
            parent.requests.append({'url': request.full_url,
                                    'headers': dict(request.header_items()),
                                    'data': request.data})
            item = parent.responses.pop(0) if parent.responses else (status, body)
            code, payload = item
            import urllib.error
            if code != 200:
                raise urllib.error.HTTPError(request.full_url, code, payload, {}, None)
            import io
            return io.BytesIO(payload)
        return open_

    def test_speaks_with_api_key_auth_and_lpcm16k(self):
        tts = YandexTTS(api_key='secret-key', opener=self.opener())
        pcm = tts.synth('Привет.')
        request = self.requests[0]
        self.assertIn('tts.api.cloud.yandex.net', request['url'])
        self.assertIn(b'Api-Key secret-key', request['headers']['Authorization'].encode()
                      if isinstance(request['headers'].get('Authorization'), str) else b'')
        self.assertEqual(tts.sample_rate, 16000)
        self.assertEqual(len(pcm), 1600)

    def test_folder_id_is_not_sent_with_api_key_auth(self):
        # The live API answers 400 "unexpected field: folder_id" under Api-Key
        # authorization: the folder is derived from the key itself.
        tts = YandexTTS(api_key='k', folder_id='b1g...', opener=self.opener())
        tts.synth('Тест.')
        self.assertNotIn(b'folder_id', self.requests[0]['data'])

    def test_voice_default_is_russian(self):
        tts = YandexTTS(api_key='k', opener=self.opener())
        tts.synth('Тест.')
        self.assertIn(b'voice=filipp', self.requests[0]['data'])

    def test_http_error_is_a_config_or_provider_error_not_audio(self):
        tts = YandexTTS(api_key='k', opener=self.opener())
        self.responses = [(403, b'UNAUTHENTICATED')]
        with self.assertRaisesRegex(AudioError, '403'):
            tts.synth('Тест.')

    def test_default_url_applies_when_not_overridden(self):
        # An empty override (config key unset) must mean the real endpoint,
        # not None -- that traceback reached the user once.
        for empty in ('', None):
            tts = YandexTTS(api_key='k', opener=self.opener(), url=empty)
            tts.synth('Тест.')
            self.assertIn('tts.api.cloud.yandex.net', self.requests[0]['url'])

    def test_provider_truncated_or_empty_reply_is_rejected(self):
        tts = YandexTTS(api_key='k', opener=self.opener())
        self.responses = [(200, b'')]
        with self.assertRaisesRegex(AudioError, 'empty'):
            tts.synth('Тест.')

    def test_credentials_loaded_only_from_the_named_env_file(self):
        env = self.dir / 'yandex.env'
        env.write_text('# comment\nYANDEX_API_KEY=key-one\nYANDEX_FOLDER=b1gfolder\n')
        key, folder = load_yandex_credentials(env)
        self.assertEqual((key, folder), ('key-one', 'b1gfolder'))
        missing = self.dir / 'absent.env'
        with self.assertRaisesRegex(AudioError, 'does not exist'):
            load_yandex_credentials(missing)

    def test_env_file_permissions_are_enforced(self):
        writable = self.dir / 'world.env'
        writable.write_text('YANDEX_API_KEY=k\n')
        writable.chmod(0o666)
        with self.assertRaisesRegex(AudioError, 'world-writable'):
            load_yandex_credentials(writable)
        linked = self.dir / 'linked.env'
        real = self.dir / 'real.env'
        real.write_text('YANDEX_API_KEY=k\n')
        linked.symlink_to(real)
        with self.assertRaisesRegex(AudioError, 'symlink'):
            load_yandex_credentials(linked)


class SpeakTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.played = []

    def engine(self, text_to_pcm):
        class E:
            sample_rate = 16000
            def synth(self, text):
                return text_to_pcm[text]
        return E()

    def transport(self, outcomes=None):
        parent = self
        class T:
            def play_wav(self, wav, volume):
                parent.played.append((wav, volume))
                return outcomes.pop(0) if outcomes else ('completed', '')
        return T()

    def test_speaks_one_chunk_through_the_transport(self):
        wav = wav_data(rate=16000)
        speak(self.engine({'Привет.': b'\0\0'}), self.transport(), 'Привет.',
              wav_builder=lambda pcm, rate: wav)
        self.assertEqual(len(self.played), 1)
        self.assertEqual(self.played[0][1], .35)

    def test_chunks_are_not_all_synthesized_before_the_first_plays(self):
        # Two chunks; the first WAV must reach the transport before the second
        # is synthesized -- the property the listener feels as latency.
        order = []
        eng = self.engine({'Раз.': b'\0\0', 'Два.': b'\0\0'})
        real_synth = eng.synth
        def synth(text):
            order.append('synth:' + text)
            return real_synth(text)
        eng.synth = synth
        class T:
            def play_wav(self, wav, volume):
                order.append('play')
                return ('completed', '')
        speak(eng, T(), 'Раз. Два.', first_max=4, rest_max=4,
              wav_builder=lambda pcm, rate: wav_data(rate=16000))
        self.assertEqual(order, ['synth:Раз.', 'play', 'synth:Два.', 'play'])

    def test_pcm_is_rewritten_to_16k_mono_before_transport(self):
        seen = {}
        class T:
            def play_wav(self, wav, volume):
                seen['wav'] = wav
                return ('completed', '')
        speak(self.engine({'x': b'\0\0'}), T(), 'x',
              wav_builder=lambda pcm, rate: wav_data(rate=rate))
        with wave.open(io.BytesIO(seen['wav'])) as audio:
            self.assertEqual((audio.getframerate(), audio.getnchannels(),
                              audio.getsampwidth()), (16000, 1, 2))

    def test_playback_failure_stops_speaking_and_reports(self):
        outcomes = [('busy', 'robot reports another playback')]
        engine = self.engine({})
        real_synth = engine.synth
        engine.synth = lambda text: real_synth(text) if text in ('a.', 'b.') else b'\0\0'
        seen_texts = []
        base = engine.synth
        def synth(text):
            seen_texts.append(text)
            return base(text)
        engine.synth = synth
        with self.assertRaises(AudioError) as caught:
            speak(engine, self.transport(outcomes), 'Кусок первый. Кусок второй.',
                  first_max=13, rest_max=13,
                  wav_builder=lambda pcm, rate: wav_data(rate=16000))
        self.assertEqual(caught.exception.code, 'busy')
        self.assertIn('another playback', str(caught.exception))
        self.assertEqual(len(self.played), 1)
        self.assertEqual(seen_texts, ['Кусок первый.'])

    def test_empty_audio_from_synth_is_an_error(self):
        class E:
            sample_rate = 16000
            def synth(self, text):
                return b''
        with self.assertRaisesRegex(AudioError, 'empty'):
            speak(E(), self.transport(), 'тишина', wav_builder=lambda pcm, rate: wav_data())


if __name__ == '__main__':
    unittest.main(verbosity=2)
