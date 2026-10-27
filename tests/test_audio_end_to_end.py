#!/usr/bin/env python3
"""FEATURE: Manual utterance -> Russian command -> Say -> TTS -> playback, natively.

SPEC: Real runtime/codecs, injected model/provider/device; no live or paid I/O.
STATUS: Replaces the ROS-action end-to-end test; wake activation no longer exists.
"""
import base64
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from test_audio_runtime import Source, wav_bytes, wait_for


class EndToEndAudioTests(unittest.TestCase):
    def test_voice_command_and_reply_are_independent_of_motion(self):
        from robot_audio.playback import AssetStore, PlaybackQueue, TransportResult
        from robot_audio.runtime import AudioRuntime
        from robot_audio.speech import BoundedRecognizer, SpeechSession, RmsVad
        from robot_audio.tts import OpenRouterTTS
        from robot_audio.types import AudioChunk

        transport_calls, provider_calls = [], []
        class Transport:
            def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
                transport_calls.append(asset.asset_id)
                if on_started: on_started()
                if on_reference: on_reference(asset.pcm, asset.sample_rate, time.monotonic())
                cancel_event.wait(asset.duration_s)
                return TransportResult('cancelled' if cancel_event.is_set() else 'completed')
        class Response(io.BytesIO):
            status = 200
        def provider(request, timeout):
            body = json.loads(request.data)
            provider_calls.append(body)
            text = json.loads(body['messages'][-1]['content'])['text']
            payload = {'choices': [{'delta': {'audio': {'data': base64.b64encode(wav_bytes()).decode(),
                                                       'transcript': text}}}]}
            return Response(('data: ' + json.dumps(payload) + '\n\ndata: [DONE]\n\n').encode())

        with tempfile.TemporaryDirectory() as directory:
            source = Source()
            playback = PlaybackQueue(AssetStore(directory), Transport())
            session = SpeechSession(BoundedRecognizer(lambda pcm: 'осмотри коридор'),
                                    vad=RmsVad(-40), silence_s=.05)
            config = {'input': {'enabled': True}, 'output': {'enabled': True},
                      'tts': {'enabled': True, 'allow_external': True,
                              'model': 'offline/model', 'voice': 'offline'}}
            runtime = AudioRuntime(config, source=source, playback=playback, session=session,
                tts=OpenRouterTTS(model='offline/model', voice='offline', api_key='offline-key',
                                  opener=provider, require_verbatim=True))
            transcripts, states = [], []
            runtime.set_callbacks(transcript=transcripts.append, playback=states.append)
            runtime.start()
            try:
                self.assertTrue(runtime.begin_utterance()['success'])
                epoch = time.time_ns()
                for i in range(16):
                    source.queue.put(AudioChunk('synthetic', i, epoch + i * 20_000_000,
                        (b'\x10\x27' if i < 4 else b'\x00\x00') * 320))
                    time.sleep(.02)
                self.assertTrue(wait_for(lambda: transcripts))
                self.assertEqual(transcripts[0]['text'], 'осмотри коридор')
                self.assertEqual(transcripts[0]['status'], 'final')
                self.assertTrue(runtime.response_finished(transcripts[0]['session_id'])['success'])
                result = runtime.execute('say', {'text': 'Начинаю осмотр', 'voice': 'offline',
                    'volume': .25, 'priority': 0, 'max_queue_wait': 3},
                    'reply', threading.Event(), lambda value: None)
                self.assertEqual(result['error_code'], 0)
                self.assertEqual(len(provider_calls), 1)
                self.assertEqual(json.loads(provider_calls[0]['messages'][-1]['content'])['text'], 'Начинаю осмотр')
                self.assertEqual(len(transport_calls), 1)
                self.assertTrue(any(state['speaking'] for state in states))
                self.assertTrue(wait_for(lambda: states and not states[-1]['speaking']))
            finally:
                runtime.close()


if __name__ == '__main__': unittest.main()
