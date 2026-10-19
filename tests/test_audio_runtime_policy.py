#!/usr/bin/env python3
"""FEATURE: Duplex and provider policy remain enforced across asynchronous workers."""
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.playback import AudioError, AssetStore, PlaybackQueue, TransportResult
from robot_audio.runtime import AudioRuntime
from robot_audio.types import AudioChunk
from robot_audio.wake import WakeDetector
from test_audio_runtime import Source, wav_bytes, wait_for


class RuntimePolicyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.events = {key: [] for key in ('chunk', 'input_state', 'wake', 'playback', 'speech_state', 'diagnostic')}
        self.stop_output = threading.Event()
        parent = self
        class Transport:
            def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
                if on_started: on_started()
                if parent.block_output:
                    cancel_event.wait(3)
                    time.sleep(parent.cancel_delay)
                    return TransportResult('cancelled')
                return TransportResult('completed')
        self.block_output = False
        self.cancel_delay = 0
        self.playback = PlaybackQueue(AssetStore(self.tmp.name), Transport())
        self.asset = self.playback.store.put(wav_bytes())

    def runtime(self, config, **kwargs):
        runtime = AudioRuntime(config, playback=self.playback, **kwargs)
        runtime.set_callbacks(**{key: values.append for key, values in self.events.items()})
        runtime.start(); self.addCleanup(runtime.close)
        return runtime

    def request(self):
        return {'text': 'Тест', 'voice': '', 'volume': .2, 'priority': 0, 'max_queue_wait': 3}

    def tts_config(self):
        return {'output': {'enabled': True}, 'tts': {'enabled': True, 'allow_external': True,
                'model': 'fake/model', 'voice': 'voice'}}

    def test_only_one_external_tts_request_runs_and_waiting_can_cancel(self):
        entered, release, second_call = threading.Event(), threading.Event(), threading.Event()
        class TTS:
            calls = 0
            def synthesize(self, text, cancel_event=None):
                self.calls += 1
                if self.calls > 1: second_call.set()
                entered.set(); release.wait(2)
                return wav_bytes()
        runtime = self.runtime(self.tts_config(), tts=TTS())
        cancel_one, cancel_two = threading.Event(), threading.Event()
        result_one, result_two = [], []
        one = threading.Thread(target=lambda: result_one.append(runtime.execute('say', self.request(),
            'one', cancel_one, lambda value: None)))
        two = threading.Thread(target=lambda: result_two.append(runtime.execute('say', self.request(),
            'two', cancel_two, lambda value: None)))
        one.start(); self.assertTrue(entered.wait(1)); two.start()
        duplicated = second_call.wait(.2)
        waiting_phase = self.events['playback'][-1]['state']
        cancel_two.set(); release.set(); one.join(3); two.join(3)
        self.assertEqual(waiting_phase, 'synthesizing')
        self.assertFalse(duplicated, 'queued speech must not launch a second paid request')
        self.assertEqual(result_one[0]['error_code'], 0)
        self.assertEqual(result_two[0]['error_code'], 3)

    def test_provider_failure_does_not_leave_output_synthesizing(self):
        class TTS:
            def synthesize(self, text, cancel_event=None): raise AudioError('provider_unavailable')
        runtime = self.runtime(self.tts_config(), tts=TTS())
        result = runtime.execute('say', self.request(), 'failed-tts', threading.Event(), lambda value: None)
        self.assertEqual(result['error_code'], 4)
        self.assertEqual(self.events['playback'][-1]['state'], 'failed')
        self.assertFalse(self.events['playback'][-1]['speaking'])

    def test_half_duplex_policy_wins_even_when_echo_backend_allows_wake(self):
        class Backend:
            def predict(self, pcm): return .9
            def reset(self): pass
        class Echo:
            def process_capture(self, chunk, playback_active):
                return SimpleNamespace(pcm=chunk.pcm, allow_wake=True, allow_barge_in=True)
            def reset(self): pass
            def close(self): pass
            def playback_finished(self, playback_id): pass
        self.block_output = True
        runtime = self.runtime({'input': {'enabled': True, 'microphone_verified': True},
            'output': {'enabled': True}}, source=Source(), detector=WakeDetector(Backend(), phrase='test'), echo=Echo())
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute('play', {
            'asset_id': self.asset.asset_id, 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'speaking', cancel, lambda value: None)))
        thread.start()
        try:
            self.assertTrue(wait_for(lambda: any(value['speaking'] for value in self.events['playback'])))
            for i in range(4):
                runtime.process_chunk(AudioChunk('fixture', i, 1_000_000_000 + i * 20_000_000,
                                                  b'\x10\x27' * 320))
        finally:
            cancel.set(); thread.join(3)
        self.assertEqual(self.events['wake'], [])

    def test_missing_echo_reference_is_reported_as_half_duplex_fallback(self):
        class Echo:
            def process_capture(self, chunk, playback_active):
                return SimpleNamespace(pcm=chunk.pcm, allow_wake=False, allow_barge_in=False,
                                       mode='half_duplex', reason='reference_missing')
            def reset(self): pass
            def close(self): pass
            def playback_finished(self, playback_id): pass
        self.block_output = True
        runtime = self.runtime({'input': {'enabled': True, 'microphone_verified': True},
            'output': {'enabled': True}, 'speech': {'mode': 'full_duplex'},
            'echo': {'enabled': True, 'verified': True}}, source=Source(), echo=Echo())
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute('play', {
            'asset_id': self.asset.asset_id, 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'echo-fallback', cancel, lambda value: None)))
        thread.start()
        try:
            self.assertTrue(wait_for(lambda: any(value['speaking'] for value in self.events['playback'])))
            runtime.process_chunk(AudioChunk('fixture', 0, time.time_ns(), b'\x10\x27' * 320))
            self.assertEqual(self.events['speech_state'][-1]['duplex_mode'], 'half_duplex')
            self.assertTrue(any(value.get('state') == 'reference_missing' for value in self.events['diagnostic']))
        finally:
            cancel.set(); thread.join(3)

    def test_full_duplex_preserves_command_while_remote_cancel_is_pending(self):
        from robot_audio.speech import SpeechSession, BoundedRecognizer, RmsVad
        class Backend:
            def predict(self, pcm): return .9
            def reset(self): pass
        class Echo:
            def process_capture(self, chunk, playback_active):
                return SimpleNamespace(pcm=chunk.pcm, allow_wake=True, allow_barge_in=True,
                                       mode='full_duplex', reason='')
            def reset(self): pass
            def close(self): pass
            def playback_finished(self, playback_id): pass
        self.block_output = True
        self.cancel_delay = .12
        source = Source()
        session = SpeechSession(BoundedRecognizer(lambda pcm: 'эй робот прерви ответ'),
                                wake_phrase='эй робот', vad=RmsVad(-40), silence_s=.05)
        transcripts = []
        runtime = self.runtime({'input': {'enabled': True, 'microphone_verified': True},
            'output': {'enabled': True}, 'speech': {'mode': 'full_duplex'},
            'echo': {'enabled': True, 'verified': True}}, source=source, session=session,
            detector=WakeDetector(Backend(), phrase='эй робот'), echo=Echo())
        runtime._callbacks['transcript'] = transcripts.append
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute('play', {
            'asset_id': self.asset.asset_id, 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'barge-in', cancel, lambda value: None)))
        thread.start()
        try:
            self.assertTrue(wait_for(lambda: any(value['speaking'] for value in self.events['playback'])))
            epoch = time.time_ns()
            for i in range(18):
                source.queue.put(AudioChunk('fixture', i, epoch + i * 20_000_000,
                    (b'\x10\x27' if i < 7 else b'\x00\x00') * 320))
                time.sleep(.02)
            self.assertTrue(wait_for(lambda: transcripts))
            self.assertEqual(transcripts[-1]['text'], 'прерви ответ')
            thread.join(2)
            self.assertEqual(result[0]['error_code'], 3)
        finally:
            cancel.set(); thread.join(3)

    def test_mute_refuses_tts_without_stopping_microphone(self):
        from robot_audio.policy import OutputPolicy
        calls = []
        class TTS:
            def synthesize(self, text, cancel_event=None):
                calls.append(text); return wav_bytes()
        config = self.tts_config(); config['input'] = {'enabled': True}
        source = Source()
        runtime = self.runtime(config, tts=TTS(), source=source, output_policy=OutputPolicy(muted=True))
        result = runtime.execute('say', self.request(), 'muted', threading.Event(), lambda value: None)
        self.assertEqual(result['error_code'], 2)
        self.assertEqual(calls, [])
        self.assertEqual(source.started, 1)
        self.assertEqual(source.stopped, 0)

    def test_muting_active_playback_waits_for_cancel_ack(self):
        self.block_output = True
        runtime = self.runtime({'output': {'enabled': True}})
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute('play', {
            'asset_id': self.asset.asset_id, 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'mute-active', cancel, lambda value: None)))
        thread.start()
        try:
            self.assertTrue(wait_for(lambda: any(value['speaking'] for value in self.events['playback'])))
            self.assertTrue(runtime.set_muted(True)['success'])
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result[0]['error_code'], 3)
            self.assertFalse(self.events['playback'][-1]['speaking'])
        finally:
            cancel.set(); thread.join(3)

    def test_quiet_hours_transition_cancels_current_output(self):
        from datetime import datetime, timezone
        from robot_audio.policy import OutputPolicy
        now = [datetime(2026, 10, 18, 21, 0, tzinfo=timezone.utc)]
        policy = OutputPolicy(quiet_hours=['22:00-08:00'], clock=lambda: now[0])
        self.block_output = True
        runtime = self.runtime({'output': {'enabled': True}}, output_policy=policy)
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute('play', {
            'asset_id': self.asset.asset_id, 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'quiet-transition', cancel, lambda value: None)))
        thread.start()
        try:
            self.assertTrue(wait_for(lambda: any(value['speaking'] for value in self.events['playback'])))
            now[0] = datetime(2026, 10, 18, 22, 0, tzinfo=timezone.utc)
            thread.join(1)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result[0]['error_code'], 3)
        finally:
            cancel.set(); thread.join(3)

    def test_model_exception_does_not_kill_pcm_delivery_silently(self):
        class Backend:
            def predict(self, pcm): raise RuntimeError('inference failed')
            def reset(self): pass
        source = Source()
        runtime = self.runtime({'input': {'enabled': True, 'microphone_verified': True}},
            source=source, detector=WakeDetector(Backend(), phrase='test'))
        for i in range(5):
            source.queue.put(AudioChunk('fixture', i, 1_000_000_000 + i * 20_000_000,
                                        b'\x10\x27' * 320))
        self.assertTrue(wait_for(lambda: len(self.events['chunk']) >= 5), 'PCM capture must survive model failure')
        self.assertTrue(any(value.get('state') == 'processing_failed' for value in self.events['diagnostic']))


if __name__ == '__main__': unittest.main()
