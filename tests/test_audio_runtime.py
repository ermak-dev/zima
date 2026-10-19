#!/usr/bin/env python3
"""FEATURE: Complete host audio wiring without devices, model downloads or API spend.

SPEC: Capture/wake/sessions/playback are bounded, cancellable and independently gated.
"""
import io
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
import wave

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))


def wait_for(test, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if test():
            return True
        time.sleep(.01)
    return bool(test())


def wav_bytes():
    output = io.BytesIO()
    with wave.open(output, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
        w.writeframes(b'\x10\x00' * 1600)
    return output.getvalue()


class Source:
    def __init__(self):
        self.queue = queue.Queue()
        self.started = self.stopped = 0
        self.mode = 'streaming'

    def start(self):
        self.started += 1

    def stop(self):
        self.stopped += 1

    def get_chunk(self, timeout=None):
        return self.queue.get(timeout=timeout)

    def state(self):
        return {'state': self.mode, 'stream_id': 'test', 'last_frame_age_s': 0,
                'signal_dbfs': -30, 'clipping_fraction': 0, 'error': None}


class AudioRuntimeTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.runtime import AudioRuntime
        except ImportError:
            self.fail('host audio runtime is not implemented')
        self.Runtime = AudioRuntime
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.events = {key: [] for key in ('chunk', 'input_state', 'wake', 'transcript',
                                           'playback', 'reference', 'speech_state', 'diagnostic')}

    def runtime(self, config=None, **kwargs):
        runtime = self.Runtime(config or {}, **kwargs)
        runtime.set_callbacks(**{key: values.append for key, values in self.events.items()})
        runtime.start()
        self.addCleanup(runtime.close)
        return runtime

    def playback(self, outcome='completed', block=False):
        from robot_audio.playback import AssetStore, PlaybackQueue, TransportResult
        parent = self
        class Transport:
            def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
                parent.transport_calls += 1
                if on_started: on_started()
                if on_reference: on_reference(asset.pcm, asset.sample_rate, time.monotonic())
                if block:
                    cancel_event.wait(3)
                    return TransportResult('cancelled' if cancel_event.is_set() else 'unknown')
                return TransportResult(outcome)
        self.transport_calls = 0
        store = AssetStore(self.tmp.name)
        asset = store.put(wav_bytes())
        return PlaybackQueue(store, Transport()), asset.asset_id

    def test_default_runtime_has_no_live_side_effects(self):
        runtime = self.runtime()
        self.assertEqual(self.events['input_state'][-1]['state'], 'disabled')
        result = runtime.execute('play', {'asset_id': 'beep', 'volume': .2,
                                         'priority': 0, 'max_queue_wait': 2},
                                 'request', threading.Event(), lambda value: None)
        self.assertEqual(result['error_code'], 2)
        self.assertEqual(list(Path(self.tmp.name).iterdir()), [])

    def test_one_capture_and_privacy_stop_restart(self):
        from robot_audio.types import AudioChunk
        source = Source()
        runtime = self.runtime({'input': {'enabled': True}}, source=source)
        source.queue.put(AudioChunk('test', 0, 1_000_000_000, b'\x00\x00' * 320))
        self.assertTrue(wait_for(lambda: self.events['chunk']))
        self.assertEqual(source.started, 1)
        self.assertFalse(self.events['input_state'][-1]['microphone_verified'])
        self.assertTrue(runtime.set_listening(False)['success'])
        self.assertEqual(source.stopped, 1)
        self.assertEqual(self.events['input_state'][-1]['state'], 'disabled')
        self.assertTrue(runtime.set_listening(True)['success'])
        self.assertEqual(source.started, 2)

    def test_playback_cancel_reaches_remote_worker_and_not_just_action(self):
        playback, asset = self.playback(block=True)
        runtime = self.runtime({'output': {'enabled': True}}, playback=playback)
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute(
            'play', {'asset_id': asset, 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'cancel-me', cancel, lambda value: None)))
        thread.start()
        self.assertTrue(wait_for(lambda: any(value['speaking'] for value in self.events['playback'])))
        cancel.set(); thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result[0]['error_code'], 3)
        self.assertFalse(self.events['playback'][-1]['speaking'])
        self.assertEqual(self.transport_calls, 1)

    def test_tts_wait_is_not_playback_and_duplicate_id_does_not_spend_twice(self):
        playback, _ = self.playback()
        class TTS:
            calls = 0
            def synthesize(self, text, cancel_event=None):
                self.calls += 1
                return wav_bytes()
        tts = TTS()
        runtime = self.runtime({'output': {'enabled': True}, 'tts': {
            'enabled': True, 'allow_external': True, 'model': 'fake/model', 'voice': 'voice'}},
            playback=playback, tts=tts)
        request = {'text': 'Проверка', 'voice': '', 'volume': .2, 'priority': 0, 'max_queue_wait': 2}
        one = runtime.execute('say', request, 'same-id', threading.Event(), lambda value: None)
        two = runtime.execute('say', request, 'same-id', threading.Event(), lambda value: None)
        self.assertEqual(one['error_code'], 0)
        self.assertEqual(two, one)
        self.assertEqual(tts.calls, 1)
        waiting = [event for event in self.events['playback'] if event['state'] == 'synthesizing']
        self.assertTrue(waiting)
        self.assertTrue(all(not event['speaking'] for event in waiting))

    def test_tts_cancellation_never_starts_playback(self):
        playback, _ = self.playback()
        entered = threading.Event()
        class TTS:
            def synthesize(self, text, cancel_event=None):
                from robot_audio.playback import AudioError
                entered.set(); cancel_event.wait(2)
                raise AudioError('cancelled')
        runtime = self.runtime({'output': {'enabled': True}, 'tts': {
            'enabled': True, 'allow_external': True, 'model': 'fake/model', 'voice': 'voice'}},
            playback=playback, tts=TTS())
        cancel = threading.Event(); result = []
        thread = threading.Thread(target=lambda: result.append(runtime.execute(
            'say', {'text': 'Проверка', 'voice': '', 'volume': .2, 'priority': 0, 'max_queue_wait': 2},
            'cancel-tts', cancel, lambda value: None)))
        thread.start(); self.assertTrue(entered.wait(2)); cancel.set(); thread.join(3)
        self.assertEqual(result[0]['error_code'], 3)
        self.assertEqual(self.transport_calls, 0)
        self.assertFalse(any(event['speaking'] for event in self.events['playback']))

    def test_unknown_outcome_is_not_a_success_or_an_immediate_idle_state(self):
        playback, asset = self.playback(outcome='unknown')
        runtime = self.runtime({'output': {'enabled': True}}, playback=playback)
        result = runtime.execute('play', {'asset_id': asset, 'volume': .2,
                                         'priority': 0, 'max_queue_wait': 2},
                                 'uncertain', threading.Event(), lambda value: None)
        self.assertEqual(result['error_code'], 5)
        self.assertEqual(self.events['playback'][-1]['state'], 'unknown')
        self.assertTrue(self.events['playback'][-1]['speaking'])
        self.assertEqual(self.transport_calls, 1)

    def test_wake_to_local_transcript_and_matching_completion(self):
        from robot_audio.types import AudioChunk
        from robot_audio.wake import WakeDetector
        from robot_audio.speech import BoundedRecognizer, SpeechSession, RmsVad
        class Backend:
            def predict(self, pcm): return .9
            def reset(self): pass
        source = Source()
        detector = WakeDetector(Backend(), phrase='эй робот')
        session = SpeechSession(BoundedRecognizer(lambda pcm: 'эй робот проверь коридор'),
                                vad=RmsVad(-40), wake_phrase='эй робот', silence_s=.05)
        runtime = self.runtime({'input': {'enabled': True, 'microphone_verified': True}},
                               source=source, detector=detector, session=session)
        for i in range(4):
            source.queue.put(AudioChunk('test', i, 1_000_000_000 + i * 20_000_000,
                                        b'\x10\x27' * 320))
        self.assertTrue(wait_for(lambda: self.events['wake']))
        self.assertEqual(self.events['wake'][-1]['start_frame'], 0)
        self.assertEqual(self.events['wake'][-1]['end_frame'], 1280)
        self.assertTrue(wait_for(lambda: self.events['transcript']))
        transcript = self.events['transcript'][-1]
        self.assertEqual(transcript['text'], 'проверь коридор')
        self.assertEqual(transcript['status'], 'final')
        self.assertFalse(runtime.response_finished('wrong-session')['success'])
        self.assertTrue(runtime.response_finished(transcript['session_id'])['success'])
        self.assertEqual(session.state, 'waiting')


if __name__ == '__main__':
    unittest.main()
