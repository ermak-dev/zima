#!/usr/bin/env python3
"""FEATURE: Native Python audio service without ROS or wake-word activation.
SPEC: Loopback-only IPC, explicit utterances, bounded/cancellable output, shared PCM.
"""
import base64
import importlib.util
import io
import json
from pathlib import Path
import queue
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
import wave

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))


class NativeContractTests(unittest.TestCase):
    def test_package_lives_at_stable_python_import_root(self):
        self.assertTrue((ROOT / 'python/robot_audio/runtime.py').is_file(),
                        'native Python audio package has not been moved')

    def test_wake_configuration_is_removed_not_silently_ignored(self):
        # Import the current package only for the RED phase before relocation.
        sys.path.append(str(ROOT / 'python'))
        from robot_audio.config import default_config, validate_config
        self.assertNotIn('wake', default_config())
        for value in ({}, {'enabled': False}, {'enabled': True}):
            with self.assertRaises(ValueError):
                validate_config({'wake': value})

    def test_removed_commands_and_flags_are_rejected(self):
        spec = importlib.util.spec_from_file_location('native_audio_cli', ROOT / 'bin/audio/cli.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        for args in (['run', '--experimental-wake'], ['train', 'dataset.json', '--features-dir', 'x', '--output', 'y'],
                     ['evaluate', 'dataset.json']):
            with self.subTest(args=args):
                with self.assertRaises(SystemExit) as raised:
                    cli.parser().parse_args(args)
                self.assertEqual(raised.exception.code, 2)


class FakeRuntime:
    def __init__(self):
        self.callbacks = {}
        self.calls = []
        self.closed = False
        self.listening = True
        self.muted = False
        self.block = False
        self.entered = threading.Event()

    def set_callbacks(self, **callbacks): self.callbacks = callbacks
    def start(self):
        self.callbacks['input_state']({'state': 'streaming', 'microphone_verified': False})
        self.callbacks['playback']({'state': 'idle', 'speaking': False})
    def close(self): self.closed = True
    def set_listening(self, enabled):
        self.listening = enabled
        return {'success': True, 'message': 'checked'}
    def set_muted(self, enabled):
        self.muted = enabled
        return {'success': True, 'message': 'checked'}
    def response_finished(self, session_id): return {'success': session_id == 'matching'}
    def begin_utterance(self): return {'success': True}
    def execute(self, kind, request, task_id, cancel, feedback):
        self.calls.append((kind, request, task_id))
        self.entered.set()
        if self.block:
            cancel.wait(2)
        return {'error_code': 3 if cancel.is_set() else 0, 'message': 'checked'}


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue((ROOT / 'python/robot_audio/service.py').is_file(),
                        'standalone audio IPC service is not implemented')
        from robot_audio.service import AudioService
        self.runtime = FakeRuntime()
        self.service = AudioService(runtime=self.runtime)
        self.service.start()
        self.addCleanup(self.service.close)

    def request(self, command, **values):
        return self.service.request({'command': command, **values})

    def test_status_preserves_uncertainty_and_no_pcm_or_credentials(self):
        state = self.request('status')
        self.assertEqual(state['input']['state'], 'streaming')
        self.assertFalse(state['input']['microphone_verified'])
        self.assertFalse(state['output']['speaking'])
        self.assertNotIn('config', state)

    def test_listen_mute_finish_and_manual_start(self):
        self.assertTrue(self.request('mute', enabled=True)['success'])
        self.assertTrue(self.runtime.listening)
        self.assertTrue(self.request('listen', enabled=False)['success'])
        self.assertFalse(self.runtime.listening)
        self.assertTrue(self.request('finish', session_id='matching')['success'])
        self.assertFalse(self.request('finish', session_id='other')['success'])
        self.assertTrue(self.request('utterance')['success'])

    def test_play_say_return_completed_result_not_request_acceptance(self):
        for kind, payload in (('play', {'asset_id': 'beep'}), ('say', {'text': 'Проверка'})):
            result = self.request(kind, **payload)
            self.assertEqual(result['error_code'], 0)
            self.assertTrue(result['task_id'])
        self.assertEqual([value[0] for value in self.runtime.calls], ['play', 'say'])

    def test_invalid_requests_never_reach_runtime(self):
        for payload in ({'command': 'play', 'asset_id': '../secret'},
                        {'command': 'play', 'asset_id': 'beep', 'volume': float('nan')},
                        {'command': 'say', 'text': ''}, {'command': 'say', 'text': 'x\0y'},
                        {'command': 'say', 'text': 'x', 'priority': True},
                        {'command': 'mute', 'enabled': 'false'},
                        {'command': 'record', 'seconds': 31},
                        {'command': 'status', 'timeout': -1}, {'command': 'wake'}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.service.request(payload)
        self.assertEqual(self.runtime.calls, [])

    def test_stop_reaches_backend_and_waiting_requests_do_not_block_control(self):
        self.runtime.block = True
        results = []
        thread = threading.Thread(target=lambda: results.append(self.request('play', asset_id='beep')))
        thread.start()
        self.assertTrue(self.runtime.entered.wait(1))
        self.assertEqual(self.request('stop')['requested'], 1)
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results[0]['error_code'], 3)

    def test_timeout_cancels_backend_without_retry(self):
        self.runtime.block = True
        result = self.request('play', asset_id='beep', timeout=.05)
        self.assertEqual(result['error_code'], 3)
        self.assertEqual(len(self.runtime.calls), 1)

    def test_admission_is_bounded_before_backend(self):
        self.runtime.block = True
        self.service.config['output']['queue_size'] = 1
        threads = [threading.Thread(target=lambda: self.request('play', asset_id='beep')) for _ in range(2)]
        for thread in threads: thread.start()
        from test_audio_runtime import wait_for
        self.assertTrue(wait_for(lambda: len(self.runtime.calls) == 2))
        try:
            self.assertEqual(self.request('play', asset_id='beep')['error_code'], 2)
        finally:
            self.request('stop')
            for thread in threads: thread.join(2)

    def test_record_consumes_shared_pcm_and_returns_private_wav_payload(self):
        from robot_audio.types import AudioChunk
        results = []
        thread = threading.Thread(target=lambda: results.append(self.request('record', seconds=.1, timeout=1)))
        thread.start()
        self.assertTrue(self.service.recording.wait(1))
        for seq in range(5):
            self.runtime.callbacks['chunk'](AudioChunk('fixture', seq, seq * 20_000_000, b'\0\0' * 320))
        thread.join(2)
        self.assertFalse(thread.is_alive())
        data = base64.b64decode(results[0]['wav_base64'], validate=True)
        with wave.open(io.BytesIO(data)) as reader:
            self.assertEqual(reader.getparams()[:3], (1, 2, 16000))
            self.assertEqual(reader.getnframes(), 1600)

    def test_record_gap_and_listen_off_remove_partial_recording(self):
        from robot_audio.types import AudioChunk
        for stop in ('gap', 'privacy'):
            errors = []
            def record():
                try: self.request('record', seconds=.1, timeout=.3)
                except (ValueError, TimeoutError) as error: errors.append(error)
            thread = threading.Thread(target=record)
            thread.start()
            self.assertTrue(self.service.recording.wait(1))
            self.runtime.callbacks['chunk'](AudioChunk('fixture', 0, 0, b'\0\0' * 320))
            if stop == 'gap':
                self.runtime.callbacks['chunk'](AudioChunk('fixture', 3, 1, b'\0\0' * 320))
            else: self.request('listen', enabled=False)
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(len(errors), 1)
            self.assertFalse(self.service.recording.is_set())

    def test_optional_readonly_controller_remains_wired_without_ros(self):
        from robot_audio.service import AudioService
        from robot_audio.commands import BatteryProvider, BatterySnapshot
        runtime = FakeRuntime()
        service = AudioService(runtime=runtime, config={'controller': {'enabled': True}},
                               battery_reader=BatteryProvider(BatterySnapshot(percent=42, freshness='fresh')))
        service.start()
        try:
            value = {'session_id': 'matching', 'stream_id': 'fixture', 'text': 'Какой у тебя заряд?',
                     'is_final': True, 'status': 'final'}
            runtime.callbacks['transcript'](value)
            self.assertTrue(runtime.entered.wait(1))
            self.assertEqual(runtime.calls[0][1]['text'], 'У меня сорок два процента заряда.')
        finally: service.close()

    def test_close_closes_runtime(self):
        self.service.close()
        self.assertTrue(self.runtime.closed)


if __name__ == '__main__': unittest.main()
