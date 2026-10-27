#!/usr/bin/env python3
# FEATURE: ROS-free foreground audio and explicit speech activation.
# SPEC: Manual utterances need no wake module; HTTP is loopback-only and bounded.
# IMPLEMENTATION: Native HTTP adapter and shared-PCM recorder; explicit session activation.
# RESULT: IMPLEMENTED — native transport, cancellation, private WAV and shutdown verified.
# STATUS: DONE
import importlib.util
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))


class StandaloneTests(unittest.TestCase):
    def test_manual_session_without_importing_wake(self):
        self.assertTrue((ROOT / 'python/robot_audio/speech.py').is_file())
        from robot_audio.speech import SpeechSession
        from robot_audio.types import AudioChunk
        class Recognizer:
            def close(self): pass
        session = SpeechSession(Recognizer(), clock=lambda: 1)
        self.assertTrue(session.begin_utterance())
        self.assertFalse(session.begin_utterance())
        session.feed(AudioChunk('manual', 0, 0, b'\0\0' * 320, discontinuity=True))
        self.assertEqual(session.state, 'listening')
        self.assertEqual(session._stream_id, 'manual')
        session.feed(AudioChunk('manual', 2, 0, b'\0\0' * 320))
        self.assertEqual(session.last_error, 'audio_gap')

    def test_runtime_manual_requires_input_and_asr_and_no_playback(self):
        self.assertTrue((ROOT / 'python/robot_audio/runtime.py').is_file())
        from robot_audio.runtime import AudioRuntime
        from robot_audio.speech import SpeechSession
        from unittest.mock import Mock
        runtime = AudioRuntime()
        self.assertFalse(runtime.begin_utterance()['success'])
        session = SpeechSession(Mock(), clock=lambda: 1)
        runtime = AudioRuntime(source=Mock(), session=session, config={'input': {'enabled': True}}, clock=lambda: 1)
        self.addCleanup(runtime.close)
        self.assertTrue(runtime.begin_utterance()['success'])
        self.assertFalse(runtime.begin_utterance()['success'])
        runtime.set_listening(False)
        self.assertFalse(runtime.begin_utterance()['success'])

    def test_http_rejects_oversize_and_non_object_before_dispatch(self):
        self.assertTrue((ROOT / 'python/robot_audio/service.py').is_file())
        from robot_audio.service import AudioRequestHandler
        from unittest.mock import Mock
        for body, length, status in ((b'[]', '2', 400), (b'{}', '65537', 413),
                                     (b'{"command":"status"}', None, 411),
                                     (b'{"command":"status"}', '-1', 400)):
            handler = object.__new__(AudioRequestHandler)
            handler.path = '/request'
            handler.headers = {} if length is None else {'Content-Length': length}
            handler.rfile, handler.wfile = io.BytesIO(body), io.BytesIO()
            handler.server = Mock()
            handler.send_response = Mock()
            handler.send_header = Mock()
            handler.end_headers = Mock()
            handler.do_POST()
            handler.send_response.assert_called_once_with(status)
            handler.server.service.request.assert_not_called()

    def test_server_refuses_non_loopback_binding_without_opening_socket(self):
        self.assertTrue((ROOT / 'python/robot_audio/service.py').is_file())
        from robot_audio.service import create_server
        with patch('robot_audio.service.ThreadingHTTPServer') as server:
            with self.assertRaises(ValueError): create_server(object(), host='0.0.0.0')
            server.assert_not_called()
            create_server(object(), port=8098)
            self.assertEqual(server.call_args.args[0], ('127.0.0.1', 8098))

    def test_shared_recording_ignores_later_chunks_once_target_is_complete(self):
        from robot_audio.service import AudioService
        from robot_audio.types import AudioChunk
        from test_audio_service import FakeRuntime
        service = AudioService(runtime=FakeRuntime())
        # Hold the condition to deterministically model more capture callbacks before
        # the recorder's waiting thread resumes and serializes the completed WAV.
        record = {'pcm': bytearray(b'\0\0' * 1600), 'bytes': 3200,
                  'previous': AudioChunk('fixture', 4, 0, b'\0\0' * 320), 'error': ''}
        service._record = record
        service._chunk(AudioChunk('reconnect', 0, 0, b'\0\0' * 320, discontinuity=True))
        self.assertEqual(record['error'], '')
        self.assertEqual(len(record['pcm']), 3200)
        service.close()

    def test_manual_utterance_is_transcribed_without_wake_modules(self):
        from concurrent.futures import Future
        from robot_audio.speech import SpeechSession
        from robot_audio.types import AudioChunk
        from unittest.mock import Mock
        future = Future()
        recognizer = Mock()
        recognizer.submit.return_value = future
        session = SpeechSession(recognizer, vad=lambda pcm: pcm != b'\0\0' * 320,
                                silence_s=.2, clock=lambda: 0)
        self.assertTrue(session.begin_utterance(now=0))
        session.feed(AudioChunk('stream', 0, 0, b'\1\1' * 320, discontinuity=True), now=.01)
        session.feed(AudioChunk('stream', 1, 1, b'\0\0' * 320), now=.3)
        self.assertEqual(session.state, 'transcribing')
        future.set_result('Какой заряд?')
        result, = session.tick(now=.4)
        self.assertEqual(result.text, 'Какой заряд?')
        self.assertEqual(result.stream_id, 'stream')
        session.response_finished(result.session_id)
        self.assertEqual(session.state, 'waiting')

    def test_wake_ros_and_training_sources_and_dependencies_are_gone(self):
        # STATUS: wake word, ROS node, evaluation and training were removed for good.
        for name in ('wake', 'ros_node', 'training', 'evaluation'):
            self.assertFalse((ROOT / f'python/robot_audio/{name}.py').exists(), name)
        requirements = (ROOT / 'requirements-audio.txt').read_text().lower()
        for dependency in ('onnxruntime', 'scipy', 'scikit-learn', 'openwakeword', 'tqdm', 'requests'):
            self.assertNotIn(dependency, requirements)
        for needed in ('numpy', 'faster-whisper'):
            self.assertIn(needed, requirements)
        example = json.loads((ROOT / 'config/audio.example.json').read_text())
        self.assertNotIn('wake', example)

    def test_import_and_default_runtime_do_not_load_ros_or_wake(self):
        import subprocess
        code = '''import sys
sys.path.insert(0, sys.argv[1])
class Block:
    def find_spec(self, fullname, *args):
        if fullname.startswith(('rclpy', 'onnxruntime', 'openwakeword')) or fullname == 'robot_audio.wake':
            raise AssertionError('forbidden module: ' + fullname)
sys.meta_path.insert(0, Block())
from robot_audio.speech import SpeechSession
from robot_audio.echo import EchoController
from robot_audio.service import AudioService
service = AudioService()
service.start()
assert service.request({'command': 'status'})['input']['state'] == 'disabled'
service.close()
'''
        result = subprocess.run([sys.executable, '-I', '-c', code, str(ROOT / 'python')],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_foreground_always_releases_server_even_if_runtime_close_fails(self):
        from robot_audio.service import run_foreground
        from unittest.mock import Mock
        service, server = Mock(), Mock()
        server.serve_forever.side_effect = KeyboardInterrupt
        service.close.side_effect = RuntimeError('close failed')
        with patch('robot_audio.service.AudioService', return_value=service), \
                patch('robot_audio.service.create_server', return_value=server), \
                patch('robot_audio.service.signal.signal'):
            with self.assertRaises(RuntimeError): run_foreground()
        server.server_close.assert_called_once()

    def test_cli_records_private_complete_wav_and_never_overwrites(self):
        import base64
        import tempfile
        import wave
        import contextlib
        spec = importlib.util.spec_from_file_location('record_cli', ROOT / 'bin/audio/cli.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        data = io.BytesIO()
        with wave.open(data, 'wb') as writer:
            writer.setnchannels(1); writer.setsampwidth(2); writer.setframerate(16000)
            writer.writeframes(b'\0\0' * 1600)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'record.wav'
            with patch.object(cli, '_ipc', return_value={'wav_base64': base64.b64encode(data.getvalue()).decode(),
                                                       'frames': 1600}) as request, \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(['record', str(path), '--seconds', '.1']), 0)
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
                self.assertEqual(path.read_bytes(), data.getvalue())
                self.assertEqual(cli.main(['record', str(path), '--seconds', '.1']), 2)
                self.assertEqual(request.call_count, 1)

    def test_client_connection_failure_requests_cancellation_without_retry(self):
        import contextlib
        from urllib.error import URLError
        spec = importlib.util.spec_from_file_location('cancel_cli', ROOT / 'bin/audio/cli.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        with patch.object(cli, '_ipc', side_effect=[URLError('lost response'), {'requested': 1}]) as request, \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(['play', 'beep']), 1)
        self.assertEqual(request.call_count, 2)
        first = request.call_args_list[0].args[1]
        second = request.call_args_list[1].args[1]
        self.assertEqual(second, {'command': 'stop', 'task_id': first['task_id']})

    def test_cli_native_transport_endpoint_and_foreground_dispatch(self):
        spec = importlib.util.spec_from_file_location('standalone_cli', ROOT / 'bin/audio/cli.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        args = cli.parser().parse_args(['status', '--endpoint', 'http://127.0.0.1:8098'])
        self.assertEqual(args.endpoint, 'http://127.0.0.1:8098')
        with patch.object(cli, '_native_request', return_value=0) as request:
            self.assertEqual(cli.main(['status']), 0)
            request.assert_called_once()
        with patch('robot_audio.service.run_foreground') as run:
            self.assertEqual(cli.main(['run', '--port', '8098']), 0)
            self.assertEqual(run.call_args.kwargs['port'], 8098)


if __name__ == '__main__': unittest.main()
