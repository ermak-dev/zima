#!/usr/bin/env python3
"""FEATURE: `robot say "text"` speaks local Piper TTS through the helper, no ROS.

SPEC: dispatch must not source ROS or touch the controller; a fake ssh proves
the only hardware access is the helper playback; Piper and the voice model are
injected, since neither is downloaded automatically.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest
import wave
import io

ROOT = Path(__file__).resolve().parents[1]
ROBOT = ROOT / 'bin' / 'robot'


def wav_bytes(seconds=.5):
    b = io.BytesIO()
    with wave.open(b, 'wb') as f:
        f.setnchannels(1); f.setsampwidth(2); f.setframerate(16000)
        f.writeframes(b'\x10\x00' * int(seconds * 16000))
    return b.getvalue()


class SayCliTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.ssh_log = self.dir / 'ssh-calls'
        # A helper-shaped ssh: accepts any `helper play ...` and prints COMPLETED.
        self.ssh = self.dir / 'ssh-fake'
        self.ssh.write_text(textwrap.dedent(f'''\
            #!/bin/sh
            echo "$@" >> {self.ssh_log}
            cat > /dev/null
            echo STARTED; echo COMPLETED
        '''))
        self.ssh.chmod(0o700)

        # Fake piper module: same interface the engine imports lazily.
        piper_pkg = self.dir / 'fakes' / 'piper'
        piper_pkg.mkdir(parents=True)
        (piper_pkg / '__init__.py').write_text(textwrap.dedent('''
            class PiperVoice:
                def __init__(self, path):
                    pass
                @classmethod
                def load(cls, path):
                    return cls(path)
                def synthesize(self, text):
                    class Chunk:
                        audio_int16_bytes = b'\\x00\\x40' * 1600
                    return [Chunk()]
        '''))

        # Voice model + config, as PiperTTS expects them.
        voice_dir = self.dir / 'voices'
        voice_dir.mkdir()
        (voice_dir / 'voice.onnx').write_bytes(b'weights')
        (voice_dir / 'voice.onnx.json').write_text('{"audio": {"sample_rate": 16000}}')

        self.config = self.dir / 'audio.json'
        self.config.write_text(json.dumps({
            'output': {'enabled': True, 'asset_dir': str(self.dir / 'assets'),
                       'volume': .35},
            'tts': {'piper_model': str(voice_dir / 'voice.onnx')},
        }))
        # Private yandex env file, as a real deployment would have it.
        self.yandex_env = self.dir / 'yandex.env'
        self.yandex_env.write_text('YANDEX_API_KEY=ya-key\nYANDEX_FOLDER=b1gfolder\n')
        self.yandex_env.chmod(0o600)
        self.env = {**os.environ,
                    'ROBOT_SSH': str(self.ssh),
                    'PYTHONPATH': str(self.dir / 'fakes'),
                    'HOME': str(self.dir)}  # keep the real user config out

    def run_cli(self, *args):
        return subprocess.run([str(ROBOT), 'say', *args], env=self.env, cwd=ROOT,
                              capture_output=True, text=True, timeout=30)

    def test_help_needs_no_ros_or_ssh(self):
        env = {**os.environ, 'ROBOT_SSH': '/nonexistent/ssh'}
        result = subprocess.run([str(ROBOT), 'say', '--help'], env=env, cwd=ROOT,
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('text', result.stdout)

    def test_empty_text_is_rejected_without_touching_ssh(self):
        result = self.run_cli('   ')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('Traceback', result.stderr)
        self.assertFalse(self.ssh_log.exists())

    def test_speaks_through_the_helper_with_injected_piper(self):
        result = self.run_cli('Привет, робот.', '--config', str(self.config),
                              '--piper-module-path', str(self.dir / 'fakes'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.ssh_log.exists(), 'playback must reach the helper over ssh')
        calls = self.ssh_log.read_text()
        self.assertIn('play', calls)
        self.assertIn('0.350000', calls)

    def test_default_configuration_works_without_a_config_file(self):
        # `robot say` is an explicit command: defaults apply, the voice is
        # discovered in ~/.local/share/piper-voices, no enabled flags needed.
        voices = self.dir / '.local' / 'share' / 'piper-voices'
        voices.mkdir(parents=True)
        (voices / 'ru_RU-dmitri-medium.onnx').write_bytes(b'weights')
        (voices / 'ru_RU-dmitri-medium.onnx.json').write_text('{"audio": {"sample_rate": 16000}}')
        result = self.run_cli('Привет.', '--piper-module-path', str(self.dir / 'fakes'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.ssh_log.exists())
        self.assertIn('play', self.ssh_log.read_text())

    def test_no_voice_anywhere_is_an_actionable_error_without_ssh(self):
        result = self.run_cli('Привет.', '--piper-module-path', str(self.dir / 'fakes'))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('piper-voices', result.stderr)
        self.assertIn('downloaded automatically', result.stderr)
        self.assertNotIn('Traceback', result.stderr)
        self.assertFalse(self.ssh_log.exists())

    def test_yandex_env_argument_selects_speechkit_without_piper(self):
        # No piper module path and no installed voice: with --yandex-env the
        # request must still reach the provider and the robot's helper.
        import http.server
        import threading
        seen = []
        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                length = int(self.headers.get('Content-Length', 0))
                seen.append({'auth': self.headers.get('Authorization'),
                             'body': self.rfile.read(length)})
                pcm = b'\x30\x01' * 1600
                self.send_response(200)
                self.send_header('Content-Length', str(len(pcm)))
                self.end_headers()
                self.wfile.write(pcm)
            def log_message(self, *args):
                pass
        server = http.server.HTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.shutdown)
        config = self.dir / 'yandex.json'
        config.write_text(json.dumps({
            'output': {'enabled': True, 'asset_dir': str(self.dir / 'assets')},
            'tts': {'yandex_url': f'http://127.0.0.1:{server.server_port}/tts'},
        }))
        env = {**self.env}
        env.pop('PYTHONPATH', None)
        result = self.run_cli_env(env, 'Привет.', '--config', str(config),
                                  '--yandex-env', str(self.yandex_env))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.ssh_log.exists())
        self.assertIn('play', self.ssh_log.read_text())
        self.assertEqual(seen[0]['auth'], 'Api-Key ya-key')
        self.assertNotIn(b'folder_id', seen[0]['body'])
        self.assertIn(b'voice=filipp', seen[0]['body'])

    def run_cli_env(self, env, *args):
        return subprocess.run([str(ROBOT), 'say', *args], env=env, cwd=ROOT,
                              capture_output=True, text=True, timeout=30)

    def test_missing_model_is_a_config_error_without_ssh(self):
        bad = self.dir / 'bad.json'
        bad.write_text(json.dumps({
            'output': {'enabled': True, 'asset_dir': str(self.dir / 'assets')},
            'tts': {'piper_model': str(self.dir / 'absent.onnx')},
        }))
        result = self.run_cli('Текст.', '--config', str(bad),
                              '--piper-module-path', str(self.dir / 'fakes'))
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('Traceback', result.stderr)
        self.assertIn('absent.onnx', result.stderr)
        self.assertFalse(self.ssh_log.exists())

    def test_helper_busy_is_reported_not_retried(self):
        self.ssh.write_text(f'''#!/bin/sh
            echo "$@" >> {self.ssh_log}
            cat > /dev/null
            echo BUSY
        ''')
        self.ssh.chmod(0o700)
        result = self.run_cli('Привет.', '--config', str(self.config),
                              '--piper-module-path', str(self.dir / 'fakes'))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('busy', result.stderr.lower())
        self.assertEqual(self.ssh_log.read_text().count('play'), 1,
                         'a busy robot must not be retried automatically')


if __name__ == '__main__':
    unittest.main(verbosity=2)
