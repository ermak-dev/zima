#!/usr/bin/env python3
"""FEATURE: Audio CLI dispatch is independent of motion and validates before I/O.

SPEC: Local configuration/help must work without ROS, credentials or a robot.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
ROBOT = ROOT / 'bin' / 'robot'


class AudioCliTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.calls = self.path / 'calls'
        forbidden = self.path / 'ssh'
        forbidden.write_text('#!/bin/sh\nprintf "unexpected hardware access\\n" >&2\nexit 97\n')
        forbidden.chmod(0o700)
        self.env = {**os.environ, 'ROBOT_SSH': str(forbidden)}
        self.env.pop('ROBOT_AUDIO_BIN', None)

    def run_cli(self, *args, env=None):
        return subprocess.run([str(ROBOT), 'audio', *args], env=env or self.env,
                              cwd=ROOT, capture_output=True, text=True, timeout=10)

    def test_robot_dispatches_audio_without_controller_handoff(self):
        fake = self.path / 'audio'
        fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$AUDIO_TEST_CALLS"\n')
        fake.chmod(0o700)
        env = {**self.env, 'ROBOT_AUDIO_BIN': str(fake), 'AUDIO_TEST_CALLS': str(self.calls)}
        result = self.run_cli('status', '--timeout', '1', env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls.read_text().splitlines(), ['status', '--timeout', '1'])

    def test_help_and_default_config_need_no_ros_or_network(self):
        result = self.run_cli('--help')
        self.assertEqual(result.returncode, 0, result.stderr)
        for name in ('run', 'play', 'say', 'status', 'stop', 'check-config'):
            self.assertIn(name, result.stdout)
        result = self.run_cli('check-config')
        self.assertEqual(result.returncode, 0, result.stderr)
        config = json.loads(result.stdout)
        self.assertFalse(config['input']['enabled'])
        self.assertFalse(config['tts']['enabled'])
        self.assertFalse(config['tts']['allow_external'])

    def test_invalid_credentials_in_config_are_not_echoed(self):
        config = self.path / 'invalid.json'
        config.write_text(json.dumps({'input': {'url': 'http://user:do-not-print-this@cam/audio.opus'}}))
        result = self.run_cli('check-config', '--config', str(config))
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('do-not-print-this', result.stdout + result.stderr)
        self.assertNotIn('Traceback', result.stderr)

    def test_import_prepares_a_private_asset_without_robot_access(self):
        import wave
        source = self.path / 'notification.wav'
        with wave.open(str(source), 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
            w.writeframes(b'\x00\x00' * 1600)
        config = self.path / 'audio.json'
        config.write_text(json.dumps({'output': {'asset_dir': str(self.path / 'assets')}}))
        result = self.run_cli('import', str(source), '--config', str(config))
        self.assertEqual(result.returncode, 0, result.stderr)
        asset = json.loads(result.stdout)
        self.assertRegex(asset['asset_id'], r'^[a-f0-9]{64}$')
        path = self.path / 'assets' / (asset['asset_id'] + '.wav')
        self.assertTrue(path.is_file())
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_reference_generation_is_local_bounded_and_never_overwrites(self):
        import wave
        path = self.path / 'reference.wav'
        result = self.run_cli('reference', str(path))
        self.assertEqual(result.returncode, 0, result.stderr)
        with wave.open(str(path), 'rb') as reader:
            self.assertEqual(reader.getnframes(), 12000)
            self.assertEqual(reader.getframerate(), 16000)
        self.assertNotEqual(self.run_cli('reference', str(path)).returncode, 0)

    def test_deploy_is_explicit_and_never_touches_controller(self):
        binary = self.path / 'robot-audio'
        header = bytearray(52)
        header[:7] = b'\x7fELF\x01\x01\x01'
        header[16:20] = b'\x02\x00\x28\x00'
        binary.write_bytes(header)
        rejected = self.run_cli('deploy', '--binary', str(binary))
        self.assertNotEqual(rejected.returncode, 0)
        log = self.path / 'ssh-command'
        uploaded = self.path / 'uploaded'
        ssh = self.path / 'ssh-upload'
        ssh.write_text('#!/usr/bin/env python3\nimport pathlib,sys\npathlib.Path(' + repr(str(log)) + ').write_text(sys.argv[-1])\npathlib.Path(' + repr(str(uploaded)) + ').write_bytes(sys.stdin.buffer.read())\nprint("DEPLOYED")\n')
        ssh.chmod(0o700)
        result = self.run_cli('deploy', '--binary', str(binary), '--confirm',
                              env={**self.env, 'ROBOT_SSH': str(ssh)})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(uploaded.read_bytes(), binary.read_bytes())
        self.assertIn('/tmp/robot-audio/robot-audio', log.read_text())
        for forbidden in ('RoboController', 'robot disable', 'rrwatchdoge', 'play '):
            self.assertNotIn(forbidden, log.read_text())

    def test_train_dispatch_is_local_and_keeps_model_unverified(self):
        import contextlib,importlib.util,io,sys
        from unittest.mock import patch
        sys.path.insert(0,str(ROOT/'ros2'))
        spec=importlib.util.spec_from_file_location('audio_cli_for_training',ROOT/'bin/audio/cli.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        output=io.StringIO()
        with patch('robot_audio.training.train_model',return_value={'verified':False}) as train,contextlib.redirect_stdout(output):
            result=cli.main(['train','dataset.json','--features-dir','features','--output','model','--kind','mlp'])
        self.assertEqual(result,0)
        self.assertFalse(json.loads(output.getvalue())['verified'])
        self.assertEqual(train.call_args.kwargs['kind'],'mlp')

    def test_invalid_playback_arguments_are_rejected_before_ros(self):
        for args in (('play', '../private'), ('play', 'beep', '--volume', 'nan'),
                     ('play', 'beep', '--volume', '2'), ('play', 'beep', '--ttl', '-1')):
            with self.subTest(args=args):
                result = self.run_cli(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('Traceback', result.stderr)
                self.assertNotIn('unexpected hardware access', result.stderr)


if __name__ == '__main__':
    unittest.main()
