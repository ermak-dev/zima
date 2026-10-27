#!/usr/bin/env python3
"""FEATURE: robot record writes one bounded test WAV from the camera without ROS or motion.

SPEC: Capture requires explicitly enabled input, opens a single source, leaves camera gain
alone unless asked, restores a requested gain lease after the recording, refuses a gain
above the measured working maximum, and retries lost samples inside its own timeout.
"""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import unittest
import wave
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))
ROBOT = ROOT / 'bin' / 'robot'

from robot_audio.types import AudioChunk


def pcm_chunks(count, *, stream_id='stream-1', start=0, gap_at=None):
    """Twenty-millisecond chunks; gap_at marks one chunk as lost samples."""
    return [AudioChunk(stream_id=stream_id, sequence=index, stamp_ns=index * 20_000_000,
                       pcm=bytes(640), discontinuity=index == gap_at)
            for index in range(start, start + count)]


class CaptureCliTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.wav = self.path / 'test.wav'
        self.config = self.path / 'audio.json'
        self.config.write_text(json.dumps({'input': {'enabled': True}}))
        spec = importlib.util.spec_from_file_location('capture_cli_test', ROOT / 'bin/audio/cli.py')
        self.cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.cli)
        self.events = []

    def source_factory(self, chunks):
        events = self.events

        class FakeSource:
            def __init__(inner, config=None, **kwargs):
                inner.config = config
                inner._chunks = list(chunks)

            def start(inner):
                events.append('start')

            def stop(inner):
                events.append('stop')

            def get_chunk(inner, timeout=None):
                if not inner._chunks:
                    raise queue.Empty
                return inner._chunks.pop(0)

        return FakeSource

    def no_gain(self):
        """Any camera gain access without --gain is a defect, not a test double."""
        def refuse(*args, **kwargs):
            raise AssertionError('capture must not touch camera gain without --gain')
        return patch.multiple('robot_audio.camera_session', CameraSSH=refuse, CameraGainLease=refuse)

    def lease_factory(self):
        events = self.events

        class FakeLease:
            volume = 90
            original_volume = 40

            def __init__(inner, ssh, **kwargs):
                inner.kwargs = kwargs
                events.append(('lease', kwargs.get('volume')))

            def __enter__(inner):
                events.append('gain-enter')
                return inner

            def __exit__(inner, *exc):
                events.append('gain-exit')
                return False

        return FakeLease

    def run_cli(self, *args, env=None):
        out = io.StringIO()
        self.errors = io.StringIO()
        argv = ['capture', str(self.wav), '--config', str(self.config), *args] if env is None \
            else ['capture', str(self.wav), *args]
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(self.errors):
            try:
                with patch.dict(os.environ, env or {}):
                    code = self.cli.main(argv)
            except SystemExit as exit_code:  # argparse rejects the value itself
                code = exit_code.code
        return code, out.getvalue()

    def test_capture_refuses_without_explicitly_enabled_input(self):
        self.config.write_text(json.dumps({'input': {'url': 'http://cam/audio.opus'}}))
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   side_effect=AssertionError('no source before consent')):
            code, out = self.run_cli('--seconds', '0.1')
        self.assertEqual(code, 2)
        # The refusal must say how to consent, not just that it happened.
        self.assertIn('--config', self.errors.getvalue())
        self.assertIn('input', out)
        self.assertIn('enabled', out)
        self.assertFalse(self.wav.exists())

    def cli_env(self):
        """A hostile environment: any direct ssh access is a defect."""
        env = dict(os.environ)
        env.pop('ROBOT_AUDIO_BIN', None)
        env.pop('XDG_CONFIG_HOME', None)
        forbidden = self.path / 'ssh'
        forbidden.write_text('#!/bin/sh\nprintf "unexpected hardware access\\n" >&2\nexit 97\n')
        forbidden.chmod(0o700)
        env['ROBOT_SSH'] = str(forbidden)
        return env

    def test_capture_prefers_the_user_default_config_without_dash_dash_config(self):
        home = self.path / 'home'
        (home / 'robot').mkdir(parents=True)
        (home / 'robot' / 'audio.json').write_text(json.dumps({'input': {'enabled': True}}))
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   self.source_factory(pcm_chunks(5))):
            code, out = self.run_cli('--seconds', '0.1',
                                     env={**self.cli_env(), 'XDG_CONFIG_HOME': str(home)})
        self.assertEqual(code, 0, out)
        self.assertEqual(json.loads(out)['frames'], 1600)

    def test_a_disabled_user_default_config_still_refuses(self):
        home = self.path / 'home'
        (home / 'robot').mkdir(parents=True)
        (home / 'robot' / 'audio.json').write_text(json.dumps({'input': {'enabled': False}}))
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   side_effect=AssertionError('no source before consent')):
            code, out = self.run_cli('--seconds', '0.1',
                                     env={**self.cli_env(), 'XDG_CONFIG_HOME': str(home)})
        self.assertEqual(code, 2)
        self.assertIn('--config', self.errors.getvalue())
        self.assertFalse(self.wav.exists())

    def test_robot_record_dispatches_capture_without_ros_or_ssh(self):
        forbidden = self.path / 'ssh'
        forbidden.write_text('#!/bin/sh\nprintf "unexpected hardware access\\n" >&2\nexit 97\n')
        forbidden.chmod(0o700)
        calls = self.path / 'calls'
        fake = self.path / 'audio'
        fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$AUDIO_TEST_CALLS"\n')
        fake.chmod(0o700)
        env = {**os.environ, 'ROBOT_SSH': str(forbidden), 'ROBOT_AUDIO_BIN': str(fake),
               'AUDIO_TEST_CALLS': str(calls)}
        result = subprocess.run([str(ROBOT), 'record', str(self.wav), '--seconds', '2'],
                                env=env, cwd=ROOT, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(calls.read_text().splitlines(),
                         ['capture', str(self.wav), '--seconds', '2'])
        help_text = subprocess.run([str(ROBOT), '--help'], env=env, cwd=ROOT,
                                   capture_output=True, text=True, timeout=10).stdout
        self.assertIn('robot record', help_text)
        self.assertIn('robot bug', help_text)

    def test_default_config_refusal_names_the_consent_switch(self):
        """The CLI default disables the microphone; the error must say how to enable it."""
        forbidden = self.path / 'ssh'
        forbidden.write_text('#!/bin/sh\nprintf "unexpected hardware access\\n" >&2\nexit 97\n')
        forbidden.chmod(0o700)
        env = {**os.environ, 'ROBOT_SSH': str(forbidden), 'XDG_CONFIG_HOME': str(self.path)}
        result = subprocess.run([str(ROBOT), 'record', str(self.wav), '--seconds', '1'],
                                env=env, cwd=ROOT, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        for fragment in ('--config', 'input', 'enabled'):
            self.assertIn(fragment, result.stderr)
        self.assertNotIn('Traceback', result.stderr)
        self.assertFalse(self.wav.exists())

    def test_capture_refuses_without_explicitly_enabled_input(self):
        self.config.write_text(json.dumps({'input': {'url': 'http://cam/audio.opus'}}))
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   side_effect=AssertionError('no source before consent')):
            code, _ = self.run_cli('--seconds', '0.1')
        self.assertEqual(code, 2)
        self.assertFalse(self.wav.exists())

    def test_capture_writes_one_private_wav_from_a_single_source(self):
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   self.source_factory(pcm_chunks(5))):
            code, out = self.run_cli('--seconds', '0.1')
        self.assertEqual(code, 0, out)
        self.assertEqual(self.events, ['start', 'stop'])
        self.assertEqual(self.wav.stat().st_mode & 0o777, 0o600)
        with wave.open(str(self.wav)) as handle:
            self.assertEqual((handle.getnchannels(), handle.getsampwidth(),
                              handle.getframerate(), handle.getnframes()), (1, 2, 16000, 1600))
        report = json.loads(out)
        self.assertEqual(report['file'], str(self.wav))
        self.assertEqual(report['frames'], 1600)
        self.assertEqual(report['sample_rate'], 16000)
        self.assertIn('level', report)
        self.assertNotIn('camera_volume', report)

    def test_requested_gain_is_leased_around_the_recording_and_restored(self):
        with patch('robot_audio.camera_session.CameraSSH'), \
             patch('robot_audio.camera_session.CameraGainLease', self.lease_factory()), \
             patch('robot_audio.capture.CameraAudioSource', self.source_factory(pcm_chunks(5))):
            code, out = self.run_cli('--seconds', '0.1', '--gain', '90')
        self.assertEqual(code, 0, out)
        # Gain must rise before the stream opens: the reload drops a live connection.
        self.assertEqual(self.events, [('lease', 90), 'gain-enter', 'start', 'stop', 'gain-exit'])
        self.assertEqual(json.loads(out)['camera_volume'], {'original': 40, 'active': 90})

    def test_gain_above_the_measured_working_maximum_is_refused(self):
        for bad in ('100', '91', '-1'):
            with self.subTest(gain=bad):
                with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                           side_effect=AssertionError('no source after bad gain')):
                    code, _ = self.run_cli('--seconds', '0.1', '--gain', bad)
                self.assertEqual(code, 2)
                self.assertFalse(self.wav.exists())

    def test_keyboard_interrupt_during_gain_setup_exits_quietly(self):
        # CameraSession turns SIGINT into CameraSessionError('session_interrupted');
        # the CLI must treat that as a user interrupt (130), not a failure (1).
        from robot_audio.camera_session import CameraSessionError
        def interrupted(*args, **kwargs):
            raise CameraSessionError('session_interrupted')
        with patch('robot_audio.camera_session.CameraSSH'), \
             patch('robot_audio.camera_session.CameraGainLease', interrupted), \
             patch('robot_audio.capture.CameraAudioSource',
                   side_effect=AssertionError('no capture after an interrupt')):
            code, out = self.run_cli('--seconds', '0.1', '--gain', '90')
        self.assertEqual(code, 130)
        self.assertFalse(self.wav.exists())

    def test_existing_output_is_refused_by_name(self):
        self.wav.write_bytes(b'keep')
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   side_effect=AssertionError('no capture before the output check')):
            code, _ = self.run_cli('--seconds', '0.1')
        self.assertEqual(code, 2)
        self.assertIn('test.wav', self.errors.getvalue())
        self.assertEqual(self.wav.read_bytes(), b'keep')

    def test_progress_is_logged_to_stderr_with_phase_timings(self):
        # A test capture can take ~10 s of invisible SSH and stream warmup;
        # stderr must show each phase and when the recording actually starts.
        with patch('robot_audio.camera_session.CameraSSH'), \
             patch('robot_audio.camera_session.CameraGainLease', self.lease_factory()), \
             patch('robot_audio.capture.CameraAudioSource', self.source_factory(pcm_chunks(5))):
            code, _ = self.run_cli('--seconds', '0.1', '--gain', '90')
        self.assertEqual(code, 0)
        log = self.errors.getvalue()
        self.assertIn('gain 40->90', log)
        self.assertIn('connecting to http://cam/audio.opus', log)
        self.assertIn('first audio after', log)
        self.assertIn('recording 0.1s', log)
        self.assertIn('wrote 1600 frames', log)
        for line in log.splitlines():
            self.assertRegex(line, r'^\[[0-9]+\.[0-9]s\] ')

    def test_invalid_durations_are_refused_before_any_capture(self):
        for seconds in ('0', '31', 'nan'):
            with self.subTest(seconds=seconds):
                with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                           side_effect=AssertionError('no source after bad duration')):
                    code, _ = self.run_cli('--seconds', seconds)
                self.assertEqual(code, 2)
                self.assertFalse(self.wav.exists())

    def test_lost_samples_are_retried_inside_the_timeout(self):
        chunks = (pcm_chunks(2) + pcm_chunks(1, start=5, gap_at=5) +
                  pcm_chunks(5, stream_id='stream-2'))
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   self.source_factory(chunks)):
            code, out = self.run_cli('--seconds', '0.1')
        self.assertEqual(code, 0, out)
        self.assertEqual(json.loads(out)['frames'], 1600)
        with wave.open(str(self.wav)) as handle:
            self.assertEqual(handle.getnframes(), 1600)

    def test_a_source_that_never_delivers_times_out_and_leaves_no_partial_file(self):
        with self.no_gain(), patch('robot_audio.capture.CameraAudioSource',
                                   self.source_factory(pcm_chunks(2))):
            code, _ = self.run_cli('--seconds', '0.1', '--timeout', '0.5')
        self.assertEqual(code, 1)
        self.assertEqual(self.events, ['start', 'stop'])
        self.assertFalse(self.wav.exists())


if __name__ == '__main__':
    unittest.main()
