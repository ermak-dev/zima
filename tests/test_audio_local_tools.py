#!/usr/bin/env python3
"""FEATURE: Offline acoustic reports distinguish transport/noise from a microphone.

SPEC: No playback, network access or external upload during local WAV analysis.
"""
import argparse
import array
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import wave

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))


class LocalAudioToolsTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.local_tools import run_local
        except ImportError:
            self.fail('local audio measurement tools are not implemented')
        self.run_local = run_local
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)

    def wav(self, name, samples):
        path = self.path / name
        with wave.open(str(path), 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(16000)
            w.writeframes(array.array('h', samples).tobytes())
        return path

    def run_report(self, wav, reference=None):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = self.run_local(argparse.Namespace(command='analyze', wav=wav, reference=reference))
        self.assertEqual(code, 0)
        return json.loads(out.getvalue())

    def test_silence_never_verifies_a_microphone(self):
        path = self.wav('silence.wav', [0] * 16000)
        result = self.run_report(path)
        self.assertFalse(result['microphone_verified'])
        self.assertFalse(result['speech_verified'])
        self.assertEqual(result['duration_s'], 1)

    def test_controlled_reference_is_required_for_verification(self):
        from robot_audio.acoustics import write_reference_wav
        baseline = self.wav('baseline.wav', [0] * 16000)
        probe = self.path / 'probe.wav'
        write_reference_wav(probe, duration_s=1, amplitude=.15)
        result = self.run_report(probe, baseline)
        self.assertTrue(result['microphone_verified'])
        self.assertFalse(result['speech_verified'])
        self.assertEqual(result['verification_scope'], 'controlled_tone_response_only')

    def test_rejects_a_symlink_instead_of_following_it(self):
        path = self.wav('silence.wav', [0] * 16000)
        link = self.path / 'link.wav'
        link.symlink_to(path)
        with self.assertRaises(ValueError):
            self.run_report(link)


if __name__ == '__main__':
    unittest.main()
