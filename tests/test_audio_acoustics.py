#!/usr/bin/env python3
"""FEATURE: Local, evidence-based microphone assessment without live devices.

SPEC: Quiet ADC noise or a decodable stream is never microphone verification;
verification needs a controlled reference with measurable signal and SNR.
"""
import array
import importlib
import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
import wave

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))


class AcousticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.audio = importlib.import_module('robot_audio.acoustics')

    def pcm(self, values):
        return array.array('h', values).tobytes()

    def noise(self, count=16000, amplitude=5):
        rng = random.Random(7)
        return [rng.randint(-amplitude, amplitude) for _ in range(count)]

    def test_signal_statistics_remove_dc_and_count_clipping(self):
        report = self.audio.analyze_pcm(self.pcm([100] * 16000))
        self.assertEqual(report['duration_s'], 1)
        self.assertEqual(report['dc_offset'], 100)
        self.assertEqual(report['ac_rms'], 0)
        self.assertIsNone(report['ac_dbfs'])
        self.assertFalse(report['microphone_verified'])
        clipped = self.audio.analyze_pcm(self.pcm([-32768, 32767, 0, 10]))
        self.assertEqual(clipped['clipped_samples'], 2)
        self.assertEqual(clipped['peak'], 32768)
        json.dumps(report, allow_nan=False)

    def test_noise_never_verifies_microphone(self):
        noise = self.pcm(self.noise())
        report = self.audio.assess_probe(noise, noise, tone_hz=880)
        self.assertFalse(report['microphone_verified'])
        self.assertIn('insufficient_signal', report['reasons'])
        self.assertIn('insufficient_snr', report['reasons'])
        self.assertIn('reference_not_detected', report['reasons'])

    def test_controlled_tone_over_baseline_passes_acoustic_not_speech_check(self):
        baseline = self.pcm(self.noise())
        probe = self.pcm([int(1500 * math.sin(2 * math.pi * 880 * i / 16000)) + n
                          for i, n in enumerate(self.noise())])
        report = self.audio.assess_probe(baseline, probe, tone_hz=880)
        self.assertTrue(report['microphone_verified'])
        self.assertFalse(report['speech_verified'])
        self.assertGreater(report['snr_db'], 30)
        self.assertGreater(report['reference_amplitude'], 1400)
        self.assertEqual(report['reasons'], [])
        json.dumps(report, allow_nan=False)

    def test_wrong_tone_and_clipping_do_not_verify(self):
        baseline = self.pcm(self.noise())
        wrong = self.pcm([int(1500 * math.sin(2 * math.pi * 440 * i / 16000)) for i in range(16000)])
        result = self.audio.assess_probe(baseline, wrong, tone_hz=880)
        self.assertFalse(result['microphone_verified'])
        self.assertIn('reference_not_detected', result['reasons'])
        clipped = self.pcm([32767 if i % 2 else -32768 for i in range(16000)])
        result = self.audio.assess_probe(baseline, clipped, tone_hz=880)
        self.assertFalse(result['microphone_verified'])
        self.assertIn('clipping', result['reasons'])

    def test_invalid_pcm_and_assessment_settings_are_rejected(self):
        for payload in (b'', b'\0', b'\0\0\0'):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.audio.analyze_pcm(payload)
        with self.assertRaises(ValueError):
            self.audio.analyze_pcm(b'\0\0', sample_rate=0)
        with self.assertRaises(ValueError):
            self.audio.assess_probe(self.pcm(self.noise()), self.pcm(self.noise()), tone_hz=9000)
        with self.assertRaises(ValueError):
            self.audio.assess_probe(self.pcm(self.noise()), self.pcm(self.noise()), min_snr_db=float('nan'))

    def test_reference_wav_is_short_bounded_and_never_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'reference.wav'
            self.audio.write_reference_wav(path)
            with wave.open(str(path), 'rb') as w:
                self.assertEqual((w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()),
                                 (1, 2, 16000, 12000))
                samples = array.array('h', w.readframes(w.getnframes()))
            self.assertLessEqual(max(map(abs, samples)), 900)
            self.assertEqual((samples[0], samples[-1]), (0, 0))
            with self.assertRaises(FileExistsError):
                self.audio.write_reference_wav(path)
            for kwargs in ({'duration_s': 30}, {'amplitude': 1.1}, {'frequency': 9000}):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    self.audio.write_reference_wav(Path(d) / 'invalid.wav', **kwargs)

    def test_wav_analysis_bounds_input_and_checks_pcm_format(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'reference.wav'
            self.audio.write_reference_wav(path)
            report = self.audio.analyze_wav(path)
            self.assertEqual(report['duration_s'], .75)
            self.assertEqual(report['sample_rate'], 16000)
            self.assertFalse(report['microphone_verified'])
            with self.assertRaises(ValueError):
                self.audio.analyze_wav(path, max_duration_s=.1)
            bad = Path(d) / 'bad.wav'
            with wave.open(str(bad), 'wb') as w:
                w.setnchannels(1)
                w.setsampwidth(1)
                w.setframerate(16000)
                w.writeframes(b'\x80' * 20)
            with self.assertRaises(ValueError):
                self.audio.analyze_wav(bad)
            link = Path(d) / 'link.wav'
            link.symlink_to(path)
            with self.assertRaises(ValueError):
                self.audio.analyze_wav(link)


if __name__ == '__main__':
    unittest.main()
