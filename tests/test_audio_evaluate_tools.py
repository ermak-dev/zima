#!/usr/bin/env python3
"""FEATURE: Wake evaluation CLI distinguishes validation from held-out acceptance."""
import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))


class Backend:
    def __init__(self, manifest): pass
    def predict(self, pcm): return .9 if any(pcm) else .0
    def reset(self): pass


class EvaluationToolsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        records = []
        for name, value, split in (('positive', 1000, 'test'), ('negative', 0, 'test'),
                                    ('validation', 2000, 'validation')):
            with wave.open(str(self.path / f'{name}.wav'), 'wb') as w:
                w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
                w.writeframes(struct.pack('<h', value) * 6400)
            records.append({'id': name, 'recording_id': name, 'speaker_id': name,
                'path': f'{name}.wav', 'split': split, 'condition': 'synthetic',
                'events': [{'start': 0, 'end': .2}] if value else []})
        self.dataset = self.path / 'dataset.json'
        self.dataset.write_text(json.dumps({'schema_version': 1, 'phrase': 'test phrase',
            'language': 'en', 'criteria': {'max_false_activations_per_hour': 0,
            'max_miss_rate': 0, 'max_latency_s': .5}, 'records': records}))
        manifest = {'schema_version': 1, 'phrase': 'test phrase', 'language': 'en',
                    'license': 'test-only', 'verified': False}
        for key in ('model_file', 'melspec_model_file', 'embedding_model_file'):
            manifest[key] = key + '.onnx'
            (self.path / manifest[key]).write_bytes(key.encode())
        self.model = self.path / 'wake.json'; self.model.write_text(json.dumps(manifest))

    def run_report(self, **changes):
        from robot_audio.local_tools import run_local
        args = dict(command='evaluate', manifest=self.dataset, wake_manifest=None,
                    threshold=.5, split='test', calibrate=False)
        args.update(changes)
        output = io.StringIO()
        with contextlib.redirect_stdout(output), patch('robot_audio.wake.OpenWakeWordBackend', Backend):
            result = run_local(argparse.Namespace(**args))
        self.assertEqual(result, 0)
        return json.loads(output.getvalue())

    def test_dataset_validation_does_not_invent_detection_metrics(self):
        report = self.run_report()
        self.assertEqual(report['mode'], 'dataset_validation_only')
        self.assertFalse(report['model_verified'])
        self.assertNotIn('accepted', report)
        self.assertEqual(report['records'], 3)

    def test_held_out_run_records_exact_model_artifact_hashes(self):
        report = self.run_report(wake_manifest=self.model)
        self.assertEqual(report['split'], 'test')
        self.assertTrue(report['accepted'])
        self.assertEqual(report['metrics']['false_activations'], 0)
        self.assertEqual(report['metrics']['misses'], 0)
        self.assertEqual(report['artifacts_sha256']['model_file'], hashlib.sha256(b'model_file').hexdigest())

    def test_calibration_uses_validation_not_test(self):
        report = self.run_report(wake_manifest=self.model, calibrate=True)
        self.assertEqual(report['split'], 'validation')
        self.assertTrue(all(item['split'] == 'validation' for item in report['reports']))

    def test_model_phrase_must_match_the_dataset(self):
        data = json.loads(self.model.read_text()); data['phrase'] = 'different'
        self.model.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            self.run_report(wake_manifest=self.model)


if __name__ == '__main__': unittest.main()
