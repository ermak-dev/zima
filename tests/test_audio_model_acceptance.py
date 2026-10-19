#!/usr/bin/env python3
"""FEATURE: Russian wake readiness is tied to held-out evaluation of exact artifacts."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.wake import ConfigurationRequired, load_wake_manifest


class ModelAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.manifest = {'schema_version': 1, 'phrase': 'выбранная фраза', 'language': 'ru',
                         'license': 'test-only', 'verified': True, 'evaluation_report': 'report.json'}
        hashes = {}
        for key in ('model_file', 'melspec_model_file', 'embedding_model_file'):
            self.manifest[key] = key + '.onnx'
            data = key.encode(); (self.path / self.manifest[key]).write_bytes(data)
            hashes[key] = hashlib.sha256(data).hexdigest()
        self.report = {'accepted': True, 'split': 'test', 'phrase': self.manifest['phrase'],
                       'language': 'ru', 'threshold': .6, 'artifacts_sha256': hashes}
        self.file = self.path / 'manifest.json'
        self.file.write_text(json.dumps(self.manifest))
        self.write_report()

    def write_report(self):
        (self.path / 'report.json').write_text(json.dumps(self.report))

    def test_only_matching_held_out_report_can_verify_russian_model(self):
        for changes in ({'split': 'validation'}, {'phrase': 'другая фраза'}, {'language': 'en'},
                        {'artifacts_sha256': {}}, {'threshold': 1.5}):
            with self.subTest(changes=changes):
                original = dict(self.report)
                self.report.update(changes); self.write_report()
                with self.assertRaises(ConfigurationRequired):
                    load_wake_manifest(self.file, require_russian=True)
                self.report = original

    def test_changing_feature_or_keyword_weights_invalidates_acceptance(self):
        (self.path / self.manifest['embedding_model_file']).write_bytes(b'new embeddings')
        with self.assertRaises(ConfigurationRequired):
            load_wake_manifest(self.file, require_russian=True)

    def test_runtime_does_not_deploy_a_different_russian_threshold(self):
        from unittest.mock import patch
        from robot_audio.runtime import AudioRuntime
        class Session:
            def close(self): pass
        config = {'wake': {'enabled': True, 'manifest': str(self.file), 'threshold': .5},
                  'speech': {'enabled': True, 'asr_model_path': 'injected', 'language': 'ru'}}
        with patch('robot_audio.wake.OpenWakeWordBackend') as backend:
            runtime = AudioRuntime(config, session=Session())
            try:
                self.assertIsNone(runtime.detector)
                backend.assert_not_called()
            finally:
                runtime.close()

    def test_matching_report_exposes_the_evaluated_threshold(self):
        manifest = load_wake_manifest(self.file, require_russian=True)
        self.assertEqual(manifest['evaluation_threshold'], .6)


if __name__ == '__main__': unittest.main()
