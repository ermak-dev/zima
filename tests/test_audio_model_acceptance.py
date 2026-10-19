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

    def test_experimental_model_requires_explicit_opt_in_and_still_checks_report(self):
        self.manifest['verified']=False
        self.file.write_text(json.dumps(self.manifest))
        with self.assertRaises(ConfigurationRequired):load_wake_manifest(self.file,require_russian=True)
        result=load_wake_manifest(self.file,require_russian=True,allow_experimental=True)
        self.assertTrue(result['experimental'])
        self.assertFalse(result['verified'])
        self.report['accepted']=False;self.write_report()
        with self.assertRaises(ConfigurationRequired):load_wake_manifest(self.file,require_russian=True,allow_experimental=True)

    def test_opposite_stress_report_cannot_verify_the_chosen_pronunciation(self):
        self.manifest['pronunciation']='вы́бранная фра́за'
        self.file.write_text(json.dumps(self.manifest))
        self.report['pronunciation']='выбра́нная фра́за';self.write_report()
        with self.assertRaises(ConfigurationRequired):load_wake_manifest(self.file,require_russian=True)

    def test_runtime_experimental_opt_in_is_visible_and_does_not_set_verified(self):
        from unittest.mock import patch
        from robot_audio.runtime import AudioRuntime
        class Session:
            state='waiting';session_id=None;last_error=''
            def close(self):pass
            def tick(self,**kwargs):return []
        self.manifest['verified']=False;self.file.write_text(json.dumps(self.manifest))
        config={'wake':{'enabled':True,'manifest':str(self.file),'threshold':.6},
                'input':{'microphone_verified':True},
                'speech':{'enabled':True,'asr_model_path':'injected','language':'ru'}}
        with patch('robot_audio.wake.OpenWakeWordBackend'):
            runtime=AudioRuntime(config,session=Session(),allow_experimental_wake=True)
            states=[];runtime.set_callbacks(speech_state=states.append)
            try:
                runtime.start();self.assertIsNotNone(runtime.detector)
                self.assertEqual(states[-1]['error'],'experimental_model')
                self.assertFalse(json.loads(self.file.read_text())['verified'])
            finally:runtime.close()

    def test_standalone_russian_detector_cannot_bypass_experimental_opt_in(self):
        from unittest.mock import patch
        from robot_audio.runtime import AudioRuntime
        self.manifest['verified']=False;self.file.write_text(json.dumps(self.manifest))
        with patch('robot_audio.wake.OpenWakeWordBackend') as backend:
            runtime=AudioRuntime({'wake':{'enabled':True,'manifest':str(self.file),'threshold':.6}})
            try:
                self.assertIsNone(runtime.detector);backend.assert_not_called()
            finally:runtime.close()

    def test_matching_report_exposes_the_evaluated_threshold(self):
        manifest = load_wake_manifest(self.file, require_russian=True)
        self.assertEqual(manifest['evaluation_threshold'], .6)


if __name__ == '__main__': unittest.main()
