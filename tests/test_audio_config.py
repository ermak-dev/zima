#!/usr/bin/env python3
"""FEATURE: Audio starts without live capture, playback or external API side effects.

SPEC: Explicit opt-in, local model paths, bounded values and no credentials in URLs.
"""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))


class AudioConfigTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.config import default_config, load_config, validate_config
        except ImportError:
            self.fail('audio configuration is not implemented')
        self.defaults, self.load, self.validate = default_config, load_config, validate_config

    def test_defaults_do_not_touch_hardware_or_external_services(self):
        config = self.defaults()
        for section in ('input', 'output', 'tts', 'wake', 'speech', 'echo'):
            self.assertFalse(config[section]['enabled'])
        self.assertFalse(config['tts']['allow_external'])
        self.assertFalse(config['input']['microphone_verified'])
        self.assertFalse(config['echo']['verified'])
        config['input']['enabled'] = True
        self.assertFalse(self.defaults()['input']['enabled'])

    def test_partial_config_is_merged_and_validated(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'audio.json'
            p.write_text(json.dumps({'input': {'enabled': True, 'url': 'http://127.0.0.1:8081/audio.opus'}}))
            config = self.load(p)
        self.assertTrue(config['input']['enabled'])
        self.assertEqual(config['input']['url'], 'http://127.0.0.1:8081/audio.opus')
        self.assertFalse(config['output']['enabled'])

    def test_rejects_unknown_keys_credentials_and_unbounded_parameters(self):
        invalid = [
            {'unknown': {}}, {'input': {'passwrod': 'not-a-secret'}},
            {'input': {'url': 'http://user:secret@cam/audio.opus'}},
            {'input': {'url': 'file:///etc/passwd'}},
            {'output': {'host': '-oProxyCommand=unsafe'}},
            {'output': {'helper': '/tmp/helper; unsafe'}},
            {'output': {'volume': float('nan')}}, {'output': {'max_volume': 1.1}},
            {'output': {'max_duration_s': 3600}},
            {'wake': {'threshold': -1}}, {'wake': {'threshold': True}},
            {'speech': {'max_command_s': 1000}}, {'speech': {'language': 'not-a-language'}},
        ]
        for value in invalid:
            with self.subTest(value=value):
                with self.assertRaises((ValueError, TypeError)):
                    self.validate(value)

    def test_tts_requires_explicit_external_consent_and_model_voice(self):
        for patch in ({'enabled': True},
                      {'enabled': True, 'allow_external': True, 'model': 'provider/model'},
                      {'enabled': True, 'model': 'provider/model', 'voice': 'voice'}):
            with self.subTest(patch=patch):
                with self.assertRaises(ValueError):
                    self.validate({'tts': patch})
        config = self.validate({'tts': {'enabled': True, 'allow_external': True,
                                       'model': 'provider/model', 'voice': 'voice'}})
        self.assertEqual(config['tts']['model'], 'provider/model')

    def test_full_duplex_cannot_be_enabled_without_evaluation(self):
        with self.assertRaises(ValueError):
            self.validate({'speech': {'mode': 'full_duplex'}})
        config = self.validate({'speech': {'mode': 'full_duplex'},
                                'echo': {'enabled': True, 'verified': True}})
        self.assertEqual(config['speech']['mode'], 'full_duplex')

    def test_limits_match_the_capture_recognizer_and_native_helper(self):
        for patch in ({'input': {'chunk_ms': 15}}, {'output': {'max_duration_s': 31}},
                      {'speech': {'max_command_s': 31}}, {'speech': {'listen_timeout_s': 16}},
                      {'wake': {'preroll_s': .01}}, {'echo': {'max_drift_ms': 101}},
                      {'echo': {'enabled': True}, 'input': {'chunk_ms': 40}}):
            with self.subTest(patch=patch):
                with self.assertRaises(ValueError):
                    self.validate(patch)
        self.assertEqual(self.defaults()['output']['helper'], '/tmp/robot-audio/robot-audio')

    def test_speaker_quiet_hours_and_mute_have_explicit_configuration(self):
        config = self.validate({'output': {'muted': True, 'quiet_hours': ['22:00-08:00'], 'timezone': 'UTC'}})
        self.assertTrue(config['output']['muted'])
        for changes in ({'quiet_hours': '22:00-08:00'}, {'quiet_hours': ['25:00-08:00']},
                        {'timezone': 'invalid/timezone'}):
            with self.assertRaises(ValueError): self.validate({'output': changes})

    def test_tts_can_explicitly_use_dotenv_with_a_bounded_generation(self):
        config = self.validate({'tts': {'api_key_file': '', 'env_file': '/private/.env', 'max_output_tokens': 512,
                                        'audio_format': 'pcm16', 'pcm_sample_rate': 24000}})
        self.assertEqual(config['tts']['env_file'], '/private/.env')
        self.assertTrue(config['tts']['require_verbatim'])
        with self.assertRaises(ValueError): self.validate({'tts': {'max_output_tokens': 5000}})

    def test_vad_threshold_can_be_calibrated_for_lidar_noise(self):
        from unittest.mock import patch
        from robot_audio.runtime import AudioRuntime
        patch_config={'speech':{'enabled':True,'asr_model_path':'injected-local-model','vad_threshold_dbfs':-35}}
        with patch('robot_audio.speech.FasterWhisperASR',return_value=lambda pcm:'тест'):
            runtime=AudioRuntime(patch_config)
            try:self.assertEqual(runtime.session.vad.threshold_dbfs,-35)
            finally:runtime.close()
        with self.assertRaises(ValueError):self.validate({'speech':{'vad_threshold_dbfs':float('nan')}})

    def test_readonly_controller_is_explicit_and_cache_ids_are_not_paths(self):
        self.assertFalse(self.defaults()['controller']['enabled'])
        config=self.validate({'controller':{'enabled':True,'allow_reported_battery':True,'reply_cache':{'ответ':'a'*64}}})
        self.assertTrue(config['controller']['enabled'])
        with self.assertRaises(ValueError):self.validate({'controller':{'reply_cache':{'ответ':'../file'}}})

    def test_recognition_requires_explicit_local_models(self):
        for patch in ({'wake': {'enabled': True}}, {'speech': {'enabled': True}}):
            with self.subTest(patch=patch):
                with self.assertRaises(ValueError):
                    self.validate(patch)


if __name__ == '__main__':
    unittest.main()
