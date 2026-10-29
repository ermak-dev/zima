#!/usr/bin/env python3
# FEATURE: Validated SLAM parameters share the private robot TOML configuration.
# SPEC: Export bounded parameters to the adapter/runtime; timing and deskew are
# unverified/off by default; never enable motion compensation without explicit verification.
# IMPLEMENTATION: shared TOML schema, bounded environment export and map admission budgets.
# RESULT: IMPLEMENTED — defaults, overrides, invalid bounds and verification guards pass.
# STATUS: DONE
import os
from pathlib import Path
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'python'))
from robot_config import load_config, export_env, ConfigError

DEFAULTS = {
    'record_path': '',
    'icp_min_goodness': .30, 'icp_max_iterations': 40,
    'icp_threshold_dist': .75, 'icp_decimation': 1,
    'insertion_linear': .10, 'insertion_angular_deg': 10.,
    'grid_resolution': .05, 'max_insert_range': 5.,
    'adapter_timeout_ms': 1500, 'max_pose_age_ms': 500,
    'odom_max_skew_ms': 100, 'source_time_verified': False,
    'deskew_enabled': False, 'beam_time_verified': False,
    'max_uncompensated_translation': .08, 'max_uncompensated_rotation_deg': 8.0,
}

class SlamConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root/'robot.toml'
        self.path.write_text(''); self.path.chmod(0o600)

    def load(self, text='', env=None):
        self.path.write_text(text)
        return load_config(self.path,root=self.root,environ=env or {})

    def test_defaults_are_bounded_and_deskew_is_not_assumed_verified(self):
        config = self.load()
        self.assertIn('slam',config,'shared SLAM settings are not implemented')
        self.assertEqual(config['slam'],DEFAULTS)
        env=export_env(config,environ={})
        for key,value in DEFAULTS.items():
            self.assertEqual(env['ROBOT_SLAM_'+key.upper()],str(value).lower())

    def test_file_and_environment_override_parameters_without_shell_evaluation(self):
        config=self.load('[slam]\nicp_min_goodness=0.5\nicp_max_iterations=60\nadapter_timeout_ms=900\n',
                         {'ROBOT_SLAM_ICP_MIN_GOODNESS':'0.65','ROBOT_SLAM_SOURCE_TIME_VERIFIED':'true'})
        self.assertEqual(config['slam']['icp_min_goodness'],.65)
        self.assertEqual(config['slam']['icp_max_iterations'],60)
        self.assertEqual(config['slam']['adapter_timeout_ms'],900)
        self.assertTrue(config['slam']['source_time_verified'])

    def test_invalid_bounds_and_types_are_rejected(self):
        for setting in ('icp_min_goodness=-0.1','icp_min_goodness=1.1','icp_min_goodness=nan',
                        'icp_max_iterations=0','icp_max_iterations=201','icp_decimation=0',
                        'icp_decimation=17','icp_threshold_dist=0.0','icp_threshold_dist=6.0',
                        'grid_resolution=0.001','grid_resolution=0.5','max_insert_range=0.1',
                        'max_insert_range=11.0','adapter_timeout_ms=0','adapter_timeout_ms=10001',
                        'max_pose_age_ms=0','odom_max_skew_ms=0','insertion_linear=-0.1',
                        'insertion_angular_deg=181.0','source_time_verified="yes"','icp_max_iterations=true',
                        'max_uncompensated_translation=-0.1','max_uncompensated_translation=1.1',
                        'max_uncompensated_rotation_deg=-1.0','max_uncompensated_rotation_deg=181.0',
                        'unknown_slam_setting=1'):
            with self.subTest(setting=setting), self.assertRaises(ConfigError):
                self.load('[slam]\n'+setting+'\n')

    # FEATURE: opt-in local raw Player recording path.
    # SPEC: empty default, TOML-relative path, explicit environment override, string only.
    # IMPLEMENTATION: add record_path to the shared schema, path resolution and env export.
    # RESULT: IMPLEMENTED — disabled default, path precedence and validation pass.
    # STATUS: DONE
    def test_record_path_is_opt_in_and_resolves_only_toml_paths(self):
        config = self.load('[slam]\nrecord_path="raw/session.jsonl"\n')
        self.assertEqual(config['slam']['record_path'], str(self.root/'raw/session.jsonl'))
        self.assertEqual(export_env(config, environ={})['ROBOT_SLAM_RECORD_PATH'],
                         str(self.root/'raw/session.jsonl'))
        config = self.load('[slam]\nrecord_path="raw/session.jsonl"\n',
                           {'ROBOT_SLAM_RECORD_PATH': 'explicit.jsonl'})
        self.assertEqual(config['slam']['record_path'], 'explicit.jsonl')
        for value in ('false', '12', '"bad\\npath"'):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                self.load('[slam]\nrecord_path='+value+'\n')

    # FEATURE: unified session recording shares run configuration and explicit env override.
    # SPEC: off by default, TOML-relative directory, string/control-character validation.
    def test_session_record_directory_is_opt_in_and_private_config_relative(self):
        config = self.load()
        self.assertIn('record_dir', config['run'])
        self.assertEqual(config['run']['record_dir'], '')
        config = self.load('[run]\nrecord_dir="runs/new-run"\n')
        self.assertEqual(config['run']['record_dir'], str(self.root/'runs/new-run'))
        self.assertEqual(export_env(config, environ={})['ROBOT_SESSION_RECORD_DIR'], str(self.root/'runs/new-run'))
        config = self.load('[run]\nrecord_dir="runs/new-run"\n', {'ROBOT_SESSION_RECORD_DIR':'explicit-run'})
        self.assertEqual(config['run']['record_dir'], 'explicit-run')
        for value in ('false', '12', '"bad\\npath"'):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                self.load('[run]\nrecord_dir='+value+'\n')

    def test_enabled_deskew_requires_both_explicit_time_verifications(self):
        for text in ('deskew_enabled=true', 'deskew_enabled=true\nsource_time_verified=true',
                     'deskew_enabled=true\nbeam_time_verified=true'):
            with self.subTest(text=text),self.assertRaises(ConfigError):
                self.load('[slam]\n'+text+'\n')
        config=self.load('[slam]\ndeskew_enabled=true\nsource_time_verified=true\nbeam_time_verified=true\n')
        self.assertTrue(config['slam']['deskew_enabled'])

if __name__=='__main__':unittest.main()
