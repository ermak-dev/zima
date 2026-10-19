#!/usr/bin/env python3
"""FEATURE: The real robot audio CLI runs a disabled, isolated ROS service safely."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'source ROS and audio interface overlay')
class AudioCliRosTests(unittest.TestCase):
    def test_disabled_service_reports_state_and_refuses_capture_enable(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            forbidden = p / 'ssh'
            marker = p / 'unexpected-device-access'
            forbidden.write_text('#!/bin/sh\ntouch ' + str(marker) + '\nexit 97\n')
            forbidden.chmod(0o700)
            env = {**os.environ, 'ROS_DOMAIN_ID': '216', 'ROBOT_SSH': str(forbidden)}
            process = subprocess.Popen([str(ROOT / 'bin/robot'), 'audio', 'run'], cwd=ROOT,
                                       env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 10
                result = None
                while time.monotonic() < deadline and process.poll() is None:
                    result = subprocess.run([str(ROOT / 'bin/robot'), 'audio', 'status', '--timeout', '.4'],
                                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        break
                self.assertIsNotNone(result)
                self.assertEqual(result.returncode, 0, result.stderr)
                state = json.loads(result.stdout)
                self.assertEqual(state['input']['state'], 'disabled')
                self.assertEqual(state['output']['state'], 'disabled')
                listen = subprocess.run([str(ROOT / 'bin/robot'), 'audio', 'listen', 'on'],
                                        cwd=ROOT, env=env, capture_output=True, text=True, timeout=8)
                self.assertNotEqual(listen.returncode, 0)
                self.assertTrue(listen.stdout, listen.stderr)
                self.assertFalse(json.loads(listen.stdout)['success'])
                muted = subprocess.run([str(ROOT / 'bin/robot'), 'audio', 'mute', 'on'],
                                       cwd=ROOT, env=env, capture_output=True, text=True, timeout=8)
                self.assertEqual(muted.returncode, 0, muted.stderr)
                self.assertTrue(json.loads(muted.stdout)['success'])
                self.assertFalse(marker.exists())
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    out, error = process.communicate(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill(); out, error = process.communicate()
                self.assertNotIn('Traceback', error)
                self.assertIn(process.returncode, (0, 130), error)


if __name__ == '__main__': unittest.main()
