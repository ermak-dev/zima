#!/usr/bin/env python3
"""FEATURE: Controller-visible speech readiness and idempotent session completion."""
from pathlib import Path
import unittest

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2' / 'robot_audio_interfaces'


class SessionInterfacesTests(unittest.TestCase):
    def test_session_state_is_separate_from_transcribed_text(self):
        path = PACKAGE / 'msg' / 'SpeechState.msg'
        self.assertTrue(path.exists(), 'speech state interface is not implemented')
        fields = path.read_text().splitlines()
        for field in ('string state', 'string session_id', 'bool listening',
                      'bool wake_ready', 'bool asr_ready', 'string duplex_mode', 'string error'):
            self.assertIn(field, fields)

    def test_finish_requires_the_matching_session_id(self):
        path = PACKAGE / 'srv' / 'FinishSpeech.srv'
        self.assertTrue(path.exists(), 'speech completion service is not implemented')
        fields = path.read_text().splitlines()
        self.assertEqual(fields, ['string session_id', '---', 'bool success', 'string message'])


if __name__ == '__main__':
    unittest.main()
