#!/usr/bin/env python3
"""FEATURE: Typed ROS audio messages and cancellable playback/speech actions.

SPEC: PCM carries epoch, sequence and estimated timestamps. Playback results
are distinct from request acceptance; audio interfaces have no motion fields.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2' / 'robot_audio_interfaces'


def fields(path):
    return [line.strip() for line in path.read_text().splitlines()
            if line.strip() and not line.lstrip().startswith('#')]


class AudioInterfacesTests(unittest.TestCase):
    def require(self, relative):
        path = PACKAGE / relative
        self.assertTrue(path.is_file(), f'audio interface is not implemented: {relative}')
        return path

    def test_package_is_a_rosidl_interface_package(self):
        xml = self.require('package.xml').read_text()
        cmake = self.require('CMakeLists.txt').read_text()
        self.assertIn('<member_of_group>rosidl_interface_packages</member_of_group>', xml)
        self.assertIn('rosidl_generate_interfaces', cmake)
        self.assertIn('builtin_interfaces', xml)

    def test_pcm_has_discontinuity_and_format_metadata(self):
        actual = fields(self.require('msg/AudioChunk.msg'))
        for required in ('std_msgs/Header header', 'string stream_id', 'uint64 sequence',
                         'uint32 sample_rate', 'uint8 channels', 'string sample_format',
                         'bool discontinuity', 'bool estimated_timestamp', 'uint8[] data'):
            self.assertIn(required, actual)

    def test_input_state_does_not_equate_streaming_with_a_verified_microphone(self):
        actual = fields(self.require('msg/AudioInputState.msg'))
        for required in ('string state', 'float64 last_frame_age',
                         'bool microphone_verified', 'string source', 'string error'):
            self.assertIn(required, actual)

    def test_actions_have_result_and_feedback_sections(self):
        for action in ('PlayAudio', 'Say'):
            with self.subTest(action=action):
                actual = fields(self.require(f'action/{action}.action'))
                self.assertEqual(actual.count('---'), 2)
                self.assertIn('float32 volume', actual)
                self.assertIn('builtin_interfaces/Duration max_queue_wait', actual)
                self.assertIn('uint16 error_code', actual)
                self.assertIn('string phase', actual)
                self.assertIn('builtin_interfaces/Duration played', actual)
        self.assertIn('string asset_id', fields(self.require('action/PlayAudio.action')))
        self.assertIn('string text', fields(self.require('action/Say.action')))

    def test_voice_events_have_ids_and_source_epochs(self):
        wake = fields(self.require('msg/WakeWord.msg'))
        transcript = fields(self.require('msg/SpeechTranscript.msg'))
        self.assertIn('string event_id', wake)
        self.assertIn('string stream_id', wake)
        self.assertIn('string session_id', transcript)
        self.assertIn('string status', transcript)
        self.assertIn('bool is_final', transcript)


if __name__ == '__main__':
    unittest.main()
