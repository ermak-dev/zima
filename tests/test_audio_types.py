#!/usr/bin/env python3
"""FEATURE: A bounded, explicit PCM contract shared by the audio stack.

SPEC: Host audio is signed little-endian PCM16, 16 kHz, mono. Stream changes,
sequence numbers and estimated capture times remain visible to consumers.
"""
import dataclasses
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))


class AudioChunkTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.types import AudioChunk
        except ImportError:
            self.fail('AudioChunk contract is not implemented')
        self.AudioChunk = AudioChunk

    def chunk(self, **changes):
        args = dict(stream_id='stream-1', sequence=0, stamp_ns=1_000_000_000,
                    pcm=b'\x00\x00' * 320)
        args.update(changes)
        return self.AudioChunk(**args)

    def test_pcm_metadata_and_duration(self):
        chunk = self.chunk()
        self.assertEqual(chunk.frames, 320)
        self.assertEqual(chunk.duration_s, .02)
        self.assertEqual((chunk.sample_rate, chunk.channels, chunk.sample_format),
                         (16000, 1, 's16le'))
        self.assertTrue(chunk.estimated_timestamp)
        self.assertFalse(chunk.discontinuity)

    def test_audio_chunk_is_immutable(self):
        chunk = self.chunk()
        with self.assertRaises(dataclasses.FrozenInstanceError):
            chunk.sequence = 5

    def test_rejects_invalid_or_unbounded_frames(self):
        for changes in ({'pcm': b''}, {'pcm': b'\x00'}, {'pcm': bytearray(640)},
                        {'pcm': b'\x00\x00' * 16001}, {'sample_rate': 48000},
                        {'channels': 2}, {'sample_format': 'float32'},
                        {'stream_id': ''}, {'stream_id': 'x' * 129},
                        {'sequence': -1}, {'sequence': True},
                        {'sequence': 2**64}, {'stamp_ns': -1}):
            with self.subTest(changes=repr(changes)[:100]):
                with self.assertRaises((ValueError, TypeError)):
                    self.chunk(**changes)

    def test_discontinuity_and_source_epoch_are_preserved(self):
        chunk = self.chunk(stream_id='reconnected', sequence=9, discontinuity=True)
        self.assertEqual(chunk.stream_id, 'reconnected')
        self.assertEqual(chunk.sequence, 9)
        self.assertTrue(chunk.discontinuity)


if __name__ == '__main__':
    unittest.main()
