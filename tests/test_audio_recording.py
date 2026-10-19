#!/usr/bin/env python3
"""FEATURE: Private bounded WAV capture from the shared ROS PCM source."""
from pathlib import Path
import sys
import tempfile
import unittest
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.types import AudioChunk


class RecordingTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.recording import PcmRecorder
        except ImportError:
            self.fail('bounded local PCM recording is not implemented')
        self.Recorder = PcmRecorder
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'record.wav'

    def chunk(self, sequence, **kwargs):
        values = dict(stream_id='source', sequence=sequence, stamp_ns=sequence * 20_000_000,
                      pcm=b'\x10\x00' * 320)
        values.update(kwargs)
        return AudioChunk(**values)

    def test_writes_exact_duration_and_private_file(self):
        recorder = self.Recorder(self.path, seconds=.1)
        for i in range(5): recorder.feed(self.chunk(i))
        self.assertTrue(recorder.complete)
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        with wave.open(str(self.path), 'rb') as w:
            self.assertEqual((w.getnframes(), w.getframerate(), w.getnchannels(), w.getsampwidth()),
                             (1600, 16000, 1, 2))
        recorder.close()
        self.assertTrue(self.path.exists())

    def test_gap_or_epoch_change_removes_incomplete_recording(self):
        for changes in ({'sequence': 2}, {'sequence': 1, 'stream_id': 'new'},
                        {'sequence': 1, 'discontinuity': True}):
            with self.subTest(changes=changes):
                recorder = self.Recorder(self.path, seconds=.1)
                recorder.feed(self.chunk(0))
                with self.assertRaises(ValueError):
                    recorder.feed(self.chunk(**changes))
                self.assertFalse(self.path.exists())

    def test_never_overwrites_an_existing_file_and_removes_aborted_partial(self):
        self.path.write_bytes(b'keep')
        with self.assertRaises((ValueError, FileExistsError)):
            self.Recorder(self.path, seconds=.1)
        self.assertEqual(self.path.read_bytes(), b'keep')
        self.path.unlink()
        recorder = self.Recorder(self.path, seconds=.1)
        recorder.feed(self.chunk(0)); recorder.close()
        self.assertFalse(self.path.exists())


if __name__ == '__main__': unittest.main()
