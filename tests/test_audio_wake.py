#!/usr/bin/env python3
"""FEATURE: Local, gap-aware wake-word detection without a microphone.
SPEC: 16 kHz S16LE, 80 ms inference, explicit local models, bounded pre-roll.
"""
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.wake import (ConfigurationRequired, OpenWakeWordBackend,
                              WakeDetector, load_wake_manifest)


def chunk(sequence, value=100, stream='one', samples=320, **kwargs):
    values = dict(stream_id=stream, sequence=sequence, stamp_ns=sequence * 20_000_000,
                  pcm=struct.pack('<h', value) * samples, sample_rate=16000,
                  channels=1, sample_format='s16le', discontinuity=False,
                  estimated_timestamp=True)
    values.update(kwargs)
    return SimpleNamespace(**values)


class Backend:
    def __init__(self, score=.9):
        self.score = score
        self.frames = []
        self.resets = 0

    def predict(self, pcm):
        self.frames.append(pcm)
        return self.score

    def reset(self):
        self.resets += 1


class WakeTests(unittest.TestCase):
    def test_infers_only_after_exact_eighty_ms_and_emits_metadata(self):
        backend = Backend()
        detector = WakeDetector(backend, phrase='hello robot', model='local')
        for i in range(3):
            self.assertEqual(detector.feed(chunk(i)), [])
        event, = detector.feed(chunk(3))
        self.assertEqual(len(backend.frames[0]), 2560)
        self.assertEqual(event.phrase, 'hello robot')
        self.assertEqual(event.model, 'local')
        self.assertEqual(event.stream_id, 'one')
        self.assertEqual(event.sequence, 3)
        self.assertEqual(event.stamp_ns, 80_000_000)
        self.assertEqual(len(event.pre_roll_pcm), 2560)
        self.assertTrue(event.event_id)

    def test_debounce_and_disabled_mode_do_not_replay_audio(self):
        detector = WakeDetector(Backend(), phrase='hi', debounce_s=.2)
        events = []
        for i in range(16):
            events.extend(detector.feed(chunk(i)))
        self.assertEqual(len(events), 2)
        detector.set_enabled(False)
        self.assertEqual(detector.feed(chunk(16, samples=1280)), [])
        detector.set_enabled(True)
        self.assertEqual(detector.feed(chunk(17)), [])

    def test_gap_resets_partial_frame_and_preroll(self):
        backend = Backend()
        detector = WakeDetector(backend, phrase='hi')
        detector.feed(chunk(0, 222))
        self.assertEqual(detector.feed(chunk(2, 333)), [])
        detector.feed(chunk(3, 333)); detector.feed(chunk(4, 333))
        event, = detector.feed(chunk(5, 333))
        self.assertEqual(set(struct.unpack('<1280h', event.pre_roll_pcm)), {333})
        self.assertGreaterEqual(backend.resets, 1)

    def test_duplicate_block_is_ignored(self):
        detector = WakeDetector(Backend(), phrase='hi')
        detector.feed(chunk(0))
        detector.feed(chunk(0))
        detector.feed(chunk(1)); detector.feed(chunk(2))
        event, = detector.feed(chunk(3))
        self.assertEqual(len(event.pre_roll_pcm), 2560)

    def test_new_stream_and_discontinuity_reset(self):
        backend = Backend(.1)
        detector = WakeDetector(backend, phrase='hi')
        detector.feed(chunk(0))
        detector.feed(chunk(0, stream='two'))
        detector.feed(chunk(1, stream='two', discontinuity=True))
        self.assertGreaterEqual(backend.resets, 2)

    def test_vad_filters_events_but_does_not_skip_model_frames(self):
        backend = Backend()
        detector = WakeDetector(backend, phrase='hi', vad_threshold=.5)
        for i in range(4):
            result = detector.feed(chunk(i), speech_probability=.1)
        self.assertEqual(result, [])
        self.assertEqual(len(backend.frames), 1)
        for i in range(4, 8):
            result = detector.feed(chunk(i), speech_probability=.9)
        self.assertEqual(len(result), 1)

    def test_preroll_bounded_and_nonfinite_score_cannot_trigger(self):
        backend = Backend(float('nan'))
        detector = WakeDetector(backend, phrase='hi', pre_roll_s=1)
        for i in range(100):
            self.assertEqual(detector.feed(chunk(i)), [])
        self.assertLessEqual(len(detector.pre_roll_pcm), 32000)

    def test_invalid_input_and_thresholds_fail(self):
        for kwargs in ({'threshold': float('nan')}, {'pre_roll_s': 99}, {'debounce_s': -1}):
            with self.assertRaises(ValueError): WakeDetector(Backend(), phrase='hi', **kwargs)
        detector = WakeDetector(Backend(), phrase='hi')
        with self.assertRaises(ValueError): detector.feed(chunk(0, sample_rate=48000))
        with self.assertRaises(ValueError): detector.feed(chunk(0, pcm=b'x'))


class ManifestTests(unittest.TestCase):
    def manifest(self, root, **kwargs):
        for name in ('wake.onnx', 'mel.onnx', 'embedding.onnx'):
            (root / name).write_bytes(b'local-placeholder-for-adapter-test')
        data = dict(schema_version=1, phrase='hello robot', language='en',
                    model_file='wake.onnx', melspec_model_file='mel.onnx',
                    embedding_model_file='embedding.onnx', license='test-only', verified=False)
        data.update(kwargs)
        path = root / 'manifest.json'; path.write_text(json.dumps(data))
        return path

    def test_manifest_requires_existing_local_files_and_explicit_phrase(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            path = self.manifest(root)
            manifest = load_wake_manifest(path)
            self.assertEqual(manifest['phrase'], 'hello robot')
            self.assertTrue(Path(manifest['model_file']).is_absolute())
            (root / 'wake.onnx').unlink()
            with self.assertRaises(ConfigurationRequired): load_wake_manifest(path)
            path = self.manifest(root, phrase='')
            with self.assertRaises(ConfigurationRequired): load_wake_manifest(path)

    def test_russian_release_requires_verified_russian_manifest(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            with self.assertRaises(ConfigurationRequired):
                load_wake_manifest(self.manifest(root), require_russian=True)
            path = self.manifest(root, phrase='эй робот', language='ru', verified=False)
            with self.assertRaises(ConfigurationRequired): load_wake_manifest(path, require_russian=True)
            path = self.manifest(root, phrase='эй робот', language='ru', verified=True,
                                 evaluation_report='report.json')
            import hashlib
            hashes = {key: hashlib.sha256((root / name).read_bytes()).hexdigest() for key, name in (
                ('model_file', 'wake.onnx'), ('melspec_model_file', 'mel.onnx'),
                ('embedding_model_file', 'embedding.onnx'))}
            (root / 'report.json').write_text(json.dumps({'accepted': True, 'split': 'test',
                'phrase': 'эй робот', 'language': 'ru', 'threshold': .5, 'artifacts_sha256': hashes}))
            self.assertEqual(load_wake_manifest(path, require_russian=True)['language'], 'ru')

    def test_backend_passes_all_local_paths_without_downloading(self):
        with tempfile.TemporaryDirectory() as d:
            manifest = load_wake_manifest(self.manifest(Path(d)))
            fake = SimpleNamespace(predict=lambda a: {'wake': .8}, reset=lambda: None)
            with patch.dict(sys.modules, {'openwakeword.model': SimpleNamespace(Model=lambda **kw: self.make_model(kw, fake))}):
                backend = OpenWakeWordBackend(manifest)
                self.assertAlmostEqual(backend.predict(b'\x00\x00' * 1280), .8)
                backend.reset()

    def make_model(self, kwargs, fake):
        self.assertEqual(kwargs['inference_framework'], 'onnx')
        self.assertEqual(kwargs['vad_threshold'], 0)
        for key in ('melspec_model_path', 'embedding_model_path'):
            self.assertTrue(Path(kwargs[key]).is_file())
        self.assertEqual(len(kwargs['wakeword_models']), 1)
        return fake


if __name__ == '__main__': unittest.main()
