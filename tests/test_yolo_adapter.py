#!/usr/bin/env python3
# FEATURE: YOLOE-26 JSON-lines adapter with an SSH launcher and offline GPU worker.
# SPEC: bounded strict JSON/JPEG, original-image normalized boxes, explicit init,
#       no runtime downloads, no camera access, stdout only protocol, EOF cleanup.
# IMPLEMENTATION: stdlib exec launcher, lazy injected worker, opt-in wheel setup.
# RESULT: IMPLEMENTED — 16 adapter tests and 328 full Python tests pass offline.
# STATUS: DONE
import base64
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / 'tools/yolo/worker.py'
ADAPTER = ROOT / 'tools/yolo/adapter'


class FakeImage:
    size = (200, 100)
    def close(self):
        self.closed = True


class FakeModel:
    def predict(self, image, **kwargs):
        self.kwargs = kwargs
        boxes = type('Boxes', (), {'xyxy': [[20, 10, 180, 90], [-10, 0, 300, 100]],
                                  'conf': [0.9, 0.8], 'cls': [0, 1]})()
        return [type('Result', (), {'orig_shape': (100, 200),
                                   'names': {0: 'chair', 1: 'table'}, 'boxes': boxes})()]


class YoloAdapterTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(WORKER.is_file(), 'GPU worker must exist')
        spec = importlib.util.spec_from_file_location('yolo_worker', WORKER)
        self.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mod)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.weights = self.dir / 'yoloe-26l-seg.pt'
        self.weights.write_bytes(b'prepared fake weights')
        (self.dir / 'mobileclip2_b.ts').write_bytes(b'prepared fake encoder')
        self.config = self.mod.Config.from_env({'YOLO_MODEL': str(self.weights)})
        self.image = FakeImage()
        self.model = FakeModel()
        self.loads = []
        def load(config):
            self.loads.append(config)
            print('library initialization noise')
            return self.model
        self.worker = self.mod.Worker(self.config, load_model=load,
                                      decode_image=lambda data: self.image)
        self.addCleanup(self.worker.close)

    def request(self, **extra):
        return {'op': 'detect', 'frame_id': 18446744073709551615,
                'jpeg': base64.b64encode(b'fake JPEG').decode(), **extra}

    def init(self):
        return self.worker.handle({'op': 'init'})

    def test_init_is_lazy_and_idempotent(self):
        self.assertEqual(self.loads, [])
        self.assertEqual(self.init(), {'ok': True, 'model': 'yoloe-26l-seg.pt'})
        self.init()
        self.assertEqual(len(self.loads), 1)

    def test_detect_requires_init(self):
        self.assertFalse(self.worker.handle(self.request())['ok'])
        self.assertEqual(self.loads, [])

    def test_original_coordinates_names_id_and_cleanup(self):
        self.init()
        result = self.worker.handle(self.request())
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['frame_id'], 18446744073709551615)
        self.assertEqual((result['width'], result['height']), (200, 100))
        self.assertEqual(result['objects'][0], {'label': 'chair', 'confidence': .9,
                                              'box': [.1, .1, .9, .9]})
        self.assertEqual(result['objects'][1]['box'], [0, 0, 1, 1])
        self.assertTrue(math.isfinite(result['inference_ms']))
        self.assertEqual(self.model.kwargs['max_det'], 100)
        self.assertFalse(self.model.kwargs['verbose'])
        self.assertTrue(self.image.closed)

    def test_empty_detections(self):
        self.init()
        self.model.predict = lambda *a, **k: [type('Result', (), {
            'orig_shape': (100, 200), 'names': {}, 'boxes': None})()]
        self.assertEqual(self.worker.handle(self.request())['objects'], [])

    def test_invalid_requests(self):
        self.init()
        for request in [None, [], {}, {'op': 'no'}, self.request(frame_id=True),
                        self.request(frame_id=-1), self.request(frame_id=2**64),
                        self.request(frame_id=1.5), self.request(jpeg='!bad!'),
                        self.request(jpeg=''), self.request(jpeg=123),
                        self.request(jpeg=base64.b64encode(b'x'*(2*1024*1024+1)).decode())]:
            with self.subTest(request=str(request)[:80]):
                self.assertFalse(self.worker.handle(request)['ok'])

    def test_bad_decode_and_dimensions(self):
        self.init()
        self.worker.decode_image = lambda _: (_ for _ in ()).throw(ValueError('bad JPEG'))
        self.assertFalse(self.worker.handle(self.request())['ok'])
        self.worker.decode_image = lambda _: type('Image', (), {
            'size': (8193, 2), 'close': lambda s: None})()
        self.assertFalse(self.worker.handle(self.request())['ok'])

    def test_nonfinite_and_too_many_model_results(self):
        self.init()
        self.model.predict = lambda *a, **k: [type('Result', (), {
            'orig_shape': (100, 200), 'names': ['chair'], 'boxes': type('Boxes', (), {
                'xyxy': [[0, 0, 10, 10]]*102, 'conf': [.8]*101+[float('nan')],
                'cls': [0]*102})()})()]
        reply = self.worker.handle(self.request())
        self.assertTrue(reply['ok'], reply)
        self.assertEqual(len(reply['objects']), 100)
        json.dumps(reply, allow_nan=False)
        self.model.predict = lambda *a, **k: [type('Result', (), {
            'orig_shape': (100, 200), 'names': ['chair'], 'boxes': type('Boxes', (), {
                'xyxy': [[0, 0, float('nan'), 10]], 'conf': [.8], 'cls': [0]})()})()]
        self.assertFalse(self.worker.handle(self.request())['ok'])

    def test_model_readiness_and_config(self):
        self.assertEqual(self.config.device, '0')
        self.assertIn('chair', self.config.classes)
        self.assertEqual(self.config.model.name, 'yoloe-26l-seg.pt')
        for env in [{'YOLO_CLASSES': 'no'}, {'YOLO_CLASSES': '[]'},
                    {'YOLO_CLASSES': '[" "]'}, {'YOLO_CLASSES': '[1]'},
                    {'YOLO_CONF': 'nan'}, {'YOLO_CONF': '1.1'}]:
            with self.subTest(env=env), self.assertRaises(ValueError):
                self.mod.Config.from_env(env)
        cfg = self.mod.Config.from_env({'YOLO_MODEL': str(self.dir/'missing.pt')})
        missing = self.mod.Worker(cfg, load_model=lambda c: self.fail('must not load'))
        self.assertFalse(missing.handle({'op': 'init'})['ok'])
        (self.dir/'mobileclip2_b.ts').unlink()
        self.assertFalse(self.init()['ok'])

    def test_stdio_strict_json_stdout_purity_and_eof(self):
        source = io.BytesIO(b'not JSON\n{"op":"init","bad":NaN}\n{"op":"init"}\n' +
                            json.dumps(self.request()).encode()+b'\n')
        output = io.StringIO()
        self.mod.serve(self.worker, source, output)
        replies = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(len(replies), 4)
        self.assertEqual([r['ok'] for r in replies], [False, False, True, True])
        self.assertIsNone(self.worker.model)

    def test_oversized_line_is_bounded_and_following_request_survives(self):
        source = io.BytesIO(b'x'*(3*1024*1024+1)+b'\n{"op":"init"}\n')
        output = io.StringIO()
        self.mod.serve(self.worker, source, output)
        replies = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual([r['ok'] for r in replies], [False, True])

    def test_backend_uses_documented_text_prompt_api_and_local_assets(self):
        calls = []
        def original_download(*args, **kwargs):
            self.fail('runtime must not invoke the asset downloader')
        downloads = types.ModuleType('ultralytics.utils.downloads')
        downloads.attempt_download_asset = original_download
        ultralytics = types.ModuleType('ultralytics')
        utils = types.ModuleType('ultralytics.utils')
        utils.downloads = downloads
        torch = types.ModuleType('torch')
        torch.set_num_threads = lambda n: calls.append(('threads', n))
        class Model:
            def __init__(s, path, **kwargs):
                calls.append(('load', path, kwargs))
            def to(s, device):
                calls.append(('device', device))
            def get_text_pe(s, classes):
                calls.append(('text', classes))
                self.assertEqual(downloads.attempt_download_asset('mobileclip2_b.ts'),
                                 str(self.dir/'mobileclip2_b.ts'))
                with self.assertRaises(FileNotFoundError):
                    downloads.attempt_download_asset('absent.ts')
                return 'embeddings'
            def set_classes(s, classes, embeddings):
                calls.append(('classes', classes, embeddings))
        ultralytics.YOLOE = Model
        with patch.dict('sys.modules', {'torch': torch, 'ultralytics': ultralytics,
                        'ultralytics.utils': utils,
                        'ultralytics.utils.downloads': downloads}), patch.dict(os.environ):
            result = self.mod.load_yolo(self.config)
            self.assertIsInstance(result, Model)
            self.assertEqual(os.environ['YOLO_AUTOINSTALL'], 'false')
            self.assertEqual(os.environ['YOLO_OFFLINE'], 'true')
        self.assertIs(downloads.attempt_download_asset, original_download)
        self.assertIn(('device', 'cuda:0'), calls)
        self.assertIn(('text', self.config.classes), calls)
        self.assertIn(('classes', self.config.classes, 'embeddings'), calls)
        self.assertIn(('load', str(self.weights), {'task': 'segment', 'verbose': False}), calls)

    def test_real_decoder_rejects_non_jpeg_without_heavy_imports(self):
        # Format gate runs before importing Pillow/Torch/Ultralytics.
        with self.assertRaises(ValueError):
            self.mod.decode_jpeg(b'not jpeg')


class PreparationTests(unittest.TestCase):
    def test_setup_is_explicit_and_wheel_only(self):
        setup = ROOT / 'tools/yolo/setup_gpu.sh'
        self.assertTrue(setup.is_file(), 'explicit GPU preparation script must exist')
        result = subprocess.run(['bash', str(setup)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--confirm', result.stderr)
        self.assertIn('--only-binary=:all:', setup.read_text())
        requirements = (ROOT / 'tools/yolo/requirements.txt').read_text()
        self.assertIn('ultralytics==8.4.191', requirements)
        self.assertIn('torch==2.10.0', requirements)
        self.assertIn('torchvision==0.25.0', requirements)
        self.assertIn('clip-anytorch==2.6.0', requirements)


class LauncherTests(unittest.TestCase):
    def test_local_eof_and_malformed_requests_need_no_gpu(self):
        result = subprocess.run([str(ADAPTER)], input=b'bad json\n', capture_output=True,
                                env={**os.environ, 'YOLO_LOCAL': '1'}, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)['ok'])

    def test_help_has_no_gpu_dependencies(self):
        self.assertTrue(ADAPTER.is_file(), 'host launcher must exist')
        result = subprocess.run([str(ADAPTER), '--help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('YOLO_SSH_HOST', result.stdout)

    def test_ssh_exec_quoting_config_and_eof(self):
        self.assertTrue(ADAPTER.is_file(), 'host launcher must exist')
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp)/'ssh'
            fake.write_text('#!/usr/bin/env python3\nimport sys,json\nprint(json.dumps(sys.argv[1:]))\nprint(sys.stdin.read(),end="")\n')
            fake.chmod(0o755)
            env = {**os.environ, 'PATH': tmp+os.pathsep+os.environ['PATH'],
                   'YOLO_SSH_HOST': 'gpu', 'YOLO_REMOTE_DIR': "robot yolo/it's",
                   'YOLO_CLASSES': '["chair", "table"]', 'YOLO_CONF': '.3'}
            env.pop('YOLO_LOCAL', None)
            result = subprocess.run([str(ADAPTER)], input='{"op":"init"}\n',
                                    capture_output=True, text=True, env=env, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            lines = result.stdout.splitlines()
            args = json.loads(lines[0])
            self.assertIn('gpu', args)
            self.assertIn('BatchMode=yes', args)
            self.assertIn('worker.py', args[-1])
            self.assertIn('YOLO_CLASSES=', args[-1])
            self.assertIn('"$HOME"/', args[-1])
            self.assertEqual(lines[1], '{"op":"init"}')


if __name__ == '__main__':
    unittest.main()
