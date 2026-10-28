#!/usr/bin/env python3
"""Bounded YOLOE-26 JSON-lines worker; heavy dependencies load only at init."""
import argparse
import base64
import contextlib
import io
import json
import math
import os
from pathlib import Path
import sys
import time
import warnings

MAX_JPEG = 2 * 1024 * 1024
MAX_LINE = 3 * 1024 * 1024
MAX_SIDE = 8192
# Limit decoded RGB memory to ~48 MiB (plus decoder/inference overhead).
MAX_PIXELS = 16 * 1024 * 1024
MAX_OBJECTS = 100
DEFAULT_CLASSES = [
    'person', 'chair', 'table', 'sofa', 'bed', 'cabinet', 'door', 'stairs',
    'rug', 'shoe', 'cable', 'bag', 'bottle', 'cup', 'plant', 'cat', 'dog',
]


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('non-finite model value')
    return value


class Config:
    @classmethod
    def from_env(cls, env=None):
        env = os.environ if env is None else env
        self = cls()
        self.model = Path(env.get('YOLO_MODEL', 'yoloe-26l-seg.pt')).expanduser().resolve()
        self.classes = json.loads(env.get('YOLO_CLASSES', json.dumps(DEFAULT_CLASSES)))
        if (not isinstance(self.classes, list) or not 1 <= len(self.classes) <= 100
                or any(not isinstance(s, str) or not s.strip() or len(s) > 100
                       for s in self.classes) or len(set(self.classes)) != len(self.classes)):
            raise ValueError('YOLO_CLASSES must be 1..100 distinct nonempty strings')
        self.conf = finite(env.get('YOLO_CONF', '0.25'))
        if not 0 <= self.conf <= 1:
            raise ValueError('YOLO_CONF must be in [0,1]')
        self.device = env.get('YOLO_DEVICE', '0')
        if not self.device:
            raise ValueError('YOLO_DEVICE must not be empty')
        return self

    def check_assets(self):
        for path in (self.model, self.model.parent / 'mobileclip2_b.ts'):
            if not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f'prepared local asset missing: {path}; run setup_gpu.sh explicitly')


def decode_jpeg(data):
    if not data.startswith(b'\xff\xd8\xff'):
        raise ValueError('expected a JPEG image')
    from PIL import Image
    with warnings.catch_warnings():
        warnings.simplefilter('error', Image.DecompressionBombWarning)
        with Image.open(io.BytesIO(data)) as image:
            if image.format != 'JPEG':
                raise ValueError('expected a JPEG image')
            check_size(image.size)
            image.load()
            # No EXIF transpose: boxes must refer to the original JPEG pixel grid.
            return image.convert('RGB')


def check_size(size):
    width, height = size
    if (not 0 < width <= MAX_SIDE or not 0 < height <= MAX_SIDE
            or width * height > MAX_PIXELS):
        raise ValueError('image dimensions exceed safe decode limits')


def load_yolo(config):
    # Set before imports. Missing dependencies must fail, not trigger pip/network.
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_OFFLINE'] = 'true'
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    os.environ.setdefault('MKL_NUM_THREADS', '2')
    import torch
    from ultralytics import YOLOE
    from ultralytics.utils import downloads
    torch.set_num_threads(2)

    def local_asset(file, *args, **kwargs):
        path = Path(file)
        if not path.is_absolute():
            path = config.model.parent / path
        if not path.is_file():
            raise FileNotFoundError(f'runtime downloads disabled; missing {path}')
        return str(path)

    # MobileCLIPTS imports this function at construction. All requested assets
    # must already be local; never allow its fallback downloader at runtime.
    original = downloads.attempt_download_asset
    downloads.attempt_download_asset = local_asset
    try:
        model = YOLOE(str(config.model), task='segment', verbose=False)
        model.to('cuda:' + config.device if config.device.isdigit() else config.device)
        embeddings = model.get_text_pe(config.classes)
        model.set_classes(config.classes, embeddings)
        return model
    finally:
        downloads.attempt_download_asset = original


class Worker:
    def __init__(self, config, *, load_model=load_yolo, decode_image=decode_jpeg):
        self.config = config
        self.load_model = load_model
        self.decode_image = decode_image
        self.model = None

    def close(self):
        self.model = None

    def handle(self, request):
        # Some dependencies print even when verbose=False. Keep protocol clean.
        with contextlib.redirect_stdout(sys.stderr):
            try:
                if not isinstance(request, dict):
                    raise ValueError('request must be an object')
                if request.get('op') == 'init':
                    self.config.check_assets()
                    if self.model is None:
                        self.model = self.load_model(self.config)
                    return {'ok': True, 'model': self.config.model.name}
                if request.get('op') != 'detect':
                    raise ValueError('unknown op; expected init or detect')
                if self.model is None:
                    raise ValueError('model not initialized; send init first')
                return self.detect(request)
            except Exception as exc:
                # Never echo the request (JPEG), credentials, or arbitrary backend
                # exception text. The exception type is enough for offline triage.
                if isinstance(exc, (ValueError, FileNotFoundError)):
                    message = str(exc)[:400]
                else:
                    message = f'worker failure: {type(exc).__name__}; see GPU environment'
                return {'ok': False, 'error': message}

    def detect(self, request):
        frame_id = request.get('frame_id')
        if type(frame_id) is not int or not 0 <= frame_id < 2**64:
            raise ValueError('frame_id must be a u64')
        encoded = request.get('jpeg')
        if not isinstance(encoded, str) or not encoded or len(encoded) > ((MAX_JPEG + 2)//3)*4:
            raise ValueError('jpeg must be nonempty base64, at most 2 MiB decoded')
        try:
            data = base64.b64decode(encoded, validate=True)
        except (ValueError, UnicodeError) as exc:
            raise ValueError('invalid JPEG base64') from exc
        if not data or len(data) > MAX_JPEG:
            raise ValueError('JPEG exceeds 2 MiB limit or is empty')
        image = self.decode_image(data)
        try:
            check_size(image.size)
            width, height = image.size
            start = time.perf_counter()
            results = self.model.predict(image, conf=self.config.conf,
                                         device=self.config.device, max_det=MAX_OBJECTS,
                                         verbose=False, save=False, stream=False)
            elapsed = (time.perf_counter() - start) * 1000
            result = results[0]
            if tuple(result.orig_shape) != (height, width):
                raise ValueError('model result does not match original image dimensions')
            objects = []
            if result.boxes is not None:
                boxes = result.boxes
                # Convert each bounded scalar; works for CUDA tensors as well as fakes.
                for xyxy, score, cls in zip(boxes.xyxy, boxes.conf, boxes.cls):
                    if len(objects) == MAX_OBJECTS:
                        break
                    confidence = finite(score)
                    class_value = finite(cls)
                    if class_value != int(class_value) or not 0 <= confidence <= 1:
                        raise ValueError('invalid model class/confidence')
                    label = result.names[int(class_value)]
                    if not isinstance(label, str) or not label or len(label) > 100:
                        raise ValueError('invalid model class name')
                    box = [min(1.0, max(0.0, finite(v)/scale))
                           for v, scale in zip(xyxy, (width, height, width, height))]
                    if len(box) != 4 or box[0] > box[2] or box[1] > box[3]:
                        raise ValueError('invalid model box')
                    objects.append({'label': label, 'confidence': confidence, 'box': box})
            return {'ok': True, 'frame_id': frame_id, 'width': width, 'height': height,
                    'objects': objects, 'inference_ms': elapsed}
        finally:
            image.close()


def reject_constant(value):
    raise ValueError('non-finite JSON constant')


def serve(worker, source, output):
    try:
        while True:
            line = source.readline(MAX_LINE + 1)
            if not line:
                break
            if len(line) > MAX_LINE:
                # Drain this record in bounded chunks before accepting the next.
                while not line.endswith(b'\n'):
                    line = source.readline(MAX_LINE + 1)
                    if not line:
                        break
                reply = {'ok': False, 'error': 'JSON line exceeds 3 MiB limit'}
            else:
                try:
                    request = json.loads(line.decode('utf-8'), parse_constant=reject_constant)
                    reply = worker.handle(request)
                except (ValueError, UnicodeError, RecursionError):
                    reply = {'ok': False, 'error': 'invalid JSON request'}
            output.write(json.dumps(reply, allow_nan=False, separators=(',', ':')) + '\n')
            output.flush()
    finally:
        worker.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stdin', action='store_true', required=True,
                        help='read JSON lines on stdin; write only JSON lines on stdout')
    parser.parse_args()
    # Preserve a protocol-only descriptor, then send even native library stdout
    # writes to stderr. No subprocesses or reader threads survive EOF.
    with os.fdopen(os.dup(sys.stdout.fileno()), 'w', buffering=1) as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        try:
            worker = Worker(Config.from_env())
            serve(worker, sys.stdin.buffer, output)
        except (ValueError, OSError) as exc:
            output.write(json.dumps({'ok': False, 'error': str(exc)[:400]}, allow_nan=False)+'\n')
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
