"""Local openWakeWord-compatible heads; training never grants deployment verification."""
import json
import os
from pathlib import Path
import shutil
import unicodedata
import wave

from .evaluation import load_dataset
from .phrases import PhraseSpec


def label_windows(end_times, intervals, *, lead_s=.08, tail_s=.32):
    import numpy as np
    if not 0 <= lead_s <= .5 or not 0 <= tail_s <= 1:
        raise ValueError('invalid positive-window bounds')
    ends = np.asarray(end_times, dtype=float)
    if not np.isfinite(ends).all() or (ends < 0).any():
        raise ValueError('invalid window timestamps')
    return np.asarray([int(any(end - lead_s - 1e-9 <= t <= end + tail_s + 1e-9
                              for start, end in intervals)) for t in ends], dtype='int64')


def fit_head(features, labels, *, kind='logistic', seed=17):
    import numpy as np
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    from sklearn.linear_model import LogisticRegression
    from sklearn.neural_network import MLPClassifier
    from threadpoolctl import threadpool_limits
    x = np.asarray(features, dtype='float32')
    y = np.asarray(labels)
    if (x.ndim != 3 or x.shape[2] != 96 or not 1 <= x.shape[1] <= 32 or
            not 2 <= len(x) <= 50000 or y.shape != (len(x),) or
            not np.isfinite(x).all() or set(np.unique(y)) != {0, 1}):
        raise ValueError('training requires bounded finite windows and both binary classes')
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError('invalid training seed')
    x = x.reshape(len(x), -1)
    if kind == 'logistic':
        classifier = LogisticRegression(C=1, class_weight='balanced', max_iter=300, random_state=seed)
    elif kind == 'mlp':
        rng = np.random.default_rng(seed)
        indices = [np.flatnonzero(y == value) for value in (0, 1)]
        count = max(map(len, indices))
        selected = np.concatenate([rng.choice(group, count, replace=len(group) < count) for group in indices])
        rng.shuffle(selected)
        x, y = x[selected], y[selected]
        classifier = MLPClassifier(hidden_layer_sizes=(32,), alpha=.01, max_iter=500,
                                   early_stopping=True, random_state=seed)
    else:
        raise ValueError('head kind must be logistic or mlp')
    head = make_pipeline(StandardScaler(), classifier)
    with threadpool_limits(limits=2):
        head.fit(x, y)
    return head


def export_head(head, path, *, window_frames=16):
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    if type(window_frames) is not int or not 1 <= window_frames <= 32:
        raise ValueError('invalid feature-window length')
    scaler, classifier = head.steps[0][1], head.steps[-1][1]
    if len(scaler.mean_) != window_frames * 96:
        raise ValueError('classifier feature shape does not match the window')
    if hasattr(classifier, 'coef_'):
        weights = [classifier.coef_.T]
        biases = [classifier.intercept_]
    else:
        weights, biases = classifier.coefs_, classifier.intercepts_
    weights = [np.asarray(value, dtype='float64').copy() for value in weights]
    biases = [np.asarray(value, dtype='float64').copy() for value in biases]
    weights[0] /= scaler.scale_[:, None]
    biases[0] -= scaler.mean_ @ weights[0]
    initializers = []
    nodes = [helper.make_node('Flatten', ['features'], ['flat'], axis=1)]
    previous = 'flat'
    for index, (weight, bias) in enumerate(zip(weights, biases)):
        w, b, linear, output = f'w{index}', f'b{index}', f'linear{index}', f'out{index}'
        initializers.extend([numpy_helper.from_array(weight.astype('float32'), w),
                             numpy_helper.from_array(bias.astype('float32'), b)])
        nodes.append(helper.make_node('Gemm', [previous, w, b], [linear]))
        nodes.append(helper.make_node('Sigmoid' if index == len(weights)-1 else 'Relu', [linear], [output]))
        previous = output
    graph = helper.make_graph(nodes, 'local_wake_head',
        [helper.make_tensor_value_info('features', TensorProto.FLOAT, ['batch', window_frames, 96])],
        [helper.make_tensor_value_info(previous, TensorProto.FLOAT, ['batch', 1])], initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    fd = os.open(Path(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as file:
        file.write(model.SerializeToString())


def _pronunciation(value):
    return unicodedata.normalize('NFC', value).casefold().strip()


def validate_training_pronunciations(dataset):
    if 'pronunciation' not in dataset:
        return None
    spec = PhraseSpec(dataset['phrase'], dataset['pronunciation'],
                      dataset.get('negative_pronunciations', ()), dataset['language'])
    target = _pronunciation(spec.pronunciation)
    negatives = {_pronunciation(value) for value in spec.negative_pronunciations}
    for record in dataset['records']:
        spoken = record.get('spoken_pronunciation')
        if record['intervals']:
            if (not isinstance(spoken, str) or _pronunciation(spoken) != target or
                    record.get('pronunciation_verified') is not True):
                raise ValueError('positive recordings require verified target pronunciation')
        elif spoken is not None and _pronunciation(spoken) == target:
            raise ValueError('target pronunciation cannot be labelled as a negative recording')
    return spec


def train_model(dataset_path, features_dir, output_dir, *, window_frames=16, kind='logistic',
                seed=17, feature_factory=None):
    import numpy as np
    dataset = load_dataset(dataset_path)
    spec = validate_training_pronunciations(dataset)
    if type(window_frames) is not int or not 1 <= window_frames <= 32:
        raise ValueError('invalid feature window')
    records = [record for record in dataset['records'] if record['split'] == 'train']
    if not records or len(records) > 500 or not any(not r['intervals'] for r in records):
        raise ValueError('training needs bounded train records including negative recordings')
    source = Path(features_dir).expanduser().resolve()
    for filename in ('melspectrogram.onnx', 'embedding_model.onnx'):
        path = source / filename
        if not path.is_file() or path.is_symlink() or not 0 < path.stat().st_size <= 32*1024*1024:
            raise ValueError('explicit bounded local feature models are required')
    if feature_factory is None:
        from openwakeword.utils import AudioFeatures
        feature_factory = lambda: AudioFeatures(melspec_model_path=str(source/'melspectrogram.onnx'),
            embedding_model_path=str(source/'embedding_model.onnx'), inference_framework='onnx', ncpu=1)
    features, labels = [], []
    for record in records:
        extractor = feature_factory()
        windows, ends = [], []
        with wave.open(record['path'], 'rb') as reader:
            if reader.getnframes() > 16000 * 120:
                raise ValueError('training recording exceeds 120 seconds')
            sample = 0
            while True:
                pcm = reader.readframes(1280)
                if len(pcm) != 2560:
                    break
                extractor(np.frombuffer(pcm, dtype='<i2').copy())
                value = np.asarray(extractor.get_features(window_frames), dtype='float32')
                if value.shape != (1, window_frames, 96) or not np.isfinite(value).all():
                    raise ValueError('feature model produced invalid windows')
                windows.append(value[0].copy())
                sample += 1280
                ends.append(sample / 16000)
        features.extend(windows)
        labels.extend(label_windows(ends, record['intervals']).tolist())
        if len(features) > 50000:
            raise ValueError('training window budget exceeded')
    head = fit_head(np.asarray(features), np.asarray(labels), kind=kind, seed=seed)
    destination = Path(output_dir).expanduser()
    destination.mkdir(mode=0o700, parents=True, exist_ok=False)
    export_head(head, destination/'wake.onnx', window_frames=window_frames)
    for filename in ('melspectrogram.onnx', 'embedding_model.onnx'):
        shutil.copyfile(source/filename, destination/filename)
        (destination/filename).chmod(0o600)
    manifest = {'schema_version': 1, 'phrase': dataset['phrase'], 'language': dataset['language'],
        'model_file': 'wake.onnx', 'melspec_model_file': 'melspectrogram.onnx',
        'embedding_model_file': 'embedding_model.onnx',
        'license': 'Private experimental classifier; upstream feature-model and dataset licenses apply',
        'verified': False, 'verification_scope': 'training_only'}
    if spec:
        manifest.update(pronunciation=spec.pronunciation, negative_pronunciations=list(spec.negative_pronunciations))
    report = {'train_records': [r['id'] for r in records],
        'train_speakers': sorted({r['speaker_id'] for r in records}),
        'windows': len(features), 'positive_windows': int(sum(labels)), 'kind': kind,
        'seed': seed, 'window_frames': window_frames, 'verified': False,
        'manifest': str(destination/'manifest.json')}
    for filename, content in (('manifest.json', manifest), ('training.json', report)):
        fd = os.open(destination/filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as file:
            json.dump(content, file, ensure_ascii=False, indent=2)
    return report
