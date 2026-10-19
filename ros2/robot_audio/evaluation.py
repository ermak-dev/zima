"""Local wake-model evaluation; threshold selection never reads held-out test scores."""
import hashlib
import json
import math
from pathlib import Path
import statistics
import wave

from .wake import _finite


def load_dataset(manifest_path):
    manifest_path = Path(manifest_path).expanduser().resolve()
    if manifest_path.stat().st_size > 2 * 1024 * 1024:
        raise ValueError('dataset manifest too large')
    data = json.loads(manifest_path.read_text())
    if data.get('schema_version') != 1:
        raise ValueError('unsupported dataset schema')
    for key in ('phrase', 'language'):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f'explicit {key} is required')
    records = data.get('records')
    if not isinstance(records, list) or not records or len(records) > 10000:
        raise ValueError('invalid dataset records')
    criteria = data.get('criteria')
    if not isinstance(criteria, dict):
        raise ValueError('explicit acceptance criteria are required')
    for key, high in (('max_false_activations_per_hour', None), ('max_miss_rate', 1), ('max_latency_s', 5)):
        _finite(criteria.get(key), key, 0, high)
    seen_ids = set()
    groups = {'recording_id': {}, 'speaker_id': {}, 'path': {}, 'content': {}}
    normalized = []
    for record in records:
        if not isinstance(record, dict):
            raise ValueError('invalid record')
        item = dict(record)
        for key in ('id', 'recording_id', 'speaker_id', 'path', 'split', 'condition'):
            if not isinstance(item.get(key), str) or not item[key].strip():
                raise ValueError(f'record requires {key}')
        if item['id'] in seen_ids or item['split'] not in ('train', 'validation', 'test'):
            raise ValueError('duplicate record id or invalid split')
        seen_ids.add(item['id'])
        path = (manifest_path.parent / item['path']).resolve()
        if not path.is_relative_to(manifest_path.parent) or not path.is_file():
            raise ValueError('record must reference a local file inside the dataset directory')
        if path.stat().st_size > 256 * 1024 * 1024:
            raise ValueError('recording too large')
        try:
            with wave.open(str(path), 'rb') as reader:
                if (reader.getnchannels(), reader.getsampwidth(), reader.getframerate(), reader.getcomptype()) != (1, 2, 16000, 'NONE'):
                    raise ValueError('dataset requires uncompressed mono 16 kHz S16LE WAV')
                duration = reader.getnframes() / reader.getframerate()
        except (wave.Error, EOFError) as error:
            raise ValueError('invalid recording') from error
        if not 0 < duration <= 3600:
            raise ValueError('record duration outside limits')
        content_hash = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(65536), b''):
                content_hash.update(block)
        group_values = {'recording_id': item['recording_id'], 'speaker_id': item['speaker_id'],
                        'path': str(path), 'content': content_hash.hexdigest()}
        for key, value in group_values.items():
            old_split = groups[key].setdefault(value, item['split'])
            if old_split != item['split']:
                raise ValueError(f'{key} leaks across dataset splits')
        events = item.get('events')
        if not isinstance(events, list):
            raise ValueError('record requires explicit event annotations')
        intervals = []
        for event in events:
            if not isinstance(event, dict):
                raise ValueError('invalid annotation')
            start = _finite(event.get('start'), 'event start', 0, duration)
            end = _finite(event.get('end'), 'event end', 0, duration)
            if end <= start or (intervals and start < intervals[-1][1]):
                raise ValueError('annotations must be ordered, disjoint intervals')
            intervals.append((start, end))
        item.update(path=str(path), duration_s=duration, intervals=intervals)
        normalized.append(item)
    result = dict(data)
    result['records'] = normalized
    result['manifest_path'] = str(manifest_path)
    return result


def score_detections(expected, detections, *, duration_s, max_latency_s=.5):
    duration_s = _finite(duration_s, 'record duration', .000001)
    max_latency_s = _finite(max_latency_s, 'maximum latency', 0, 5)
    intervals = []
    for start, end in expected:
        start = _finite(start, 'annotation start', 0, duration_s)
        end = _finite(end, 'annotation end', 0, duration_s)
        if end <= start or (intervals and start < intervals[-1][1]):
            raise ValueError('annotations must be ordered disjoint intervals')
        intervals.append((start, end))
    times = sorted(_finite(t, 'activation time', 0, duration_s) for t in detections)
    matched = set()
    latencies = []
    false = 0
    for detection in times:
        index = next((i for i, (start, end) in enumerate(intervals)
                      if i not in matched and start <= detection <= min(duration_s, end + max_latency_s)), None)
        if index is None:
            false += 1
        else:
            matched.add(index)
            latencies.append(max(0, detection - intervals[index][1]))
    latencies.sort()
    return {'true_positives': len(matched), 'misses': len(intervals) - len(matched),
            'expected': len(intervals), 'false_activations': false,
            'duration_s': duration_s, 'false_activations_per_hour': false * 3600 / duration_s,
            'miss_rate': (len(intervals) - len(matched)) / len(intervals) if intervals else None,
            'latency_p95_s': latencies[max(0, math.ceil(.95 * len(latencies)) - 1)] if latencies else None,
            'latencies_s': latencies}


def _aggregate(records):
    duration = sum(record['duration_s'] for record in records)
    expected = sum(record['expected'] for record in records)
    misses = sum(record['misses'] for record in records)
    false = sum(record['false_activations'] for record in records)
    latencies = sorted(value for record in records for value in record['latencies_s'])
    return {'duration_s': duration, 'expected': expected, 'misses': misses,
            'true_positives': expected - misses, 'false_activations': false,
            'false_activations_per_hour': false * 3600 / duration if duration else None,
            'miss_rate': misses / expected if expected else None,
            'latency_p95_s': latencies[max(0, math.ceil(.95 * len(latencies)) - 1)] if latencies else None}


def evaluate_dataset(manifest_path, run_record, *, threshold, split='test'):
    threshold = _finite(threshold, 'wake threshold', 0, 1)
    if split not in ('validation', 'test'):
        raise ValueError('evaluation requires validation or held-out test split')
    dataset = load_dataset(manifest_path)
    records = []
    for record in dataset['records']:
        if record['split'] != split:
            continue
        times = run_record(record, threshold)
        metrics = score_detections(record['intervals'], times, duration_s=record['duration_s'],
                                   max_latency_s=dataset['criteria']['max_latency_s'])
        metrics.update(id=record['id'], condition=record['condition'])
        records.append(metrics)
    if not records:
        raise ValueError(f'no {split} recordings')
    aggregate = _aggregate(records)
    criteria = dataset['criteria']
    accepted = (aggregate['expected'] > 0 and aggregate['miss_rate'] is not None and
                aggregate['miss_rate'] <= criteria['max_miss_rate'] and
                aggregate['false_activations_per_hour'] <= criteria['max_false_activations_per_hour'] and
                aggregate['latency_p95_s'] is not None and
                aggregate['latency_p95_s'] <= criteria['max_latency_s'])
    by_condition = {condition: _aggregate([record for record in records if record['condition'] == condition])
                    for condition in sorted({record['condition'] for record in records})}
    return {'schema_version': 1, 'phrase': dataset['phrase'], 'language': dataset['language'],
            'split': split, 'threshold': threshold, 'criteria': criteria, 'metrics': aggregate,
            'by_condition': by_condition, 'records': records, 'accepted': bool(accepted)}


def calibrate_threshold(manifest_path, run_record, *, thresholds):
    thresholds = list(thresholds)
    if not thresholds or len(thresholds) > 100:
        raise ValueError('provide a bounded validation threshold grid')
    reports = [evaluate_dataset(manifest_path, run_record, threshold=t, split='validation') for t in thresholds]
    candidates = [report for report in reports if report['accepted']]
    if not candidates:
        return {'threshold': None, 'accepted': False, 'split': 'validation', 'reports': reports}
    best = min(candidates, key=lambda report: (report['metrics']['miss_rate'],
               report['metrics']['false_activations_per_hour'], report['metrics']['latency_p95_s']))
    return {'threshold': best['threshold'], 'accepted': True, 'split': 'validation', 'reports': reports}


def wav_record_runner(backend_factory, *, phrase, model='local', debounce_s=1.5):
    from .types import AudioChunk
    from .wake import WakeDetector

    def run(record, threshold):
        backend = backend_factory()
        detector = WakeDetector(backend, phrase=phrase, model=model,
                                threshold=threshold, debounce_s=debounce_s)
        detector.reset()
        result = []
        with wave.open(record['path'], 'rb') as reader:
            sequence = 0
            sample = 0
            while True:
                pcm = reader.readframes(320)
                if not pcm:
                    break
                chunk = AudioChunk(record['id'], sequence, int(sample / 16000 * 1e9), pcm)
                result.extend(event.stamp_ns / 1e9 for event in detector.feed(chunk))
                sample += len(pcm) // 2
                sequence += 1
        return result
    return run
