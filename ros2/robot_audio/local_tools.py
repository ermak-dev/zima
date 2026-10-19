"""Local measurement commands; none opens a robot, camera or external API."""
import json
from pathlib import Path
import os
import stat
import wave

from .acoustics import analyze_wav, assess_probe


def _probe_pcm(path):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        raise ValueError('probe must be a regular WAV, not a symlink') from None
    with os.fdopen(fd, 'rb') as handle:
        info = os.fstat(handle.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 1_000_000:
            raise ValueError('probe WAV is not a bounded regular file')
        try:
            with wave.open(handle, 'rb') as source:
                if (source.getnchannels(), source.getsampwidth(), source.getframerate(), source.getcomptype()) != (1, 2, 16000, 'NONE'):
                    raise ValueError('controlled probe requires 16000 Hz mono PCM16 WAV')
                frames = source.getnframes()
                if not 0 < frames <= 480000:
                    raise ValueError('probe duration exceeds 30 seconds')
                pcm = source.readframes(frames)
                if len(pcm) != frames * 2:
                    raise ValueError('truncated probe WAV')
                return pcm
        except (wave.Error, EOFError):
            raise ValueError('invalid probe WAV') from None


def prepare_asset(path, config):
    from .playback import AssetStore, normalize_wav
    maximum = config['tts']['max_audio_bytes']
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        raise ValueError('audio import requires an accessible regular WAV file') from None
    with os.fdopen(fd, 'rb') as handle:
        metadata = os.fstat(handle.fileno())
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > maximum:
            raise ValueError('import WAV is not a bounded regular file')
        data = handle.read(maximum + 1)
    normalized = normalize_wav(data, maximum, config['output']['max_duration_s'])
    store = AssetStore(config['output']['asset_dir'], max_bytes=maximum,
                       max_duration_s=config['output']['max_duration_s'],
                       ttl_s=config['tts']['cache_ttl_s'])
    asset = store.put(normalized)
    return {'asset_id': asset.asset_id, 'duration_s': asset.duration_s,
            'sample_rate': asset.sample_rate, 'channels': 1, 'sample_format': 's16le'}


def run_local(args):
    if args.command == 'analyze':
        if args.reference:
            report = assess_probe(_probe_pcm(args.reference), _probe_pcm(args.wav))
        else:
            report = analyze_wav(args.wav)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        return 0
    if args.command == 'evaluate':
        from .evaluation import load_dataset, evaluate_dataset, calibrate_threshold, wav_record_runner
        dataset = load_dataset(args.manifest)
        from .training import validate_training_pronunciations
        phrase_spec = validate_training_pronunciations(dataset)
        model_path = getattr(args, 'wake_manifest', None)
        if not model_path:
            report = {'mode': 'dataset_validation_only', 'model_verified': False,
                      'records': len(dataset['records']), 'phrase': dataset['phrase'],
                      'language': dataset['language']}
        else:
            from .wake import load_wake_manifest, OpenWakeWordBackend, wake_artifact_hashes
            manifest = load_wake_manifest(model_path)
            if (manifest['phrase'], manifest['language']) != (dataset['phrase'], dataset['language']):
                raise ValueError('wake model phrase and language must match the dataset')
            runner = wav_record_runner(lambda: OpenWakeWordBackend(manifest), phrase=manifest['phrase'])
            if getattr(args, 'calibrate', False):
                report = calibrate_threshold(args.manifest, runner, thresholds=[.3, .4, .5, .6, .7])
            else:
                report = evaluate_dataset(args.manifest, runner, threshold=args.threshold, split=args.split)
            report['artifacts_sha256'] = wake_artifact_hashes(manifest)
        if phrase_spec:
            report.update(pronunciation=phrase_spec.pronunciation,
                          negative_pronunciations=list(phrase_spec.negative_pronunciations))
        if 'verification_scope' in dataset:
            report['verification_scope'] = dataset['verification_scope']
        if 'heldout_wrong_stress_negative' in dataset:
            report['heldout_wrong_stress_negative'] = dataset['heldout_wrong_stress_negative']
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        return 0
    raise ValueError('unsupported local audio operation')
