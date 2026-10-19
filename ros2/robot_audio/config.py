"""Explicit audio configuration; loading it never contacts a device or provider."""
import copy
import json
import math
from pathlib import Path
import re
from urllib.parse import urlsplit

_DEFAULTS = {
    'input': {'enabled': False, 'url': 'http://cam/audio.opus',
              'netrc': '~/.config/robot/camera.netrc', 'microphone_verified': False,
              'chunk_ms': 20, 'queue_size': 10, 'timeout_s': 3.0},
    'output': {'enabled': False, 'host': 'robot', 'helper': '/tmp/robot-audio/robot-audio',
               'asset_dir': '~/.cache/robot/audio', 'volume': .35, 'max_volume': .7,
               'max_duration_s': 30.0, 'queue_size': 8, 'muted': False,
               'quiet_hours': [], 'timezone': 'UTC'},
    'tts': {'enabled': False, 'allow_external': False, 'model': '', 'voice': '',
            'api_key_file': '~/.config/robot/openrouter.key', 'env_file': '',
            'max_output_tokens': 512, 'audio_format': 'wav', 'pcm_sample_rate': 24000,
            'require_verbatim': True, 'timeout_s': 30.0,
            'max_text_chars': 2000, 'max_audio_bytes': 4_000_000,
            'cache_ttl_s': 3600.0},
    'wake': {'enabled': False, 'manifest': '', 'threshold': .5,
             'refractory_s': 2.0, 'preroll_s': 1.5},
    'speech': {'enabled': False, 'asr_model_path': '', 'language': 'ru',
               'mode': 'half_duplex', 'max_command_s': 15.0,
               'end_silence_s': .8, 'listen_timeout_s': 5.0,
               'asr_timeout_s': 30.0, 'echo_tail_s': .5, 'vad_threshold_dbfs': -40.0},
    'echo': {'enabled': False, 'verified': False, 'delay_ms': 0,
             'max_drift_ms': 40, 'filter_ms': 200},
}


def default_config():
    return copy.deepcopy(_DEFAULTS)


def _number(value, low, high, name, integer=False):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} is outside its permitted range')
    if integer and type(value) is not int:
        raise ValueError(f'{name} must be an integer')


def validate_config(patch):
    if not isinstance(patch, dict):
        raise ValueError('audio configuration must be an object')
    config = default_config()
    for section, values in patch.items():
        if section not in config or not isinstance(values, dict):
            raise ValueError('unknown audio configuration section')
        if set(values) - set(config[section]):
            raise ValueError(f'unknown option in audio section {section}')
        config[section].update(values)
    for section, defaults in _DEFAULTS.items():
        for key, default in defaults.items():
            value = config[section][key]
            if isinstance(default, bool) and type(value) is not bool:
                raise ValueError(f'{section}.{key} must be boolean')
            if isinstance(default, str) and (not isinstance(value, str) or '\x00' in value):
                raise ValueError(f'{section}.{key} must be a string without NUL')
    source = urlsplit(config['input']['url'])
    if source.scheme not in ('http', 'https') or not source.hostname or source.username or source.password or source.fragment:
        raise ValueError('audio URL must be HTTP(S), without credentials or a fragment')
    out = config['output']
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', out['host']):
        raise ValueError('output host must be an SSH alias or hostname')
    if not re.fullmatch(r'/[A-Za-z0-9_./-]+', out['helper']) or '..' in Path(out['helper']).parts:
        raise ValueError('output helper must be an absolute safe path')
    bounds = {
        ('input', 'chunk_ms'): (10, 80, True),
        ('input', 'queue_size'): (1, 100, True),
        ('input', 'timeout_s'): (.1, 30, False),
        ('output', 'volume'): (0, 1, False),
        ('output', 'max_volume'): (0, 1, False),
        ('output', 'max_duration_s'): (.1, 30, False),
        ('output', 'queue_size'): (1, 32, True),
        ('tts', 'timeout_s'): (.1, 120, False),
        ('tts', 'max_text_chars'): (1, 10000, True),
        ('tts', 'max_output_tokens'): (32, 4096, True),
        ('tts', 'pcm_sample_rate'): (8000, 96000, True),
        ('tts', 'max_audio_bytes'): (1024, 16_000_000, True),
        ('tts', 'cache_ttl_s'): (0, 86400, False),
        ('wake', 'threshold'): (0, 1, False),
        ('wake', 'refractory_s'): (.1, 30, False),
        ('wake', 'preroll_s'): (.08, 2, False),
        ('speech', 'max_command_s'): (.5, 30, False),
        ('speech', 'end_silence_s'): (.1, 5, False),
        ('speech', 'listen_timeout_s'): (.1, 15, False),
        ('speech', 'asr_timeout_s'): (.1, 120, False),
        ('speech', 'echo_tail_s'): (0, 5, False),
        ('speech', 'vad_threshold_dbfs'): (-100, 0, False),
        ('echo', 'delay_ms'): (0, 2000, True),
        ('echo', 'max_drift_ms'): (1, 100, True),
        ('echo', 'filter_ms'): (20, 1000, True),
    }
    for (section, key), (low, high, integer) in bounds.items():
        _number(config[section][key], low, high, f'{section}.{key}', integer)
    if config['input']['chunk_ms'] not in (10, 20, 40, 80):
        raise ValueError('capture chunks must be 10, 20, 40 or 80 milliseconds')
    if config['echo']['enabled'] and config['input']['chunk_ms'] != 20:
        raise ValueError('echo cancellation requires 20 millisecond chunks')
    if out['volume'] > out['max_volume']:
        raise ValueError('default volume exceeds output volume policy')
    tts = config['tts']
    if tts['audio_format'] not in ('wav', 'pcm16'):
        raise ValueError('TTS audio format must be wav or pcm16')
    if tts['enabled'] and (not tts['allow_external'] or not tts['model'].strip() or not tts['voice'].strip()):
        raise ValueError('TTS requires external consent and an explicit model and voice')
    if config['wake']['enabled'] and not config['wake']['manifest'].strip():
        raise ValueError('wake word requires an explicit local model manifest')
    speech = config['speech']
    if speech['language'] not in ('ru', 'en'):
        raise ValueError('speech language must be ru or en')
    if speech['enabled'] and not speech['asr_model_path'].strip():
        raise ValueError('speech recognition requires an explicit local model path')
    if speech['mode'] not in ('half_duplex', 'full_duplex'):
        raise ValueError('unknown speech duplex mode')
    if speech['mode'] == 'full_duplex' and not (config['echo']['enabled'] and config['echo']['verified']):
        raise ValueError('full duplex requires enabled, evaluated echo cancellation')
    from .policy import OutputPolicy
    OutputPolicy(muted=out['muted'], quiet_hours=out['quiet_hours'], timezone_name=out['timezone'])
    return config


def load_config(path=None):
    if path is None:
        return default_config()
    file = Path(path).expanduser()
    if file.stat().st_size > 65536:
        raise ValueError('audio configuration is too large')
    with file.open(encoding='utf-8') as handle:
        return validate_config(json.load(handle))
