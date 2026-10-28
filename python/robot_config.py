"""Private, validated shared configuration. Never evaluate configuration as code."""
import copy
import math
import os
from pathlib import Path
import re
import stat
import tomllib

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'etc' / 'robot.toml'


class ConfigError(ValueError):
    """A safe diagnostic that never includes configuration values."""


def defaults(root):
    return {
        'robot': {'ssh_host': 'robot', 'player_host': '', 'player_port': 6665},
        'server': {'listen': '127.0.0.1', 'port': 8085},
        'supervisor': {'host': '', 'port': 7665, 'timeout_ms': 5000,
                       'token': '', 'listen': '0.0.0.0',
                       'binary': str(root / 'crates/target/armv7-unknown-linux-gnueabihf/release/robot-supervisor')},
        'run': {'slam_adapter': str(root / 'tools/mrpt-adapter/adapter'),
                'yolo_adapter': str(root / 'tools/yolo/adapter')},
        'camera': {'netrc': str(Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'robot/camera.netrc'),
                   'snapshot_url': 'http://cam/image.jpg', 'mjpeg_url': 'http://cam/mjpeg',
                   'audio_url': 'http://cam/audio.opus'},
        'audio': {'profile': '', 'endpoint': 'http://127.0.0.1:8085', 'volume': 0.35,
                  'max_volume': 0.9},
        'yandex': {'api_key': '', 'url': 'https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize',
                   'voice': 'filipp', 'folder_id': '', 'enabled': False},
        'providers': {},
    }


ENV = {
    'ROBOT_SSH_HOST': ('robot', 'ssh_host'), 'ROBOT_PLAYER_HOST': ('robot', 'player_host'),
    'ROBOT_PLAYER_PORT': ('robot', 'player_port'),
    'ROBOT_SERVER_LISTEN': ('server', 'listen'), 'ROBOT_SERVER_PORT': ('server', 'port'),
    'ROBOT_SUPERVISOR_HOST': ('supervisor', 'host'), 'ROBOT_SUPERVISOR_PORT': ('supervisor', 'port'),
    'ROBOT_SUPERVISOR_TIMEOUT_MS': ('supervisor', 'timeout_ms'),
    'ROBOT_SUPERVISOR_TOKEN': ('supervisor', 'token'),
    'ROBOT_SUPERVISOR_BIN': ('supervisor', 'binary'),
    'ROBOT_SUPERVISOR_LISTEN': ('supervisor', 'listen'),
    'ROBOT_SLAM_ADAPTER': ('run', 'slam_adapter'), 'ROBOT_YOLO_ADAPTER': ('run', 'yolo_adapter'),
    'ROBOT_CAMERA_NETRC': ('camera', 'netrc'), 'ROBOT_CAMERA_URL': ('camera', 'mjpeg_url'),
    'ROBOT_CAMERA_SNAPSHOT_URL': ('camera', 'snapshot_url'),
    'ROBOT_CAMERA_AUDIO_URL': ('camera', 'audio_url'),
    'ROBOT_AUDIO_CONFIG': ('audio', 'profile'), 'ROBOT_API_ENDPOINT': ('audio', 'endpoint'),
    'ROBOT_AUDIO_VOLUME': ('audio', 'volume'), 'ROBOT_AUDIO_MAX_VOLUME': ('audio', 'max_volume'),
    **{'ROBOT_YANDEX_' + key.upper(): ('yandex', key)
       for key in ('api_key', 'url', 'voice', 'folder_id', 'enabled')},
}
PATHS = {('supervisor', 'binary'), ('run', 'slam_adapter'), ('run', 'yolo_adapter'),
         ('camera', 'netrc'), ('audio', 'profile')}


def validate(config, schema):
    for section, values in config.items():
        if section not in schema or not isinstance(values, dict):
            raise ConfigError('config: unknown or invalid section')
        for key, value in values.items():
            if section == 'providers':
                if not re.fullmatch(r'[a-z][a-z0-9_]*_api_key', key) or not isinstance(value, str):
                    raise ConfigError('config: invalid provider key')
            elif key not in schema[section]:
                raise ConfigError('config: unknown setting')
            else:
                expected = type(schema[section][key])
                valid = (type(value) in (int, float) and math.isfinite(value)
                         if expected is float else type(value) is expected)
                if not valid:
                    raise ConfigError('config: invalid setting type')
            if isinstance(value, str) and ('\0' in value or '\n' in value or '\r' in value):
                raise ConfigError('config: control characters are not allowed')
    for section, key in (('server', 'port'), ('supervisor', 'port'), ('robot', 'player_port')):
        if not 1 <= config[section][key] <= 65535:
            raise ConfigError('config: port outside 1..65535')
    if not 100 <= config['supervisor']['timeout_ms'] <= 300000:
        raise ConfigError('config: timeout outside 100..300000')
    if not 0 <= config['audio']['volume'] <= config['audio']['max_volume'] <= 1:
        raise ConfigError('config: invalid audio volume limits')
    for section, key in (('robot', 'ssh_host'), ('robot', 'player_host'),
                         ('supervisor', 'host'), ('supervisor', 'listen'), ('server', 'listen')):
        if config[section][key] and not re.fullmatch(r'[a-zA-Z0-9_.:\[\]-]+', config[section][key]):
            raise ConfigError('config: invalid host address')


def load_config(path=None, *, root=ROOT, environ=None):
    root = Path(root)
    environ = os.environ if environ is None else environ
    explicit = path is not None
    path = Path(path) if explicit else root / 'etc/robot.toml'
    schema = defaults(root)
    config = copy.deepcopy(schema)
    data = {}
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except FileNotFoundError:
        if explicit:
            raise ConfigError('config: file does not exist') from None
    except OSError:
        raise ConfigError('config: cannot open private regular file') from None
    else:
        try:
            with os.fdopen(fd, 'rb') as stream:
                info = os.fstat(stream.fileno())
                if not stat.S_ISREG(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o600 or info.st_uid != os.getuid():
                    raise ConfigError('config: file must be owner-owned regular mode 0600')
                data = tomllib.load(stream)
        except (tomllib.TOMLDecodeError, UnicodeError, OSError):
            raise ConfigError('config: invalid TOML file') from None
        for section, values in data.items():
            if section not in schema or not isinstance(values, dict):
                raise ConfigError('config: unknown or invalid section')
            config[section].update(values)
        validate(config, schema)
        for section, key in PATHS:
            value = config[section][key]
            if value and key in data.get(section, {}):
                config[section][key] = str((path.parent / Path(value).expanduser()).absolute())
    for name, (section, key) in ENV.items():
        if name not in environ:
            continue
        value = environ[name]
        expected = type(schema[section][key])
        try:
            if expected is bool:
                if value.lower() not in ('true', 'false', '1', '0'):
                    raise ValueError()
                value = value.lower() in ('true', '1')
            elif expected in (int, float):
                value = expected(value)
        except ValueError:
            raise ConfigError('config: invalid environment setting') from None
        config[section][key] = value
    if 'ROBOT_SUPERVISOR_ADDR' in environ:
        try:
            host, port = environ['ROBOT_SUPERVISOR_ADDR'].rsplit(':', 1)
            if not host:
                raise ValueError()
            config['supervisor']['host'] = host
            config['supervisor']['port'] = int(port)
        except ValueError:
            raise ConfigError('config: invalid supervisor address') from None
    if 'endpoint' not in data.get('audio', {}) and 'ROBOT_API_ENDPOINT' not in environ:
        host = config['server']['listen']
        host = {'0.0.0.0': '127.0.0.1', '::': '::1'}.get(host, host)
        if ':' in host and not host.startswith('['):
            host = f'[{host}]'
        config['audio']['endpoint'] = f"http://{host}:{config['server']['port']}"
    validate(config, schema)
    return config


def export_env(config, *, environ=None):
    env = dict(os.environ if environ is None else environ)
    for name, (section, key) in ENV.items():
        value = config[section][key]
        env[name] = str(value).lower() if isinstance(value, bool) else str(value)
    supervisor = config['supervisor']
    host = supervisor['host'] or config['robot']['player_host']
    if host:
        env['ROBOT_SUPERVISOR_ADDR'] = f"{host}:{supervisor['port']}"
    for key, value in config['providers'].items():
        env['ROBOT_PROVIDER_' + key.upper()] = value
    return env
