#!/usr/bin/env python3
"""Load TOML once, then re-exec the shell CLI with data-only environment."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from robot_config import ConfigError, export_env, load_config


def main():
    args = sys.argv[1:]
    path = os.environ.get('ROBOT_CONFIG') or None
    if args[:1] == ['--config']:
        if len(args) < 3:
            raise ConfigError('config: --config requires a path and command')
        path, args = args[1], args[2:]
    if args and args[0] in ('run', 'panel', 'bridge', 'serve', 'daemon'):
        cleaned = [args[0]]
        index = 1
        while index < len(args):
            if args[index] == '--config':
                if index + 1 >= len(args):
                    raise ConfigError('config: --config requires a path')
                path = args[index + 1]
                index += 2
            else:
                cleaned.append(args[index])
                index += 1
        args = cleaned
    config = load_config(path)
    env = export_env(config)
    env['_ROBOT_CONFIG_LOADED'] = '1'
    os.execve(str(ROOT / 'bin/robot'), [str(ROOT / 'bin/robot'), *args], env)


if __name__ == '__main__':
    try:
        main()
    except ConfigError as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
