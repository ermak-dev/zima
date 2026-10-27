#!/usr/bin/env python3
"""Speak text on the robot with local Piper TTS; no ROS and no cloud.

Called by bin/robot as:  say.py "text" --config FILE [--volume V]
 [--priority N] [--piper-module-path DIR]

The robot's helper accepts one WAV (16 kHz mono PCM16, at most 30 s) per
invocation over ssh, so a long text is spoken as several sequential helper
runs; chunking keeps the first utterance short so the robot starts talking
before the whole text is synthesized.
"""
import io
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2'))

from robot_audio.config import load_config
from robot_audio.playback import AssetStore, AudioError, SSHPlaybackTransport
from robot_audio.say import (HelperTransport, PiperTTS, YandexTTS,
                             load_yandex_credentials, speak)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(
        prog='robot say',
        description='Speak text on the robot with local Piper TTS (no ROS, no cloud)')
    parser.add_argument('text', nargs='?')
    parser.add_argument('--text-file', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--volume', type=float, default=.35)
    parser.add_argument('--yandex-env', type=Path,
                        help='env file with YANDEX_API_KEY/YANDEX_FOLDER; '
                             'selects Yandex SpeechKit synthesis')
    parser.add_argument('--piper-module-path', type=Path,
                        help='directory containing an importable piper package')
    args = parser.parse_args(argv)

    if args.text_file and args.text is not None:
        parser.error('choose text or --text-file, not both')
    if args.text_file:
        if args.text_file.stat().st_size > 16000:
            raise SystemExit('robot say: text file is too large')
        args.text = args.text_file.read_text(encoding='utf-8')
    if not args.text or not args.text.strip() or len(args.text) > 2000 or '\x00' in args.text:
        raise SystemExit('robot say: text must be 1..2000 characters')
    if not 0 <= args.volume <= 1:
        raise SystemExit('robot say: volume must be between zero and one')

    config = load_config(args.config)
    # `robot say` is an explicit command: it plays by default and does not
    # require the runtime-only output.enabled flag to be set by hand.
    model = config['tts']['piper_model']
    if not model:
        model = None  # PiperTTS discovers an installed voice on its own.

    if args.piper_module_path:
        sys.path.insert(0, str(args.piper_module_path))

    store = AssetStore(config['output']['asset_dir'],
                       max_bytes=config['tts']['max_audio_bytes'],
                       max_duration_s=config['output']['max_duration_s'],
                       ttl_s=config['tts']['cache_ttl_s'])
    transport = HelperTransport(store, SSHPlaybackTransport(
        host=config['output']['host'], helper_path=config['output']['helper'],
        ssh=os.environ.get('ROBOT_SSH', 'ssh'),
        timeout_s=config['output']['max_duration_s'] + 10))
    # The backend is explicit, never silently switched: --yandex-env (or
    # tts.yandex_env in the config) selects SpeechKit, otherwise local Piper.
    yandex_env = args.yandex_env or (Path(config['tts']['yandex_env'])
                                     if config['tts']['yandex_env'] else None)
    try:
        if yandex_env:
            api_key, folder = load_yandex_credentials(yandex_env)
            tts = YandexTTS(api_key, folder_id=folder,
                            url=config['tts']['yandex_url'] or '')
        else:
            tts = PiperTTS(model)
        speak(tts, transport, args.text, volume=args.volume)
    except AudioError as error:
        print(f'robot say: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
