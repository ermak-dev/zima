#!/usr/bin/env python3
"""Audio commands; device and provider access require explicit enabled configuration."""
import argparse
import contextlib
import json
import math
import os
import queue
import signal
import subprocess
from urllib.parse import urlsplit
from pathlib import Path
import re
import sys
import time
import wave

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'python'))

from robot_audio.config import load_config


def parser():
    p = argparse.ArgumentParser(prog='robot audio')
    sub = p.add_subparsers(dest='command', required=True)
    for name in ('run', 'check-config'):
        item = sub.add_parser(name)
        item.add_argument('--config', type=Path)
        if name == 'run':
            item.add_argument('--port', type=int, default=8086)
    companion = sub.add_parser('companion', help='explicit bounded LDS/camera/gain session; no motion')
    companion.add_argument('--config', type=Path, required=True)
    companion.add_argument('--max-seconds', type=int, default=600)
    companion.add_argument('--startup-seconds', type=int, default=120)
    companion.add_argument('--gain', type=int, default=90)
    for name in ('status', 'stop', 'start', 'finish'):
        item = sub.add_parser(name)
        item.add_argument('--timeout', type=float, default=5)
        if name == 'finish':
            item.add_argument('session_id')
        if name == 'stop':
            item.add_argument('--task-id')
    record = sub.add_parser('record')
    record.add_argument('wav', type=Path)
    record.add_argument('--seconds', type=float, default=6)
    record.add_argument('--timeout', type=float, default=35)
    capture = sub.add_parser('capture', help='standalone test WAV; no ROS, no robot modes')
    capture.add_argument('wav', type=Path)
    capture.add_argument('--config', type=Path)
    capture.add_argument('--seconds', type=float, default=6)
    capture.add_argument('--timeout', type=float, default=35)
    capture.add_argument('--gain', type=int, help='temporary camera gain 0..90, restored after')
    for name in ('listen', 'mute'):
        toggle = sub.add_parser(name)
        toggle.add_argument('state', choices=('on', 'off'))
        toggle.add_argument('--timeout', type=float, default=5)
    for name in ('play', 'say'):
        item = sub.add_parser(name)
        if name == 'play':
            item.add_argument('asset_id')
        else:
            item.add_argument('text', nargs='?')
            item.add_argument('--text-file', type=Path)
            item.add_argument('--voice', default='')
        item.add_argument('--volume', type=float, default=.35)
        item.add_argument('--priority', type=int, choices=(0, 1, 2), default=0)
        item.add_argument('--ttl', type=float, default=5)
        item.add_argument('--timeout', type=float, default=45)
    deploy = sub.add_parser('deploy')
    deploy.add_argument('--binary', type=Path, required=True)
    deploy.add_argument('--config', type=Path)
    deploy.add_argument('--confirm', action='store_true')
    prepare = sub.add_parser('import')
    prepare.add_argument('wav', type=Path)
    prepare.add_argument('--config', type=Path)
    reference = sub.add_parser('reference')
    reference.add_argument('wav', type=Path)
    analyze = sub.add_parser('analyze')
    analyze.add_argument('wav', type=Path)
    analyze.add_argument('--reference', type=Path)
    for name in ('status', 'stop', 'start', 'finish', 'record', 'listen', 'mute', 'play', 'say'):
        sub.choices[name].add_argument('--endpoint', default='http://127.0.0.1:8086')
    return p


def _ipc(endpoint, payload, timeout):
    from urllib.request import Request, build_opener, ProxyHandler
    parsed = urlsplit(endpoint)
    if (parsed.scheme != 'http' or parsed.hostname != '127.0.0.1' or
            parsed.username or parsed.password or parsed.path not in ('', '/') or
            parsed.query or parsed.fragment or not parsed.port):
        raise ValueError('audio endpoint must be http://127.0.0.1:PORT')
    request = Request(endpoint.rstrip('/') + '/request',
        data=json.dumps(payload, allow_nan=False).encode(),
        headers={'Content-Type': 'application/json'}, method='POST')
    # Never send private audio IPC through environment-configured proxies.
    with build_opener(ProxyHandler({})).open(request, timeout=timeout) as response:
        data = response.read(2_000_001)
    if len(data) > 2_000_000:
        raise ValueError('audio response too large')
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError('invalid audio response')
    return value


def _native_request(args):
    import base64
    import io
    import uuid
    payload = {'command': args.command, 'timeout': args.timeout}
    if args.command in ('listen', 'mute'):
        payload['enabled'] = args.state == 'on'
    elif args.command == 'finish':
        payload['session_id'] = args.session_id
    elif args.command == 'stop' and args.task_id:
        payload['task_id'] = args.task_id
    elif args.command == 'record':
        payload['seconds'] = args.seconds
        if args.wav.expanduser().exists() or args.wav.expanduser().is_symlink():
            raise ValueError('recording never overwrites an existing output')
    elif args.command in ('play', 'say'):
        payload.update(volume=args.volume, priority=args.priority, ttl=args.ttl, task_id=uuid.uuid4().hex)
        payload.update({'asset_id': args.asset_id} if args.command == 'play' else
                       {'text': args.text, 'voice': args.voice})
    try:
        result = _ipc(args.endpoint, payload, args.timeout + 8)
    except (OSError, KeyboardInterrupt):
        if 'task_id' in payload:
            try:
                _ipc(args.endpoint, {'command': 'stop', 'task_id': payload['task_id']}, 5)
            except Exception:
                pass  # Outcome remains unknown; never retry playback.
        raise
    if args.command == 'record':
        data = base64.b64decode(result.pop('wav_base64'), validate=True)
        with wave.open(io.BytesIO(data)) as reader:
            if (reader.getnchannels(), reader.getsampwidth(), reader.getframerate(), reader.getnframes()) != (
                    1, 2, 16000, round(args.seconds * 16000)):
                raise ValueError('invalid recorded WAV')
        path = args.wav.expanduser()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            with os.fdopen(fd, 'wb') as output:
                output.write(data)
        except BaseException:
            path.unlink(missing_ok=True)
            raise
        result['file'] = str(path)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get('success', True) and result.get('error_code', 0) == 0 else 1


def _gain_lease(cfg, volume, timeout):
    """Temporary camera gain without owning LDS power: the camera is reachable on its own.

    `audio companion` additionally stops the stock controller and spins the LDS, whose
    noise dominates the microphone; a test WAV must not require either.
    """
    from robot_audio.camera_session import CameraGainLease, CameraSSH
    parsed = urlsplit(cfg['url'])
    if parsed.scheme != 'http' or parsed.port not in (None, 80):
        raise ValueError('camera gain expects the OpenIPC HTTP endpoint on port 80')
    ssh = CameraSSH(host=parsed.hostname, netrc_path=cfg['netrc'])
    return CameraGainLease(ssh, volume=volume, ttl_s=max(60, int(timeout) + 60))


class CameraSessionInterrupted(Exception):
    """Marker for the sanitized interrupt raised by bounded camera contexts."""


def _interrupted(error):
    return isinstance(error, RuntimeError) and str(error) == 'session_interrupted'


def _enter_gain(cfg, volume, timeout):
    # Bounded camera contexts convert SIGINT/SIGTERM into
    # CameraSessionError('session_interrupted'); give the CLI's interrupt
    # handling its semantics back instead of reporting a failed operation
    # with the gain possibly still applied. Enters the lease; the caller
    # registers the exit itself (ExitStack.enter_context would enter twice).
    try:
        lease = _gain_lease(cfg, volume, timeout)
        lease.__enter__()
    except CameraSessionInterrupted:
        raise
    except Exception as error:
        if _interrupted(error):
            raise CameraSessionInterrupted from None
        raise
    return lease


def _capture_once(source, path, seconds, deadline, *, log=None, started=None):
    """One attempt. Lost samples raise ValueError; no partial file survives either way."""
    from robot_audio.recording import PcmRecorder
    recorder = PcmRecorder(path, seconds=seconds)
    first_chunk = True
    try:
        while not recorder.complete:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('camera did not provide a complete recording')
            try:
                recorder.feed(source.get_chunk(min(.5, remaining)))
                if first_chunk and log is not None:
                    # Log at the first audio, not before it: the wait for the
                    # decoder warmup (up to ~2 s after a gain reload) happens
                    # inside this loop, and probing here would spend a chunk.
                    first_chunk = False
                    stream_id = ''
                    if hasattr(source, 'state'):
                        stream_id = str(source.state().get('stream_id', ''))[:8]
                    log(f'stream is live ({stream_id}…), first audio after '
                        f'{time.monotonic() - started:.1f}s; recording {seconds}s')
            except queue.Empty:
                continue
        return recorder.frames
    finally:
        recorder.close()


def _run_capture(args):
    from robot_audio.acoustics import analyze_pcm
    from robot_audio.capture import CameraAudioSource, CaptureConfig
    # Unlike `audio run`, capture may fall back to the user's default profile:
    # it is a standalone test command, so the user default is the expected way
    # to consent. An explicit --config still wins; the refusal for a disabled
    # input must keep naming that switch.
    if args.config is None:
        base = os.environ.get('XDG_CONFIG_HOME') or '~/.config'
        default = Path(base, 'robot', 'audio.json').expanduser()
        args.config = default if default.is_file() else None
    config = load_config(args.config)
    cfg = config['input']
    if not cfg['enabled']:
        raise ValueError('capture requires an explicitly enabled microphone source: '
                         'pass --config with a profile where input.enabled is true')
    path = args.wav.expanduser()
    if path.exists() or path.is_symlink():
        raise ValueError(f'output already exists: {path} (recording never overwrites)')

    started = time.monotonic()

    def log(message):
        # Progress goes to stderr; stdout stays a single JSON report.
        print(f'[{time.monotonic() - started:.1f}s] {message}', file=sys.stderr, flush=True)

    report = {}
    host = urlsplit(cfg['url']).hostname
    log(f'config {args.config or "built-in defaults"}; output {path}')
    with contextlib.ExitStack() as stack:
        if args.gain is not None:
            # Gain first: applying it reloads the camera audio and drops a live stream.
            log(f'raising camera gain to {args.gain} on {host} '
                f'(3 SSH steps + audio reload; the slow part, ~5 s)')
            lease = _enter_gain(cfg, args.gain, args.timeout)
            stack.callback(lease.__exit__, None, None, None)
            report['camera_volume'] = {'original': lease.original_volume,
                                       'active': lease.volume}
            log(f'gain {lease.original_volume}->{lease.volume} confirmed by readback')
        log(f'connecting to {cfg["url"]}')
        source = CameraAudioSource(CaptureConfig(
            url=cfg['url'], netrc_path=cfg['netrc'], frame_ms=cfg['chunk_ms'],
            queue_size=cfg['queue_size'], connect_timeout=cfg['timeout_s'],
            stall_timeout=cfg['timeout_s']))
        source.start()
        stack.callback(source.stop)
        deadline = time.monotonic() + args.timeout
        while True:
            try:
                log(f'recording {args.seconds}s from the live stream')
                frames = _capture_once(source, path, args.seconds, deadline,
                                       log=log, started=started)
                break
            except ValueError as error:
                # A reconnect or dropped block invalidates the interval, not the source:
                # start the recording again while the explicit timeout still allows it.
                log(f'recording invalidated ({error}); retrying within the timeout')
                if time.monotonic() >= deadline:
                    raise TimeoutError(str(error)) from None
    log(f'wrote {frames} frames to {path}')
    with wave.open(str(path)) as handle:
        level = analyze_pcm(handle.readframes(handle.getnframes()))
    report.update({'file': str(path), 'frames': frames, 'sample_rate': 16000,
                   'channels': 1, 'sample_format': 's16le',
                   'level': {key: level[key] for key in ('ac_dbfs', 'peak', 'clipping_fraction')}})
    print(json.dumps(report))
    return 0


def _stop_child(process):
    previous = {}
    try:
        # Session handlers raise: repeated signals must not bypass bounded shutdown.
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, signal.SIG_IGN)
        if process.poll() is not None:
            return
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        except ProcessLookupError:
            process.wait(timeout=2)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def _run_companion(args):
    from robot_audio.camera_session import CameraSession, CameraGainLease, CameraSSH, CameraSessionError
    config = load_config(args.config)
    if not config['input']['enabled']:
        raise ValueError('companion requires an explicitly enabled microphone source')
    if not 1 <= args.max_seconds <= 3600 or not 1 <= args.startup_seconds <= 3600 or not 0 <= args.gain <= 100:
        raise ValueError('invalid bounded camera session parameters')
    parsed = urlsplit(config['input']['url'])
    if parsed.scheme != 'http' or parsed.port not in (None, 80):
        raise ValueError('camera session expects the OpenIPC HTTP endpoint on port 80')
    ssh = CameraSSH(host=parsed.hostname, netrc_path=config['input']['netrc'])
    gain = CameraGainLease(ssh, volume=args.gain, ttl_s=args.max_seconds, startup_s=args.startup_seconds)
    try:
        with CameraSession(explicit=True, ttl_s=args.max_seconds, startup_s=args.startup_seconds, gain=gain) as session:
            command = [str(ROOT/'bin/robot'), 'audio', 'run', '--config', str(args.config)]
            process = subprocess.Popen(command, start_new_session=True)
            next_check = 0.0
            try:
                while process.poll() is None:
                    if time.monotonic() >= next_check or getattr(session, 'remaining_s', 1) <= 0:
                        session.check()
                        next_check = time.monotonic() + 10
                    time.sleep(.1)
                code = process.poll()
            finally:
                _stop_child(process)
        return 0 if code == 0 else 1
    except CameraSessionError as error:
        if str(error) == 'session_interrupted':
            raise KeyboardInterrupt from None
        raise


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        if hasattr(args, 'timeout') and (not math.isfinite(args.timeout) or not 0 < args.timeout <= 180):
            raise ValueError('timeout must be between zero and 180 seconds')
        if args.command in ('record', 'capture') and (not math.isfinite(args.seconds) or not .1 <= args.seconds <= 30):
            raise ValueError('recording duration must be .1..30 seconds')
        if args.command == 'capture' and args.gain is not None and not 0 <= args.gain <= 90:
            raise ValueError('camera gain must be 0..90; measured 100 returns digital zeros')
        if args.command in ('play', 'say'):
            if not math.isfinite(args.volume) or not 0 <= args.volume <= 1:
                raise ValueError('volume must be between zero and one')
            if not math.isfinite(args.ttl) or not 0 < args.ttl <= 60:
                raise ValueError('queue TTL must be between zero and 60 seconds')
        if args.command == 'play' and (not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', args.asset_id) or '..' in args.asset_id):
            raise ValueError('play requires a prepared asset identifier, not a path or URL')
        if args.command == 'say':
            if args.text_file and args.text is not None:
                raise ValueError('choose text or --text-file, not both')
            if args.text_file:
                if args.text_file.stat().st_size > 16000:
                    raise ValueError('text file is too large')
                args.text = args.text_file.read_text(encoding='utf-8')
            if not args.text or not args.text.strip() or len(args.text) > 2000 or '\x00' in args.text:
                raise ValueError('speech requires 1..2000 text characters')
        if args.command == 'check-config':
            print(json.dumps(load_config(args.config), ensure_ascii=False, indent=2))
            return 0
        if args.command == 'companion':
            return _run_companion(args)
        if args.command == 'capture':
            return _run_capture(args)
        if args.command == 'run':
            config = load_config(args.config)
            from robot_audio.service import run_foreground
            run_foreground(config, port=args.port)
            return 0
        if args.command == 'reference':
            from robot_audio.acoustics import write_reference_wav
            print(json.dumps(write_reference_wav(args.wav)))
            return 0
        if args.command == 'deploy':
            from robot_audio.deployment import deploy
            print(json.dumps(deploy(args.binary, load_config(args.config), confirmed=args.confirm)))
            return 0
        if args.command == 'import':
            from robot_audio.local_tools import prepare_asset
            print(json.dumps(prepare_asset(args.wav, load_config(args.config))))
            return 0
        if args.command == 'analyze':
            from robot_audio.local_tools import run_local
            return run_local(args)
        return _native_request(args)
    except KeyboardInterrupt:
        return 130
    except CameraSessionInterrupted:
        # The bounded camera contexts turn SIGINT/SIGTERM into a sanitized
        # error; restore the interrupt semantics of every other command.
        return 130
    except (ValueError, TypeError) as error:
        print(f'robot audio: {error}', file=sys.stderr)
        return 2
    except ImportError:
        print('robot audio: install the explicitly configured optional audio backend', file=sys.stderr)
        return 2
    except TimeoutError:
        print('robot audio: operation timed out; outcome may be unknown, no automatic retry', file=sys.stderr)
        return 1
    except Exception:
        print('robot audio: operation failed; check audio diagnostics and local configuration', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
