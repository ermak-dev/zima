#!/usr/bin/env python3
"""Audio commands; device and provider access require explicit enabled configuration."""
import argparse
import json
import math
import signal
import subprocess
from urllib.parse import urlsplit
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2'))

from robot_audio.config import load_config


def parser():
    p = argparse.ArgumentParser(prog='robot audio')
    sub = p.add_subparsers(dest='command', required=True)
    for name in ('run', 'check-config'):
        item = sub.add_parser(name)
        item.add_argument('--config', type=Path)
        if name == 'run':
            item.add_argument('--experimental-wake', action='store_true')
    companion = sub.add_parser('companion', help='explicit bounded LDS/camera/gain session; no motion')
    companion.add_argument('--config', type=Path, required=True)
    companion.add_argument('--experimental-wake', action='store_true')
    companion.add_argument('--max-seconds', type=int, default=600)
    companion.add_argument('--startup-seconds', type=int, default=120)
    companion.add_argument('--gain', type=int, default=90)
    for name in ('status', 'stop'):
        item = sub.add_parser(name)
        item.add_argument('--timeout', type=float, default=5)
    record = sub.add_parser('record')
    record.add_argument('wav', type=Path)
    record.add_argument('--seconds', type=float, default=6)
    record.add_argument('--timeout', type=float, default=35)
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
    train = sub.add_parser('train')
    train.add_argument('manifest', type=Path)
    train.add_argument('--features-dir', type=Path, required=True)
    train.add_argument('--output', type=Path, required=True)
    train.add_argument('--kind', choices=('logistic', 'mlp'), default='logistic')
    train.add_argument('--window-frames', type=int, default=16)
    train.add_argument('--seed', type=int, default=17)
    evaluate = sub.add_parser('evaluate')
    evaluate.add_argument('manifest', type=Path)
    evaluate.add_argument('--wake-manifest', type=Path)
    evaluate.add_argument('--threshold', type=float, default=.5)
    evaluate.add_argument('--split', choices=('validation', 'test'), default='test')
    evaluate.add_argument('--calibrate', action='store_true')
    return p


def _wait(rclpy, node, future, timeout):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(.05, max(0, end - time.monotonic())))
    if not future.done():
        raise TimeoutError('audio operation timed out; playback outcome may be unknown')
    return future.result()


def _record_ros(rclpy, node, args):
    from robot_audio.recording import PcmRecorder
    from robot_audio.types import AudioChunk
    from robot_audio_interfaces.msg import AudioChunk as RosChunk
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    recorder = PcmRecorder(args.wav, seconds=args.seconds)
    errors = []
    def receive(message):
        if recorder.complete or errors:
            return
        try:
            recorder.feed(AudioChunk(message.stream_id, message.sequence,
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                bytes(message.data), sample_rate=message.sample_rate, channels=message.channels,
                sample_format=message.sample_format, discontinuity=message.discontinuity,
                estimated_timestamp=message.estimated_timestamp))
        except (ValueError, TypeError):
            errors.append('audio stream changed or lost samples during recording')
            recorder.close()
    node.create_subscription(RosChunk, '/robot/audio/input/pcm', receive,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        deadline = time.monotonic() + args.timeout
        while not recorder.complete and not errors and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if errors:
            raise ValueError(errors[0])
        if not recorder.complete:
            raise TimeoutError('audio source did not provide a complete recording')
        print(json.dumps({'file': str(args.wav), 'frames': recorder.frames,
                          'sample_rate': 16000, 'channels': 1, 'sample_format': 's16le'}))
        return 0
    finally:
        recorder.close()


def _ros_request(args):
    import rclpy
    rclpy.init()
    node = rclpy.create_node('robot_audio_client')
    try:
        if args.command == 'record':
            return _record_ros(rclpy, node, args)
        if args.command == 'status':
            from robot_audio_interfaces.msg import AudioInputState, PlaybackState
            from rclpy.qos import QoSProfile, DurabilityPolicy
            values = {}
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            def input_status(msg):
                values['input'] = {'state': msg.state, 'source': msg.source,
                                   'stream_id': msg.stream_id, 'last_frame_age': msg.last_frame_age,
                                   'microphone_verified': msg.microphone_verified, 'error': msg.error}
            def output_status(msg):
                values['output'] = {'state': msg.state, 'speaking': msg.speaking,
                                    'task_id': msg.task_id, 'error': msg.error}
            node.create_subscription(AudioInputState, '/robot/audio/input/state', input_status, qos)
            node.create_subscription(PlaybackState, '/robot/audio/output/state', output_status, qos)
            end = time.monotonic() + args.timeout
            while len(values) < 2 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            if not values:
                raise TimeoutError('audio server is unavailable')
            print(json.dumps(values, ensure_ascii=False))
            return 0
        if args.command in ('listen', 'mute'):
            from std_srvs.srv import SetBool
            service = '/robot/audio/listening/set_enabled' if args.command == 'listen' else '/robot/audio/output/set_muted'
            client = node.create_client(SetBool, service)
            if not client.wait_for_service(timeout_sec=args.timeout):
                raise TimeoutError('audio server is unavailable')
            result = _wait(rclpy, node, client.call_async(SetBool.Request(data=args.state == 'on')), args.timeout)
            print(json.dumps({'success': result.success, 'message': result.message}))
            return 0 if result.success else 1
        if args.command == 'stop':
            from action_msgs.srv import CancelGoal
            result = {}
            for name in ('/robot/audio/play', '/robot/speech/say'):
                client = node.create_client(CancelGoal, name + '/_action/cancel_goal')
                if not client.wait_for_service(timeout_sec=args.timeout):
                    result[name] = {'error': 'unavailable'}
                    continue
                response = _wait(rclpy, node, client.call_async(CancelGoal.Request()), args.timeout)
                result[name] = {'requested': len(response.goals_canceling), 'return_code': response.return_code}
            print(json.dumps(result))
            return 0 if all('error' not in value for value in result.values()) else 1
        from rclpy.action import ActionClient
        from robot_audio_interfaces.action import PlayAudio, Say
        action = PlayAudio if args.command == 'play' else Say
        name = '/robot/audio/play' if args.command == 'play' else '/robot/speech/say'
        client = ActionClient(node, action, name)
        if not client.wait_for_server(timeout_sec=min(5, args.timeout)):
            raise TimeoutError('audio action server is unavailable')
        goal = action.Goal(volume=args.volume, priority=args.priority)
        ns = int(args.ttl * 1e9)
        goal.max_queue_wait.sec, goal.max_queue_wait.nanosec = divmod(ns, 1_000_000_000)
        if args.command == 'play':
            goal.asset_id = args.asset_id
        else:
            goal.text, goal.voice = args.text, args.voice
        handle = _wait(rclpy, node, client.send_goal_async(goal), min(5, args.timeout))
        if not handle.accepted:
            print(json.dumps({'error_code': 1, 'message': 'audio request rejected'}))
            return 1
        try:
            completed = _wait(rclpy, node, handle.get_result_async(), args.timeout)
        except (TimeoutError, KeyboardInterrupt):
            handle.cancel_goal_async()
            raise
        result = completed.result
        print(json.dumps({'error_code': result.error_code, 'message': result.message,
                          'task_id': result.task_id, 'status': completed.status}, ensure_ascii=False))
        return 0 if result.error_code == 0 else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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
            if args.experimental_wake:
                command.append('--experimental-wake')
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
        if args.command == 'record' and (not math.isfinite(args.seconds) or not .1 <= args.seconds <= 30):
            raise ValueError('recording duration must be .1..30 seconds')
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
        if args.command == 'run':
            config = load_config(args.config)
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from robot_audio.ros_node import AudioNode
            rclpy.init()
            node = None
            executor = None
            try:
                node = AudioNode(config=config, allow_experimental_wake=args.experimental_wake)
                executor = MultiThreadedExecutor(num_threads=4)
                executor.add_node(node)
                executor.spin()
            finally:
                if node:
                    node.close()
                if executor:
                    executor.shutdown(timeout_sec=5)
                if node:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
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
        if args.command == 'train':
            from robot_audio.training import train_model
            print(json.dumps(train_model(args.manifest, args.features_dir, args.output,
                kind=args.kind, window_frames=args.window_frames, seed=args.seed), ensure_ascii=False, indent=2))
            return 0
        if args.command in ('analyze', 'evaluate'):
            from robot_audio.local_tools import run_local
            return run_local(args)
        return _ros_request(args)
    except KeyboardInterrupt:
        return 130
    except (ValueError, TypeError) as error:
        print(f'robot audio: {error}', file=sys.stderr)
        return 2
    except ImportError:
        print('robot audio: build/source the audio interfaces and install the configured optional backend', file=sys.stderr)
        return 2
    except TimeoutError:
        print('robot audio: operation timed out; outcome may be unknown, no automatic retry', file=sys.stderr)
        return 1
    except Exception:
        print('robot audio: operation failed; check audio diagnostics and local configuration', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
