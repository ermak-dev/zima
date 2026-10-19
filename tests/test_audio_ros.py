#!/usr/bin/env python3
"""FEATURE: Offline ROS audio Actions, PCM/state publication and cancellation.

SPEC: Isolated DDS only, injected backends, no camera/robot or paid API calls.
"""
import importlib.util
from pathlib import Path
import sys
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))


def wait_for(predicate, timeout=6):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.01)
    return bool(predicate())


class FakeRuntime:
    def __init__(self):
        self.callbacks = {}
        self.calls = []
        self.started = threading.Event()
        self.block = False
        self.closed = False
        self.listening = True
        self.muted = False

    def set_callbacks(self, **callbacks):
        self.callbacks = callbacks

    def start(self):
        pass

    def close(self):
        self.closed = True

    def set_listening(self, enabled):
        self.listening = enabled
        return {'success': True, 'message': 'enabled' if enabled else 'disabled'}

    def set_muted(self, muted):
        self.muted = muted
        return {'success': True, 'message': 'checked'}

    def response_finished(self, session_id):
        return {'success': session_id == 'session-1', 'message': 'checked'}

    def execute(self, kind, request, task_id, cancel, feedback):
        self.calls.append((kind, request, task_id))
        self.started.set()
        feedback({'phase': 'playing', 'played_s': .1})
        if self.block:
            if not cancel.wait(5):
                return {'error_code': 4, 'message': 'test timeout'}
            return {'error_code': 3, 'message': 'cancelled'}
        return {'error_code': 0, 'message': 'completed'}


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'source ROS Jazzy and the audio interface overlay')
class AudioRosTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.ros_node import AudioNode
        except ImportError:
            self.fail('ROS audio adapter is not implemented or its interfaces are not built')
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context, domain_id=217)
        self.runtime = FakeRuntime()
        self.action_clients = []
        self.server = AudioNode(runtime=self.runtime, context=self.context)
        self.client = rclpy.create_node('audio_offline_client', context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.server)
        self.executor.add_node(self.client)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.addCleanup(self.cleanup_ros)

    def cleanup_ros(self):
        self.server.close()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        for client in self.action_clients:
            client.destroy()
        self.client.destroy_node()
        self.server.destroy_node()
        self.context.try_shutdown()
        self.assertTrue(self.runtime.closed)

    def action_client(self, action, name):
        from rclpy.action import ActionClient
        client = ActionClient(self.client, action, name)
        self.action_clients.append(client)
        self.assertTrue(client.wait_for_server(timeout_sec=5))
        return client

    def send(self, client, goal, feedback=None):
        future = client.send_goal_async(goal, feedback_callback=feedback)
        self.assertTrue(wait_for(future.done))
        return future.result()

    def test_play_and_say_return_completed_results_and_feedback(self):
        from robot_audio_interfaces.action import PlayAudio, Say
        feedback = []
        play = self.action_client(PlayAudio, '/robot/audio/play')
        goal = PlayAudio.Goal(asset_id='beep', volume=.25, priority=1)
        goal.max_queue_wait.sec = 2
        handle = self.send(play, goal, lambda msg: feedback.append(msg.feedback.phase))
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.assertTrue(wait_for(result.done))
        self.assertEqual(result.result().status, 4)
        self.assertEqual(result.result().result.error_code, 0)
        self.assertTrue(wait_for(lambda: 'playing' in feedback))
        say = self.action_client(Say, '/robot/speech/say')
        request = Say.Goal(text='Проверка', voice='configured', volume=.25)
        request.max_queue_wait.sec = 2
        handle = self.send(say, request)
        result = handle.get_result_async()
        self.assertTrue(wait_for(result.done))
        self.assertEqual(result.result().result.error_code, 0)
        self.assertEqual([call[0] for call in self.runtime.calls], ['play', 'say'])
        self.assertEqual(self.runtime.calls[1][1]['text'], 'Проверка')

    def test_cancellation_reaches_active_backend(self):
        from robot_audio_interfaces.action import PlayAudio
        self.runtime.block = True
        client = self.action_client(PlayAudio, '/robot/audio/play')
        goal = PlayAudio.Goal(asset_id='beep', volume=.25)
        goal.max_queue_wait.sec = 2
        handle = self.send(client, goal)
        self.assertTrue(handle.accepted)
        self.assertTrue(self.runtime.started.wait(3))
        cancelled = handle.cancel_goal_async()
        self.assertTrue(wait_for(cancelled.done))
        result = handle.get_result_async()
        self.assertTrue(wait_for(result.done))
        self.assertEqual(result.result().status, 5)
        self.assertEqual(result.result().result.error_code, 3)

    def test_concurrent_actions_do_not_starve_cancellation_callbacks(self):
        from robot_audio_interfaces.action import PlayAudio
        self.runtime.block = True
        client = self.action_client(PlayAudio, '/robot/audio/play')
        requests = []
        for _ in range(4):
            goal = PlayAudio.Goal(asset_id='beep', volume=.25)
            goal.max_queue_wait.sec = 2
            requests.append(client.send_goal_async(goal))
        self.assertTrue(wait_for(lambda: all(future.done() for future in requests), timeout=1.5),
                        'action workers must not monopolize the ROS executor')
        handles = [future.result() for future in requests]
        self.assertTrue(all(handle.accepted for handle in handles))
        self.assertTrue(wait_for(lambda: len(self.runtime.calls) == 4, timeout=1))
        cancellations = [handle.cancel_goal_async() for handle in handles]
        self.assertTrue(wait_for(lambda: all(future.done() for future in cancellations), timeout=1))
        results = [handle.get_result_async() for handle in handles]
        self.assertTrue(wait_for(lambda: all(future.done() for future in results), timeout=1))
        self.assertTrue(all(future.result().status == 5 for future in results))

    def test_action_admission_is_bounded_before_starting_worker_threads(self):
        from robot_audio_interfaces.action import PlayAudio
        self.runtime.block = True
        self.server.config['output']['queue_size'] = 1
        client = self.action_client(PlayAudio, '/robot/audio/play')
        handles = []
        for _ in range(3):
            goal = PlayAudio.Goal(asset_id='beep', volume=.25)
            goal.max_queue_wait.sec = 2
            handles.append(self.send(client, goal))
        self.assertEqual([handle.accepted for handle in handles], [True, True, False])
        cancellations = [handle.cancel_goal_async() for handle in handles[:2]]
        self.assertTrue(wait_for(lambda: all(future.done() for future in cancellations)))
        results = [handle.get_result_async() for handle in handles[:2]]
        self.assertTrue(wait_for(lambda: all(future.done() for future in results)))
        self.assertTrue(all(future.result().status == 5 for future in results))

    def test_invalid_volume_and_asset_are_rejected_before_backend(self):
        from robot_audio_interfaces.action import PlayAudio
        client = self.action_client(PlayAudio, '/robot/audio/play')
        for asset, volume in (('../secret', .2), ('beep', float('nan')), ('beep', 1.1)):
            with self.subTest(asset=asset, volume=volume):
                goal = PlayAudio.Goal(asset_id=asset, volume=volume)
                goal.max_queue_wait.sec = 2
                self.assertFalse(self.send(client, goal).accepted)
        self.assertEqual(self.runtime.calls, [])

    def test_pcm_and_input_state_keep_format_epoch_and_microphone_uncertainty(self):
        from robot_audio.types import AudioChunk
        from robot_audio_interfaces.msg import AudioChunk as RosChunk, AudioInputState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        chunks, states = [], []
        self.client.create_subscription(RosChunk, '/robot/audio/input/pcm', chunks.append,
                                        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.client.create_subscription(AudioInputState, '/robot/audio/input/state', states.append,
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.assertTrue(wait_for(lambda: self.server.count_subscribers('/robot/audio/input/pcm') > 0))
        chunk = AudioChunk('offline', 2, 1_000_000_000, b'\x00\x00' * 320, discontinuity=True)
        self.runtime.callbacks['chunk'](chunk)
        self.runtime.callbacks['input_state']({'state': 'streaming', 'stream_id': 'offline',
                                                'last_frame_age': 0.0, 'microphone_verified': False})
        self.assertTrue(wait_for(lambda: chunks and states and states[-1].state == 'streaming'))
        self.assertEqual(chunks[-1].stream_id, 'offline')
        self.assertEqual(chunks[-1].sequence, 2)
        self.assertEqual(chunks[-1].sample_rate, 16000)
        self.assertEqual(chunks[-1].header.stamp.sec, 1)
        self.assertTrue(chunks[-1].estimated_timestamp)
        self.assertTrue(chunks[-1].discontinuity)
        self.assertFalse(states[-1].microphone_verified)
        import math
        self.assertTrue(math.isnan(states[-1].peak), 'unknown peak must not look like a measured zero')

    def test_speech_readiness_and_session_completion_are_explicit(self):
        from robot_audio_interfaces.msg import SpeechState
        from robot_audio_interfaces.srv import FinishSpeech
        from rclpy.qos import QoSProfile, DurabilityPolicy
        states = []
        self.client.create_subscription(SpeechState, '/robot/speech/state', states.append,
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.assertIn('speech_state', self.runtime.callbacks)
        self.runtime.callbacks['speech_state']({'state': 'processing', 'session_id': 'session-1',
            'listening': True, 'wake_ready': True, 'asr_ready': True,
            'duplex_mode': 'half_duplex', 'error': ''})
        self.assertTrue(wait_for(lambda: states and states[-1].session_id == 'session-1'))
        self.assertEqual(states[-1].state, 'processing')
        service = self.client.create_client(FinishSpeech, '/robot/speech/finish')
        self.assertTrue(service.wait_for_service(timeout_sec=4))
        for session_id, expected in (('wrong', False), ('session-1', True)):
            future = service.call_async(FinishSpeech.Request(session_id=session_id))
            self.assertTrue(wait_for(future.done))
            self.assertEqual(future.result().success, expected)

    def test_cli_records_the_shared_pcm_without_a_second_camera_connection(self):
        import json
        import os
        import subprocess
        import tempfile
        import wave
        from robot_audio.types import AudioChunk
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'record.wav'
            env = {**os.environ, 'ROS_DOMAIN_ID': '217'}
            process = subprocess.Popen([sys.executable, str(ROOT / 'bin/audio/cli.py'), 'record',
                str(path), '--seconds', '.1', '--timeout', '3'], env=env,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                self.assertTrue(wait_for(lambda: self.server.count_subscribers('/robot/audio/input/pcm') > 0
                                         or process.poll() is not None))
                self.assertIsNone(process.poll(), 'record CLI exited before subscribing')
                for i in range(10):
                    self.runtime.callbacks['chunk'](AudioChunk('fixture', i, time.time_ns(),
                                                              b'\x10\x00' * 320))
                    time.sleep(.02)
                output, error = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, error)
                self.assertEqual(json.loads(output)['frames'], 1600)
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
                with wave.open(str(path), 'rb') as reader:
                    self.assertEqual(reader.getnframes(), 1600)
            finally:
                if process.poll() is None: process.kill(); process.communicate()

    def test_speaker_mute_does_not_disable_listening(self):
        from std_srvs.srv import SetBool
        client = self.client.create_client(SetBool, '/robot/audio/output/set_muted')
        self.assertTrue(client.wait_for_service(timeout_sec=1))
        future = client.call_async(SetBool.Request(data=True))
        self.assertTrue(wait_for(future.done))
        self.assertTrue(future.result().success)
        self.assertTrue(self.runtime.muted)
        self.assertTrue(self.runtime.listening)

    def test_listening_can_be_disabled_without_any_motion_api(self):
        from std_srvs.srv import SetBool
        client = self.client.create_client(SetBool, '/robot/audio/listening/set_enabled')
        self.assertTrue(client.wait_for_service(timeout_sec=4))
        future = client.call_async(SetBool.Request(data=False))
        self.assertTrue(wait_for(future.done))
        self.assertTrue(future.result().success)
        self.assertFalse(self.runtime.listening)
        topics = [name for name, _ in self.server.get_publisher_names_and_types_by_node(
            self.server.get_name(), self.server.get_namespace())]
        self.assertNotIn('/cmd_vel', topics)


if __name__ == '__main__':
    unittest.main()
