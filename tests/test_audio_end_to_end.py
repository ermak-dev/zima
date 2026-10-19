#!/usr/bin/env python3
"""FEATURE: PCM -> wake -> Russian command -> ROS Action -> TTS -> playback.

SPEC: Real ROS/runtime/codecs, injected model/provider/device; no live or paid I/O.
"""
import base64
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2'))
from test_audio_runtime import Source, wav_bytes, wait_for


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'source ROS and audio interface overlay')
class EndToEndAudioTests(unittest.TestCase):
    def test_voice_command_and_reply_are_independent_of_motion(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from robot_audio.playback import AssetStore, PlaybackQueue, TransportResult
        from robot_audio.ros_node import AudioNode
        from robot_audio.runtime import AudioRuntime
        from robot_audio.speech import BoundedRecognizer, SpeechSession, RmsVad
        from robot_audio.tts import OpenRouterTTS
        from robot_audio.types import AudioChunk
        from robot_audio.wake import WakeDetector
        from robot_audio_interfaces.action import Say
        from robot_audio_interfaces.msg import AudioChunk as RosChunk, SpeechTranscript, PlaybackState
        from robot_audio_interfaces.srv import FinishSpeech

        class Backend:
            def predict(self, pcm): return .9
            def reset(self): pass
        transport_calls, provider_calls = [], []
        class Transport:
            def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
                transport_calls.append(asset.asset_id)
                if on_started: on_started()
                if on_reference: on_reference(asset.pcm, asset.sample_rate, time.monotonic())
                cancel_event.wait(asset.duration_s)
                return TransportResult('cancelled' if cancel_event.is_set() else 'completed')
        class Response(io.BytesIO):
            status = 200
        def provider(request, timeout):
            body = json.loads(request.data)
            provider_calls.append(body)
            text = json.loads(body['messages'][-1]['content'])['text']
            payload = {'choices': [{'delta': {'audio': {'data': base64.b64encode(wav_bytes()).decode(),
                                                       'transcript': text}}}]}
            return Response(('data: ' + json.dumps(payload) + '\n\ndata: [DONE]\n\n').encode())

        with tempfile.TemporaryDirectory() as directory:
            source = Source()
            playback = PlaybackQueue(AssetStore(directory), Transport())
            session = SpeechSession(BoundedRecognizer(lambda pcm: 'эй робот осмотри коридор'),
                wake_phrase='эй робот', vad=RmsVad(-40), silence_s=.05)
            config = {'input': {'enabled': True, 'microphone_verified': True},
                      'output': {'enabled': True}, 'tts': {'enabled': True, 'allow_external': True,
                      'model': 'offline/model', 'voice': 'offline'}}
            runtime = AudioRuntime(config, source=source, playback=playback, session=session,
                detector=WakeDetector(Backend(), phrase='эй робот'),
                tts=OpenRouterTTS(model='offline/model', voice='offline', api_key='offline-key',
                                  opener=provider, require_verbatim=True))
            context = Context(); rclpy.init(context=context, domain_id=215)
            server = AudioNode(runtime=runtime, config=config, context=context)
            client = rclpy.create_node('voice_fixture_client', context=context)
            executor = MultiThreadedExecutor(num_threads=4, context=context)
            executor.add_node(server); executor.add_node(client)
            thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
            transcripts, states, references, pcm = [], [], [], []
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            client.create_subscription(SpeechTranscript, '/robot/speech/transcript', transcripts.append, 5)
            client.create_subscription(PlaybackState, '/robot/audio/output/state', states.append, 10)
            client.create_subscription(RosChunk, '/robot/audio/output/reference', references.append, qos)
            client.create_subscription(RosChunk, '/robot/audio/input/pcm', pcm.append, qos)
            try:
                self.assertTrue(wait_for(lambda: server.count_subscribers('/robot/audio/input/pcm') > 0))
                epoch = time.time_ns()
                for i in range(16):
                    source.queue.put(AudioChunk('synthetic', i, epoch + i * 20_000_000,
                        (b'\x10\x27' if i < 4 else b'\x00\x00') * 320))
                    time.sleep(.02)
                self.assertTrue(wait_for(lambda: transcripts))
                self.assertEqual(transcripts[0].text, 'осмотри коридор')
                self.assertEqual(transcripts[0].status, 'final')
                finish = client.create_client(FinishSpeech, '/robot/speech/finish')
                self.assertTrue(finish.wait_for_service(timeout_sec=3))
                acknowledged = finish.call_async(FinishSpeech.Request(session_id=transcripts[0].session_id))
                self.assertTrue(wait_for(acknowledged.done)); self.assertTrue(acknowledged.result().success)
                action = ActionClient(client, Say, '/robot/speech/say')
                self.assertTrue(action.wait_for_server(timeout_sec=3))
                goal = Say.Goal(text='Начинаю осмотр', voice='offline', volume=.25)
                goal.max_queue_wait.sec = 3
                accepted = action.send_goal_async(goal)
                self.assertTrue(wait_for(accepted.done)); self.assertTrue(accepted.result().accepted)
                completed = accepted.result().get_result_async()
                self.assertTrue(wait_for(completed.done)); self.assertEqual(completed.result().result.error_code, 0)
                self.assertEqual(len(provider_calls), 1)
                self.assertEqual(json.loads(provider_calls[0]['messages'][-1]['content'])['text'], 'Начинаю осмотр')
                self.assertEqual(len(transport_calls), 1)
                self.assertTrue(wait_for(lambda: references))
                self.assertTrue(any(state.speaking for state in states))
                self.assertTrue(wait_for(lambda: states and not states[-1].speaking))
                self.assertTrue(pcm)
                self.assertNotIn('/cmd_vel', [name for name, _ in server.get_publisher_names_and_types_by_node(
                    server.get_name(), server.get_namespace())])
            finally:
                server.close(); executor.shutdown(timeout_sec=4); thread.join(4)
                client.destroy_node(); server.destroy_node(); context.try_shutdown()


if __name__ == '__main__': unittest.main()
