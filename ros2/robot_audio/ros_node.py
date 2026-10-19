"""ROS transport for the host audio runtime; it never publishes robot motion."""
import math
import re
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_srvs.srv import SetBool

from robot_audio_interfaces.action import PlayAudio, Say
from robot_audio_interfaces.msg import (
    AudioChunk as RosChunk, AudioInfo, AudioInputState, PlaybackState,
    SpeechTranscript, SpeechState, WakeWord,
)
from robot_audio_interfaces.srv import FinishSpeech

from .config import validate_config
from .types import AudioChunk


def _stamp(target, ns):
    ns = max(0, int(ns))
    target.sec, target.nanosec = divmod(ns, 1_000_000_000)


def _duration(value):
    if value.sec < 0 or not 0 <= value.nanosec < 1_000_000_000:
        raise ValueError('invalid duration')
    return value.sec + value.nanosec / 1e9


def _seconds(value):
    value = max(0.0, float(value))
    if not math.isfinite(value):
        value = 0.0
    ns = int(value * 1e9)
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class AudioNode(Node):
    def __init__(self, runtime=None, config=None, **kwargs):
        super().__init__('robot_audio', **kwargs)
        self.config = validate_config(config or {})
        if runtime is None:
            from .runtime import AudioRuntime
            runtime = AudioRuntime(self.config)
        self.runtime = runtime
        self._closed = False
        self._goal_lock = threading.Lock()
        self._cancellations = {}
        self._workers = set()
        self._reserved = 0
        self._input = {'state': 'disabled', 'stream_id': '', 'last_frame_age': -1.0,
                       'microphone_verified': False, 'error': ''}
        self._last_chunk_at = None
        self._group = ReentrantCallbackGroup()
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pcm_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pcm_pub = self.create_publisher(RosChunk, '/robot/audio/input/pcm', pcm_qos)
        self._reference_pub = self.create_publisher(RosChunk, '/robot/audio/output/reference', pcm_qos)
        self._info_pub = self.create_publisher(AudioInfo, '/robot/audio/input/info', state_qos)
        self._input_pub = self.create_publisher(AudioInputState, '/robot/audio/input/state', state_qos)
        self._playback_pub = self.create_publisher(PlaybackState, '/robot/audio/output/state', state_qos)
        self._wake_pub = self.create_publisher(WakeWord, '/robot/audio/wake_word', 5)
        self._transcript_pub = self.create_publisher(SpeechTranscript, '/robot/speech/transcript', 5)
        self._speech_pub = self.create_publisher(SpeechState, '/robot/speech/state', state_qos)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 5)
        self._actions = [
            ActionServer(self, PlayAudio, '/robot/audio/play',
                         execute_callback=self._play_action,
                         goal_callback=lambda goal: self._goal('play', goal),
                         cancel_callback=self._cancel, callback_group=self._group),
            ActionServer(self, Say, '/robot/speech/say',
                         execute_callback=self._say_action,
                         goal_callback=lambda goal: self._goal('say', goal),
                         cancel_callback=self._cancel, callback_group=self._group),
        ]
        self._listening_service = self.create_service(
            SetBool, '/robot/audio/listening/set_enabled', self._set_listening,
            callback_group=self._group)
        self._mute_service = self.create_service(
            SetBool, '/robot/audio/output/set_muted', self._set_muted, callback_group=self._group)
        self._finish_service = self.create_service(
            FinishSpeech, '/robot/speech/finish', self._finish_speech, callback_group=self._group)
        runtime.set_callbacks(chunk=self._chunk, input_state=self._input_state,
                              wake=self._wake, transcript=self._transcript,
                              playback=self._playback, reference=self._reference,
                              speech_state=self._speech_state, diagnostic=self._component_diagnostic)
        info = AudioInfo(source='openipc_camera', sample_rate=16000, channels=1,
                         sample_format='s16le', coding_format='pcm', estimated_timestamp=True)
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = 'camera_microphone'
        self._info_pub.publish(info)
        self._input_state(self._input)
        self.create_timer(.5, self._diagnostics)
        runtime.start()

    def _goal(self, kind, goal):
        try:
            if not math.isfinite(goal.volume) or not 0 <= goal.volume <= self.config['output']['max_volume']:
                return GoalResponse.REJECT
            if goal.priority > 2 or not 0 < _duration(goal.max_queue_wait) <= 60:
                return GoalResponse.REJECT
            if kind == 'play':
                if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', goal.asset_id) or '..' in goal.asset_id:
                    return GoalResponse.REJECT
            elif not goal.text.strip() or len(goal.text) > self.config['tts']['max_text_chars'] or '\x00' in goal.text or len(goal.voice) > 128:
                return GoalResponse.REJECT
        except (ValueError, TypeError):
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._closed or self._reserved >= self.config['output']['queue_size'] + 1:
                return GoalResponse.REJECT
            self._reserved += 1
        return GoalResponse.ACCEPT

    def _cancel(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        with self._goal_lock:
            self._cancellations.setdefault(task_id, threading.Event()).set()
        return CancelResponse.ACCEPT

    async def _play_action(self, handle):
        return await self._execute('play', PlayAudio, handle)

    async def _say_action(self, handle):
        return await self._execute('say', Say, handle)

    async def _execute(self, kind, action, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        with self._goal_lock:
            cancel = self._cancellations.setdefault(task_id, threading.Event())
        request = {'volume': float(handle.request.volume), 'priority': int(handle.request.priority),
                   'max_queue_wait': _duration(handle.request.max_queue_wait)}
        if kind == 'play':
            request['asset_id'] = handle.request.asset_id
        else:
            request.update(text=handle.request.text, voice=handle.request.voice)

        def feedback(value):
            if handle.is_active:
                message = action.Feedback(phase=str(value.get('phase', '')),
                                          played=_seconds(value.get('played_s', 0)),
                                          task_id=task_id)
                handle.publish_feedback(message)

        completed = Future(executor=self.executor)
        def run_backend():
            try:
                value = self.runtime.execute(kind, request, task_id, cancel, feedback)
            except Exception:
                value = {'error_code': 4, 'message': 'audio operation failed'}
            completed.set_result(value)
        worker = threading.Thread(target=run_backend, name='ros-audio-action', daemon=True)
        with self._goal_lock:
            self._workers.add(worker)
        worker.start()
        try:
            value = await completed
            code = int(value.get('error_code', 4))
            result = action.Result(error_code=code, message=str(value.get('message', '')),
                                   task_id=task_id)
            if code == 0:
                handle.succeed()
            elif code == 3 and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            return result
        except Exception:
            handle.abort()
            return action.Result(error_code=4, message='audio operation failed', task_id=task_id)
        finally:
            with self._goal_lock:
                self._cancellations.pop(task_id, None)
                self._workers.discard(worker)
                self._reserved -= 1

    def _set_listening(self, request, response):
        value = self.runtime.set_listening(bool(request.data))
        response.success = bool(value.get('success', False))
        response.message = str(value.get('message', ''))
        return response

    def _set_muted(self, request, response):
        value = self.runtime.set_muted(bool(request.data))
        response.success = bool(value.get('success', False))
        response.message = str(value.get('message', ''))
        return response

    def _finish_speech(self, request, response):
        value = self.runtime.response_finished(request.session_id)
        response.success = bool(value.get('success', False))
        response.message = str(value.get('message', ''))
        return response

    def _speech_state(self, value):
        message = SpeechState(state=str(value.get('state', 'disabled')),
            session_id=str(value.get('session_id', '')), listening=bool(value.get('listening', False)),
            wake_ready=bool(value.get('wake_ready', False)), asr_ready=bool(value.get('asr_ready', False)),
            duplex_mode=str(value.get('duplex_mode', 'half_duplex')), error=str(value.get('error', '')))
        message.header.stamp = self.get_clock().now().to_msg()
        self._speech_pub.publish(message)

    def _component_diagnostic(self, value):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [DiagnosticStatus(name='robot_audio/' + str(value.get('name', 'runtime')),
            hardware_id='host_audio', level=DiagnosticStatus.WARN, message=str(value.get('state', 'unknown')))]
        self._diagnostic_pub.publish(message)

    def _ros_chunk(self, chunk):
        if not isinstance(chunk, AudioChunk):
            raise TypeError('expected normalized AudioChunk')
        message = RosChunk(stream_id=chunk.stream_id, sequence=chunk.sequence,
                           sample_rate=chunk.sample_rate, channels=chunk.channels,
                           sample_format=chunk.sample_format, discontinuity=chunk.discontinuity,
                           estimated_timestamp=chunk.estimated_timestamp, data=list(chunk.pcm))
        _stamp(message.header.stamp, chunk.stamp_ns)
        message.header.frame_id = 'camera_microphone'
        return message

    def _chunk(self, chunk):
        self._last_chunk_at = time.monotonic()
        self._pcm_pub.publish(self._ros_chunk(chunk))

    def _reference(self, chunk):
        message = self._ros_chunk(chunk)
        message.header.frame_id = 'robot_speaker'
        self._reference_pub.publish(message)

    def _input_state(self, value):
        self._input.update(value)
        message = AudioInputState(
            source='openipc_camera', state=str(self._input.get('state', 'unknown')),
            stream_id=str(self._input.get('stream_id', '')),
            last_frame_age=float(self._input.get('last_frame_age', -1)),
            level_dbfs=float(self._input.get('level_dbfs', float('nan'))),
            peak=float(self._input.get('peak', float('nan'))), clipping=bool(self._input.get('clipping', False)),
            microphone_verified=bool(self._input.get('microphone_verified', False)),
            error=str(self._input.get('error', '')))
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'camera_microphone'
        self._input_pub.publish(message)

    def _wake(self, value):
        message = WakeWord(event_id=str(value['event_id']), stream_id=str(value['stream_id']),
                           phrase=str(value.get('phrase', '')), model_id=str(value.get('model_id', '')),
                           score=float(value.get('score', 0)), start_frame=int(value.get('start_frame', 0)),
                           end_frame=int(value.get('end_frame', 0)))
        _stamp(message.header.stamp, value.get('stamp_ns', time.time_ns()))
        self._wake_pub.publish(message)

    def _transcript(self, value):
        message = SpeechTranscript(session_id=str(value.get('session_id', '')),
                                   stream_id=str(value.get('stream_id', '')),
                                   text=str(value.get('text', '')), is_final=bool(value.get('is_final', True)),
                                   status=str(value.get('status', '')), error=str(value.get('error', '')))
        _stamp(message.header.stamp, value.get('stamp_ns', time.time_ns()))
        self._transcript_pub.publish(message)

    def _playback(self, value):
        message = PlaybackState(state=str(value.get('state', 'idle')),
                                task_id=str(value.get('task_id', '')), speaking=bool(value.get('speaking', False)),
                                volume=float(value.get('volume', 0)), played_samples=int(value.get('played_samples', 0)),
                                error=str(value.get('error', '')))
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'robot_speaker'
        self._playback_pub.publish(message)

    def _diagnostics(self):
        state = self._input.get('state', 'unknown')
        age = -1.0 if self._last_chunk_at is None else time.monotonic() - self._last_chunk_at
        if state == 'streaming':
            self._input_state({'last_frame_age': age})
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='robot_audio/input', hardware_id='openipc_camera',
                                  level=DiagnosticStatus.OK if state == 'streaming' else DiagnosticStatus.WARN, message=str(state))
        status.values = [KeyValue(key='last_frame_age', value=str(age)),
                         KeyValue(key='microphone_verified', value=str(self._input.get('microphone_verified', False)))]
        message.status = [status]
        self._diagnostic_pub.publish(message)

    def close(self):
        if self._closed:
            return
        self._closed = True
        with self._goal_lock:
            for event in self._cancellations.values():
                event.set()
        self.runtime.close()
        with self._goal_lock:
            workers = list(self._workers)
        for worker in workers:
            worker.join(timeout=6)
