"""Composition of capture, local recognition and cancellable speaker output."""
from collections import OrderedDict, deque
from dataclasses import replace
import hashlib
import json
import queue
import threading
import time
import uuid

from .config import validate_config
from .types import AudioChunk

_TERMINAL = {'completed', 'cancelled', 'expired', 'failed', 'unknown'}


class AudioRuntime:
    def __init__(self, config=None, *, source=None, detector=None, session=None,
                 playback=None, tts=None, echo=None, output_policy=None, clock=time.monotonic,
                 allow_experimental_wake=False):
        if type(allow_experimental_wake) is not bool:
            raise ValueError('experimental wake consent must be boolean')
        self.allow_experimental_wake = allow_experimental_wake
        self.config = validate_config(config or {})
        self.clock = clock
        from .policy import OutputPolicy
        output = self.config['output']
        self.output_policy = output_policy or OutputPolicy(muted=output['muted'],
            quiet_hours=output['quiet_hours'], timezone_name=output['timezone'])
        self._last_inhibited = self.output_policy.inhibited
        self.source, self.detector, self.session = source, detector, session
        self.playback, self.tts, self.echo = playback, tts, echo
        self._callbacks = {}
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._threads = []
        self._closed = self._started = False
        self._listening = self.config['input']['enabled']
        self._stream_id = None
        self._source_failed = False
        self._active_playback = None
        self._hold_until = 0.0
        self._reference = None
        self._echo_mode = 'half_duplex'
        self._echo_reason = ''
        self._inflight = {}
        self._tts_slot = threading.BoundedSemaphore(1)
        self._results = OrderedDict()
        self._errors = {}
        self._barge = None
        self._last_input_state = self._last_speech_state = None
        self._configure()

    def _configure(self):
        if self.source is None and self.config['input']['enabled']:
            from .capture import CameraAudioSource, CaptureConfig
            cfg = self.config['input']
            self.source = CameraAudioSource(CaptureConfig(
                url=cfg['url'], netrc_path=cfg['netrc'], frame_ms=cfg['chunk_ms'],
                queue_size=cfg['queue_size'], connect_timeout=cfg['timeout_s'], stall_timeout=cfg['timeout_s']))
        if self.detector is None and self.config['wake']['enabled']:
            try:
                from .wake import load_wake_manifest, OpenWakeWordBackend, WakeDetector
                cfg = self.config['wake']
                manifest = load_wake_manifest(cfg['manifest'])
                russian = (manifest['language'].lower() == 'ru' or
                    (self.config['speech']['enabled'] and self.config['speech']['language'] == 'ru'))
                if russian:
                    manifest = load_wake_manifest(cfg['manifest'], require_russian=True,
                                                  allow_experimental=self.allow_experimental_wake)
                if manifest.get('experimental'):
                    self._errors['wake'] = 'experimental_model'
                if russian and abs(cfg['threshold'] - manifest['evaluation_threshold']) > 1e-9:
                    raise ValueError('wake threshold differs from held-out evaluation')
                self.detector = WakeDetector(OpenWakeWordBackend(manifest), phrase=manifest['phrase'],
                    model=manifest.get('model_name', 'local'), threshold=cfg['threshold'],
                    debounce_s=cfg['refractory_s'], pre_roll_s=cfg['preroll_s'])
            except (ValueError, ImportError, OSError):
                self._errors['wake'] = 'configuration_required'
        if self.session is None and self.config['speech']['enabled']:
            try:
                from .speech import FasterWhisperASR, BoundedRecognizer, SpeechSession, RmsVad
                cfg = self.config['speech']
                self.session = SpeechSession(
                    BoundedRecognizer(FasterWhisperASR(cfg['asr_model_path'], language=cfg['language'])),
                    wake_phrase=self.detector.phrase if self.detector else '', vad=RmsVad(cfg['vad_threshold_dbfs']),
                    silence_s=cfg['end_silence_s'], wait_s=cfg['listen_timeout_s'],
                    max_command_s=cfg['max_command_s'], asr_timeout_s=cfg['asr_timeout_s'],
                    echo_tail_s=cfg['echo_tail_s'], clock=self.clock)
            except (ValueError, ImportError, OSError):
                self._errors['speech'] = 'configuration_required'
        if self.echo is None and self.config['echo']['enabled']:
            try:
                from .echo import SpeexEchoCanceller, EchoController
                cfg = self.config['echo']
                self.echo = EchoController(SpeexEchoCanceller(filter_length=cfg['filter_ms'] * 16),
                    delay_ms=cfg['delay_ms'], max_drift_ms=cfg['max_drift_ms'], calibrated=cfg['verified'])
            except (ValueError, ImportError, OSError, RuntimeError):
                self._errors['echo'] = 'configuration_required'
        if self.playback is None and self.config['output']['enabled']:
            from .playback import AssetStore, PlaybackQueue, SSHPlaybackTransport
            cfg = self.config['output']
            store = AssetStore(cfg['asset_dir'], max_duration_s=cfg['max_duration_s'],
                               ttl_s=self.config['tts']['cache_ttl_s'])
            self.playback = PlaybackQueue(store, SSHPlaybackTransport(
                host=cfg['host'], helper_path=cfg['helper'], timeout_s=cfg['max_duration_s'] + 10),
                max_queue=cfg['queue_size'])
        if self.playback is not None:
            self.playback.on_state = self._on_playback
            self.playback.on_reference = self._on_reference
        if self.tts is None and self.config['tts']['enabled']:
            from .tts import OpenRouterTTS
            cfg = self.config['tts']
            self.tts = OpenRouterTTS(model=cfg['model'], voice=cfg['voice'], token_file=cfg['api_key_file'],
                max_chars=cfg['max_text_chars'], max_audio_bytes=cfg['max_audio_bytes'],
                max_duration_s=self.config['output']['max_duration_s'], timeout_s=cfg['timeout_s'],
                max_output_tokens=cfg['max_output_tokens'], env_file=cfg['env_file'] or None,
                audio_format=cfg['audio_format'], pcm_sample_rate=cfg['pcm_sample_rate'],
                require_verbatim=cfg['require_verbatim'])

    def set_callbacks(self, **callbacks):
        self._callbacks = callbacks

    def _emit(self, key, value):
        callback = self._callbacks.get(key)
        if callback is not None and not self._closed:
            callback(value)

    def start(self):
        if self._started or self._closed:
            return
        self._started = True
        self._emit('input_state', self._input_state())
        self._emit('playback', {'state': ('muted' if self.output_policy.inhibited else 'idle')
                               if self.playback else 'disabled', 'speaking': False})
        self._publish_speech()
        for name, code in self._errors.items():
            self._emit('diagnostic', {'name': name, 'state': code})
        if self.source is not None and self._listening:
            self.source.start()
        workers = [('audio-monitor', self._monitor)]
        if self.source is not None:
            workers.append(('audio-capture-consumer', self._capture))
        if self.playback is not None:
            workers.append(('audio-playback', self._play))
        for name, target in workers:
            thread = threading.Thread(target=target, name=name, daemon=True)
            self._threads.append(thread)
            thread.start()

    def _input_state(self):
        if self.source is None or not self._listening:
            return {'state': 'disabled', 'source': 'openipc_camera', 'stream_id': '',
                    'last_frame_age': -1.0, 'microphone_verified': False, 'error': ''}
        state = self.source.state()
        return {'state': state['state'], 'source': 'openipc_camera',
                'stream_id': state.get('stream_id') or '',
                'last_frame_age': state.get('last_frame_age_s') if state.get('last_frame_age_s') is not None else -1.0,
                'level_dbfs': state.get('signal_dbfs') if state.get('signal_dbfs') is not None else float('nan'),
                'clipping': bool(state.get('clipping_fraction', 0)),
                'microphone_verified': self.config['input']['microphone_verified'],
                'error': state.get('error') or ''}

    def _capture(self):
        while not self._stop.is_set():
            if not self._listening:
                self._stop.wait(.05)
                continue
            try:
                chunk = self.source.get_chunk(timeout=.05)
            except queue.Empty:
                continue
            except Exception:
                self._emit('diagnostic', {'name': 'capture', 'state': 'capture_failed'})
                self._stop.wait(.1)
                continue
            try:
                with self._lock:
                    if self._listening and not self._closed:
                        self.process_chunk(chunk)
            except Exception:
                with self._lock:
                    self._errors['speech'] = 'processing_failed'
                    self.detector = None
                    if self.session:
                        self.session.source_lost()
                    self._emit('diagnostic', {'name': 'recognition', 'state': 'processing_failed'})
                    self._publish_speech()

    def process_chunk(self, chunk):
        with self._lock:
            self._emit('chunk', chunk)
            if self._source_failed or (self._stream_id is not None and chunk.stream_id != self._stream_id):
                if self.session:
                    self.session.reset()
                if self.detector:
                    self.detector.reset()
                if self.echo:
                    self.echo.reset()
            self._source_failed = False
            self._stream_id = chunk.stream_id
            active = self._active_playback is not None
            allow_wake, allow_barge = not active, False
            clean = chunk
            if self.echo is not None and len(chunk.pcm) == 640:
                result = self.echo.process_capture(chunk, playback_active=active)
                clean = replace(chunk, pcm=result.pcm)
                allow_wake, allow_barge = result.allow_wake, result.allow_barge_in
                self._echo_mode = 'full_duplex' if (allow_barge and
                    self.config['speech']['mode'] == 'full_duplex') else 'half_duplex'
                reason = getattr(result, 'reason', '')
                if reason and reason != self._echo_reason:
                    self._emit('diagnostic', {'name': 'echo', 'state': reason})
                self._echo_reason = reason
            if active and self.config['speech']['mode'] != 'full_duplex':
                allow_wake = allow_barge = False
            if self._barge is not None:
                buffered = self._barge['chunks']
                if sum(len(item[0].pcm) for item in buffered) + len(clean.pcm) <= 160000:
                    buffered.append((clean, self.clock()))
                else:
                    self._barge['overflow'] = True
            elif self.session is not None:
                self.session.feed(clean, now=self.clock())
            if self.detector is not None:
                allowed = (self._listening and self.config['input']['microphone_verified'] and
                           allow_wake and self.clock() >= self._hold_until and self._barge is None)
                self.detector.set_enabled(allowed)
                if allowed:
                    for event in self.detector.feed(clean):
                        end_frame = (event.sequence + 1) * chunk.frames
                        self._emit('wake', {'event_id': event.event_id, 'stream_id': event.stream_id,
                            'phrase': event.phrase, 'model_id': event.model, 'score': event.score,
                            'stamp_ns': event.stamp_ns, 'start_frame': max(0, end_frame - len(event.pre_roll_pcm) // 2),
                            'end_frame': end_frame})
                        if self.session:
                            if active and allow_barge:
                                self._begin_barge(event)
                            else:
                                if self.session.state == 'fault':
                                    self.session.reset()
                                self.session.on_wake(event, now=self.clock())
            self._publish_speech()

    def _monitor(self):
        last_state_at = 0.0
        while not self._stop.wait(.02):
            with self._lock:
                if self._closed:
                    return
                self._enforce_quiet()
                self._flush_reference()
                if self.session is not None and self._barge is None:
                    for transcript in self.session.tick(now=self.clock()):
                        self._emit('transcript', {'session_id': transcript.session_id,
                            'stream_id': transcript.stream_id, 'text': transcript.text,
                            'stamp_ns': transcript.stamp_ns, 'is_final': transcript.final,
                            'status': 'final', 'error': transcript.error})
                if self.clock() - last_state_at >= .2:
                    state = self._input_state()
                    self._emit('input_state', state)
                    if self._listening and state['state'] not in ('streaming', 'connecting') and not self._source_failed:
                        self._source_failed = True
                        if self.session:
                            self.session.source_lost()
                        if self.detector:
                            self.detector.reset()
                        if self.echo:
                            self.echo.reset()
                    self._publish_speech()
                    last_state_at = self.clock()

    def _publish_speech(self):
        state = {'state': self.session.state if self.session else 'disabled',
                 'session_id': self.session.session_id or '' if self.session else '',
                 'listening': self._listening, 'wake_ready': self.detector is not None,
                 'asr_ready': self.session is not None,
                 'duplex_mode': self._echo_mode,
                 'error': (self.session.last_error if self.session else '') or
                          self._errors.get('wake', '') or self._errors.get('speech', '')}
        if self.detector and not self.config['input']['microphone_verified']:
            state['error'] = 'microphone_unverified'
        if state != self._last_speech_state:
            self._last_speech_state = dict(state)
            self._emit('speech_state', state)

    def _play(self):
        while not self._stop.is_set():
            if self.playback.run_next() is None:
                self._stop.wait(.02)

    def _on_playback(self, job):
        with self._lock:
            if job.state == 'playing':
                self._active_playback = job.task_id
                if self.session:
                    self.session.playback_started(job.task_id, now=self.clock())
            elif job.state in _TERMINAL and job.task_id == self._active_playback:
                if job.state != 'unknown':
                    self._active_playback = None
                    self._reference = None
                    self._echo_mode = 'half_duplex'
                    self._hold_until = self.clock() + self.config['speech']['echo_tail_s']
                    if self.echo:
                        self.echo.playback_finished(job.task_id)
                    if self.session and not (self._barge and self._barge['playback_id'] == job.task_id):
                        self.session.playback_finished(job.task_id, now=self.clock())
            self._emit('playback', {'state': job.state, 'task_id': self._active_playback or job.task_id,
                'speaking': self._active_playback is not None, 'volume': job.volume, 'error': job.error_code})
            self._publish_speech()

    def _on_reference(self, pcm, sample_rate, started_at):
        with self._lock:
            if sample_rate != 16000 or not self._active_playback:
                return
            self._reference = {'pcm': pcm, 'position': 0, 'sequence': 0,
                'epoch': uuid.uuid4().hex, 'playback_id': self._active_playback,
                'start_ns': time.time_ns() + int((started_at - self.clock()) * 1e9)}

    def _flush_reference(self):
        reference = self._reference
        if not reference:
            return
        while reference['position'] + 640 <= len(reference['pcm']):
            stamp = reference['start_ns'] + reference['sequence'] * 20_000_000
            if stamp > time.time_ns():
                break
            position = reference['position']
            chunk = AudioChunk(reference['epoch'], reference['sequence'], stamp,
                reference['pcm'][position:position + 640], discontinuity=reference['sequence'] == 0)
            self._emit('reference', chunk)
            if self.echo:
                self.echo.push_reference(chunk, playback_id=reference['playback_id'], actual_playback=True)
            reference['position'] += 640
            reference['sequence'] += 1

    def _begin_barge(self, event):
        if self._barge is not None or self.playback is None:
            return
        pending = {'event': event, 'playback_id': self._active_playback, 'chunks': [],
                   'overflow': False, 'started': self.clock()}
        self._barge = pending

        def cancel_then_listen():
            target = pending['playback_id']
            self.playback.cancel(target)
            end = time.monotonic() + 5
            job = self.playback.get(target)
            while not self._stop.is_set() and time.monotonic() < end and job and job.state not in _TERMINAL:
                self._stop.wait(.01)
                job = self.playback.get(target)
            with self._lock:
                if self._barge is not pending:
                    return
                self._barge = None
                stopped = job is not None and job.state in ('completed', 'cancelled')
                if stopped and not pending['overflow'] and not self._closed and self.session:
                    self._hold_until = 0
                    accepted = self.session.barge_in(event, lambda target: True, now=pending['started'])
                    if accepted:
                        for chunk, arrived_at in pending['chunks']:
                            self.session.feed(chunk, now=arrived_at)
                else:
                    self._emit('diagnostic', {'name': 'barge_in', 'state': 'cancel_not_confirmed'})
                    if stopped and self.session:
                        self.session.playback_finished(target, now=self.clock())
                self._publish_speech()
        thread = threading.Thread(target=cancel_then_listen, name='audio-barge-in', daemon=True)
        self._threads.append(thread)
        thread.start()

    def _enforce_quiet(self):
        inhibited = self.output_policy.inhibited
        if inhibited:
            for cancel in self._inflight.values():
                cancel.set()
        if inhibited != self._last_inhibited:
            self._last_inhibited = inhibited
            if self.playback is not None:
                self._pending_output('muted' if inhibited else 'idle', '')
                self._emit('diagnostic', {'name': 'output_policy',
                    'state': 'muted' if inhibited else 'enabled'})

    def set_muted(self, muted):
        with self._lock:
            try:
                self.output_policy.set_muted(muted)
            except ValueError:
                return {'success': False, 'message': 'boolean mute state required'}
            self._enforce_quiet()
        return {'success': True, 'message': 'output inhibited; active cancellation requested'
                if self.output_policy.inhibited else 'output enabled'}

    def set_listening(self, enabled):
        if type(enabled) is not bool:
            return {'success': False, 'message': 'boolean listening state required'}
        if enabled and self.source is None:
            return {'success': False, 'message': 'input is disabled in configuration'}
        with self._lock:
            self._listening = enabled
            self._stream_id = None
            if self.detector:
                self.detector.reset()
            if self.session:
                self.session.cancel()
            if self.echo:
                self.echo.reset()
            self._barge = None
        if self.source:
            if enabled:
                self.source.start()
            else:
                self.source.stop()
        self._emit('input_state', self._input_state())
        self._publish_speech()
        return {'success': True, 'message': 'enabled' if enabled else 'disabled'}

    def response_finished(self, session_id):
        with self._lock:
            if self.session is None or not session_id or session_id != self.session.session_id:
                return {'success': False, 'message': 'session does not match'}
            self.session.response_finished(session_id)
            self._publish_speech()
            return {'success': True, 'message': 'session completed'}

    def _pending_output(self, phase, task_id, error=''):
        with self._lock:
            if self._active_playback is None:
                self._emit('playback', {'state': phase, 'task_id': task_id,
                                       'speaking': False, 'error': error})

    def _synthesize(self, request, task_id, cancel, feedback, started):
        from .playback import AudioError
        feedback({'phase': 'waiting_for_tts', 'played_s': 0})
        while not self._tts_slot.acquire(timeout=.02):
            if cancel.is_set() or self._stop.is_set():
                raise AudioError('cancelled')
            if self.clock() - started >= request['max_queue_wait']:
                raise AudioError('queue_expired')
        try:
            if cancel.is_set() or self._stop.is_set():
                raise AudioError('cancelled')
            if self.clock() - started >= request['max_queue_wait']:
                raise AudioError('queue_expired')
            feedback({'phase': 'synthesizing', 'played_s': 0})
            self._pending_output('synthesizing', task_id)
            return self.tts.synthesize(request['text'], cancel_event=cancel)
        finally:
            self._tts_slot.release()

    def execute(self, kind, request, task_id, cancel, feedback):
        from .playback import AudioError
        fingerprint = hashlib.sha256(json.dumps([kind, request], sort_keys=True).encode()).hexdigest()
        with self._lock:
            if task_id in self._results:
                previous, result = self._results[task_id]
                return dict(result) if previous == fingerprint else {'error_code': 1, 'message': 'task ID conflict'}
            if task_id in self._inflight:
                return {'error_code': 2, 'message': 'task already in progress'}
            if self._closed or self.playback is None or not self.config['output']['enabled']:
                return {'error_code': 2, 'message': 'audio output is disabled'}
            if self.output_policy.inhibited:
                return {'error_code': 2, 'message': 'audio output is muted or in quiet hours'}
            if not 0 <= request.get('volume', .35) <= self.config['output']['max_volume']:
                return {'error_code': 1, 'message': 'volume exceeds policy'}
            self._inflight[task_id] = cancel
        result = {'error_code': 4, 'message': 'audio operation failed'}
        submitted = False
        started = self.clock()
        try:
            if cancel.is_set():
                raise AudioError('cancelled')
            if kind == 'say':
                if self.tts is None or not self.config['tts']['enabled'] or not self.config['tts']['allow_external']:
                    raise AudioError('configuration_required')
                if request.get('voice') and request['voice'] != self.config['tts']['voice']:
                    raise AudioError('unsupported_voice')
                data = self._synthesize(request, task_id, cancel, feedback, started)
                asset_id = self.playback.store.put(data).asset_id
            elif kind == 'play':
                asset_id = request['asset_id']
            else:
                raise AudioError('invalid_request')
            if cancel.is_set():
                raise AudioError('cancelled')
            remaining = request['max_queue_wait'] - (self.clock() - started)
            if remaining <= 0:
                raise AudioError('queue_expired')
            job = self.playback.submit(task_id, asset_id, volume=request['volume'],
                priority=request['priority'], max_queue_wait_s=remaining)
            submitted = True
            last_phase = None
            while job.state not in _TERMINAL:
                if cancel.is_set() or self._stop.is_set():
                    self.playback.cancel(task_id)
                if job.state != last_phase:
                    feedback({'phase': job.state, 'played_s': 0})
                    last_phase = job.state
                if self._stop.wait(.01):
                    result = {'error_code': 5, 'message': 'shutdown; playback outcome not confirmed'}
                    break
            else:
                codes = {'completed': 0, 'cancelled': 3, 'expired': 6, 'failed': 4, 'unknown': 5}
                result = {'error_code': codes[job.state], 'message': job.error_code or job.state}
        except AudioError as error:
            code = {'cancelled': 3, 'queue_expired': 6, 'configuration_required': 2}.get(error.code, 4)
            result = {'error_code': code, 'message': error.code}
        except Exception:
            result = {'error_code': 4, 'message': 'audio operation failed'}
        finally:
            if not submitted:
                phase = {3: 'cancelled', 6: 'expired'}.get(result['error_code'], 'failed')
                self._pending_output(phase, task_id, result['message'])
            with self._lock:
                self._inflight.pop(task_id, None)
                self._results[task_id] = (fingerprint, dict(result))
                while len(self._results) > 1024:
                    self._results.popitem(last=False)
        return result

    def close(self):
        if self._closed:
            return
        with self._lock:
            self._closed = True
            events = list(self._inflight.items())
            self._stop.set()
        for task_id, cancel in events:
            cancel.set()
            if self.playback:
                self.playback.cancel(task_id)
        if self.source:
            self.source.stop()
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=6)
        with self._lock:
            if self.session:
                self.session.close()
            if self.echo:
                self.echo.close()
            if self.detector:
                self.detector.reset()
