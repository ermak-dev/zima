"""Loopback-only native audio IPC. No device access until explicit service start."""
import base64
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
import re
import signal
import threading
import time
import uuid
import wave

from .config import validate_config
from .validation import validate_chunk

MAX_BODY = 65536


def _number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('numeric argument outside permitted range')
    return value


def _identifier(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}', value) and '..' not in value


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class AudioService:
    def __init__(self, config=None, *, runtime=None, battery_reader=None):
        self.config = validate_config(config or {})
        if runtime is None:
            from .runtime import AudioRuntime
            runtime = AudioRuntime(self.config)
        self.runtime = runtime
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._closed = self._started = False
        self._jobs = {}
        self._record = None
        self.recording = threading.Event()
        self._state = {'input': {'state': 'disabled', 'microphone_verified': False},
                       'output': {'state': 'disabled', 'speaking': False},
                       'speech': {'state': 'disabled'}, 'diagnostics': {}}
        self._controller = None
        if self.config['controller']['enabled']:
            from .controller import VoiceController, RobotBatteryReader
            cfg = self.config['controller']
            self._controller = VoiceController(battery_reader or RobotBatteryReader(), self._speak_reply,
                runtime.response_finished, allow_reported=cfg['allow_reported_battery'], max_queue_s=cfg['max_queue_s'])
        runtime.set_callbacks(input_state=lambda value: self._update('input', value),
            playback=lambda value: self._update('output', value),
            speech_state=lambda value: self._update('speech', value),
            diagnostic=self._diagnostic, transcript=self._transcript, chunk=self._chunk)

    def start(self):
        with self._lock:
            if self._started or self._closed:
                return
            self._started = True
        if self._controller:
            self._controller.start()
        self.runtime.start()

    def _update(self, key, value):
        with self._lock:
            self._state[key] = _json_safe(dict(value))

    def _diagnostic(self, value):
        with self._lock:
            self._state['diagnostics'][str(value.get('name', 'audio'))] = str(value.get('state', 'unknown'))

    def _transcript(self, value):
        self._update('transcript', value)
        if self._controller:
            self._controller.submit(value)

    def _speak_reply(self, text, session_id, cancel):
        cfg = self.config['controller']
        asset = cfg['reply_cache'].get(text)
        if not asset and cfg['cache_only']:
            self._diagnostic({'name': 'voice_controller', 'state': 'reply_cache_miss'})
            return
        request = {'command': 'play' if asset else 'say', 'volume': self.config['output']['volume'],
                   'task_id': 'voice-' + hashlib.sha256(session_id.encode()).hexdigest(),
                   'ttl': 30.0, 'timeout': 180}
        request.update({'asset_id': asset} if asset else {'text': text})
        return self._execute(request, cancel=cancel)

    def request(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('request must be a JSON object')
        command = payload.get('command')
        allowed = {'status': set(), 'listen': {'enabled'}, 'mute': {'enabled'},
                   'finish': {'session_id'}, 'utterance': set(), 'start': set(),
                   'stop': {'task_id'}, 'record': {'seconds'},
                   'play': {'asset_id', 'volume', 'priority', 'ttl', 'task_id'},
                   'say': {'text', 'voice', 'volume', 'priority', 'ttl', 'task_id'}}
        if not isinstance(command, str) or command not in allowed:
            raise ValueError('unknown audio command')
        if set(payload) - allowed[command] - {'command', 'timeout'}:
            raise ValueError('unknown audio request field')
        timeout = _number(payload.get('timeout', 45 if command in ('play', 'say') else 5), .01, 180)
        with self._lock:
            if self._closed:
                raise ValueError('audio service is closed')
        if command == 'status':
            with self._lock:
                return copy.deepcopy(self._state)
        if command in ('listen', 'mute'):
            enabled = payload.get('enabled')
            if type(enabled) is not bool:
                raise ValueError('enabled must be boolean')
            if command == 'listen':
                if not enabled:
                    with self._condition:
                        self._abort_record('listening disabled')
                        self._state.pop('transcript', None)
                return self.runtime.set_listening(enabled)
            return self.runtime.set_muted(enabled)
        if command == 'finish':
            if not _identifier(payload.get('session_id')):
                raise ValueError('invalid session ID')
            return self.runtime.response_finished(payload['session_id'])
        if command in ('utterance', 'start'):
            return self.runtime.begin_utterance()
        if command == 'stop':
            task_id = payload.get('task_id')
            if task_id is not None and not _identifier(task_id):
                raise ValueError('invalid task ID')
            with self._lock:
                events = [event for key, (event, _) in self._jobs.items() if task_id is None or key == task_id]
                for event in events:
                    event.set()
            return {'requested': len(events)}
        if command == 'record':
            seconds = _number(payload.get('seconds', 6), .1, 30)
            return self._record_wav(seconds, timeout)
        return self._execute(payload)

    def _execute(self, payload, *, cancel=None):
        kind = payload['command']
        volume = _number(payload.get('volume', .35), 0, self.config['output']['max_volume'])
        priority = payload.get('priority', 0)
        if type(priority) is not int or not 0 <= priority <= 2:
            raise ValueError('priority must be 0, 1 or 2')
        ttl = _number(payload.get('ttl', 5), .01, 60)
        timeout = _number(payload.get('timeout', 45), .01, 180)
        task_id = payload.get('task_id', uuid.uuid4().hex)
        if not _identifier(task_id):
            raise ValueError('invalid task ID')
        request = {'volume': volume, 'priority': priority, 'max_queue_wait': ttl}
        if kind == 'play':
            asset = payload.get('asset_id')
            if not _identifier(asset) or ':' in asset:
                raise ValueError('play requires a prepared asset identifier')
            request['asset_id'] = asset
        else:
            text, voice = payload.get('text'), payload.get('voice', '')
            if (not isinstance(text, str) or not text.strip() or '\0' in text or
                    len(text) > self.config['tts']['max_text_chars'] or
                    not isinstance(voice, str) or len(voice) > 128 or '\0' in voice):
                raise ValueError('invalid speech text or voice')
            request.update(text=text, voice=voice)
        cancel = cancel if cancel is not None else threading.Event()
        done, result = threading.Event(), {}
        def run():
            try:
                result.update(self.runtime.execute(kind, request, task_id, cancel, lambda value: None))
            except Exception:
                result.update(error_code=4, message='audio operation failed')
            finally:
                with self._lock:
                    self._jobs.pop(task_id, None)
                done.set()
        worker = threading.Thread(target=run, name='native-audio-operation', daemon=True)
        with self._lock:
            if self._closed or len(self._jobs) >= self.config['output']['queue_size'] + 1:
                return {'error_code': 2, 'message': 'audio service busy or closed', 'task_id': task_id}
            if task_id in self._jobs:
                return {'error_code': 2, 'message': 'task already in progress', 'task_id': task_id}
            self._jobs[task_id] = (cancel, worker)
            worker.start()
        if not done.wait(timeout):
            cancel.set()
            # Cancellation is a request, not proof the speaker stopped.
            if not done.wait(6):
                return {'error_code': 5, 'message': 'timeout; playback outcome not confirmed', 'task_id': task_id}
        return {**result, 'task_id': task_id}

    def _abort_record(self, reason):
        if self._record is not None:
            self._record['error'] = reason
            self._record['pcm'].clear()
            self._condition.notify_all()

    def _chunk(self, chunk):
        with self._condition:
            record = self._record
            if record is None or record['error'] or len(record['pcm']) >= record['bytes']:
                return
            try:
                validate_chunk(chunk)
                previous = record['previous']
                if previous is not None and (chunk.stream_id != previous.stream_id or
                        chunk.sequence != previous.sequence + 1 or chunk.discontinuity):
                    raise ValueError('audio gap')
                remaining = record['bytes'] - len(record['pcm'])
                record['pcm'].extend(chunk.pcm[:remaining])
                record['previous'] = chunk
                self._condition.notify_all()
            except (ValueError, TypeError, AttributeError):
                self._abort_record('audio gap or invalid format')

    def _record_wav(self, seconds, timeout):
        record = {'pcm': bytearray(), 'bytes': round(seconds * 16000) * 2, 'previous': None, 'error': ''}
        with self._condition:
            if self._record is not None:
                raise ValueError('recording already active')
            if self._closed:
                raise ValueError('audio service closed')
            self._record = record
            self.recording.set()
            try:
                complete = self._condition.wait_for(lambda: record['error'] or
                    len(record['pcm']) == record['bytes'], timeout)
                if record['error']:
                    raise ValueError(record['error'])
                if not complete:
                    raise TimeoutError('audio source did not provide a complete recording')
                output = io.BytesIO()
                with wave.open(output, 'wb') as writer:
                    writer.setnchannels(1)
                    writer.setsampwidth(2)
                    writer.setframerate(16000)
                    writer.writeframes(record['pcm'])
                return {'wav_base64': base64.b64encode(output.getvalue()).decode('ascii'),
                        'frames': len(record['pcm']) // 2, 'sample_rate': 16000,
                        'channels': 1, 'sample_format': 's16le'}
            finally:
                record['pcm'].clear()
                self._record = None
                self.recording.clear()

    def close(self):
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._abort_record('audio service closed')
            jobs = list(self._jobs.values())
            for cancel, _ in jobs:
                cancel.set()
        try:
            if self._controller:
                self._controller.close()
        finally:
            self.runtime.close()
            for _, worker in jobs:
                if worker is not threading.current_thread():
                    worker.join(timeout=6)


class AudioRequestHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, *args):
        pass  # Neither speech text nor local filenames belong in access logs.

    def _reply(self, status, value):
        body = json.dumps(_json_safe(value), ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(body)
        self.close_connection = True

    def do_GET(self):
        if self.path != '/status':
            return self._reply(404, {'error': 'unknown audio route'})
        self._dispatch({'command': 'status'})

    def do_POST(self):
        if self.path != '/request':
            return self._reply(404, {'error': 'unknown audio route'})
        # No CORS; reject browser form requests and origins to reduce loopback CSRF.
        if self.headers.get('Origin') or self.headers.get('Transfer-Encoding'):
            return self._reply(400, {'error': 'unsupported request headers'})
        raw_length = self.headers.get('Content-Length')
        if raw_length is None:
            return self._reply(411, {'error': 'content length required'})
        try:
            length = int(raw_length)
            if length < 0:
                raise ValueError
        except ValueError:
            return self._reply(400, {'error': 'invalid content length'})
        if length > MAX_BODY:
            return self._reply(413, {'error': 'audio request too large'})
        try:
            data = self.rfile.read(length)
            if len(data) != length:
                raise ValueError
            payload = json.loads(data, parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
            if not isinstance(payload, dict):
                raise ValueError
        except (ValueError, UnicodeError):
            return self._reply(400, {'error': 'invalid JSON object'})
        self._dispatch(payload)

    def _dispatch(self, payload):
        try:
            value = self.server.service.request(payload)
        except (ValueError, TypeError):
            return self._reply(400, {'error': 'invalid audio request or unavailable state'})
        except TimeoutError:
            return self._reply(504, {'error': 'audio operation timed out; no automatic retry'})
        except Exception:
            return self._reply(500, {'error': 'audio operation failed'})
        self._reply(200, value)


def create_server(service, *, host='127.0.0.1', port=8086):
    if host != '127.0.0.1' or type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('audio IPC requires IPv4 loopback and port 1..65535')
    server = ThreadingHTTPServer((host, port), AudioRequestHandler)
    server.daemon_threads = True
    server.service = service
    return server


def run_foreground(config=None, *, port=8086):
    service = AudioService(config)
    server = None
    previous = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    try:
        server = create_server(service, port=port)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, interrupted)
        service.start()
        server.serve_forever(poll_interval=.1)
    finally:
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        try:
            try:
                service.close()
            finally:
                if server is not None:
                    server.server_close()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
