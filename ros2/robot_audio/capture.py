"""One authenticated OpenIPC audio source with bounded, fresh PCM delivery."""
import base64
from collections import deque
from dataclasses import dataclass, replace
import math
import netrc
import os
from pathlib import Path
import queue
import selectors
import stat
import subprocess
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

from .acoustics import analyze_pcm
from .types import AudioChunk


class CaptureError(Exception):
    def __init__(self, code):
        self.code = code
        super().__init__(f'audio capture: {code}')


def _validate_url(url):
    try:
        parsed = urllib.parse.urlsplit(url)
        if (parsed.scheme not in ('http', 'https') or not parsed.hostname or
                parsed.username is not None or parsed.password is not None or
                parsed.fragment or parsed.query or not 0 < (parsed.port or 80) <= 65535):
            raise ValueError
    except (ValueError, TypeError, AttributeError):
        raise CaptureError('invalid_url') from None
    return parsed


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def open_audio_stream(url, netrc_path, timeout=2):
    parsed = _validate_url(url)
    if not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    fd = None
    try:
        path = Path(netrc_path).expanduser()
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0))
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o600 or
                info.st_uid != os.getuid() or info.st_size > 65536):
            raise ValueError
        # Parsing the already-open inode avoids following a replaced credential path.
        entry = netrc.netrc(f'/proc/self/fd/{fd}').hosts.get(parsed.hostname)
        if not entry or not entry[0] or not entry[2]:
            raise ValueError
        authorization = base64.b64encode(f'{entry[0]}:{entry[2]}'.encode()).decode('ascii')
    except (OSError, ValueError, netrc.NetrcParseError, TypeError):
        raise CaptureError('credentials') from None
    finally:
        if fd is not None:
            os.close(fd)
    request = urllib.request.Request(url, headers={
        'Authorization': 'Basic ' + authorization,
        'Cache-Control': 'no-cache',
    })
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
    try:
        response = opener.open(request, timeout=timeout)
        if response.status != 200:
            response.close()
            raise CaptureError('http_error')
        return response
    except urllib.error.HTTPError as error:
        code = 'unauthorized' if error.code in (401, 403) else 'http_error'
        error.close()
        raise CaptureError(code) from None
    except (urllib.error.URLError, OSError, TimeoutError):
        raise CaptureError('disconnected') from None


@dataclass(frozen=True)
class CaptureConfig:
    url: str = 'http://cam/audio.opus'
    netrc_path: str = '~/.config/robot/camera.netrc'
    frame_ms: int = 20
    queue_size: int = 5
    connect_timeout: float = 2
    stall_timeout: float = 3
    max_reconnects: int = 5
    backoff_s: float = .25
    max_backoff_s: float = 4
    max_frame_age_s: float = .25
    ffmpeg: str = 'ffmpeg'

    def __post_init__(self):
        _validate_url(self.url)
        if not isinstance(self.frame_ms, int) or isinstance(self.frame_ms, bool) or self.frame_ms not in (10, 20, 40, 80):
            raise ValueError('frame_ms must be 10, 20, 40 or 80')
        if not isinstance(self.queue_size, int) or isinstance(self.queue_size, bool) or not 1 <= self.queue_size <= 256:
            raise ValueError('queue_size must be between 1 and 256')
        if not isinstance(self.max_reconnects, int) or isinstance(self.max_reconnects, bool) or not 0 <= self.max_reconnects <= 100:
            raise ValueError('max_reconnects must be between 0 and 100')
        for name in ('connect_timeout', 'stall_timeout', 'max_backoff_s', 'max_frame_age_s'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0 or value > 300:
                raise ValueError(f'{name} must be finite, positive and at most 300')
        if not isinstance(self.backoff_s, (int, float)) or isinstance(self.backoff_s, bool) or not math.isfinite(self.backoff_s) or self.backoff_s < 0:
            raise ValueError('backoff_s must be finite and nonnegative')
        if self.backoff_s > self.max_backoff_s:
            raise ValueError('backoff exceeds maximum')
        if not isinstance(self.ffmpeg, str) or not self.ffmpeg:
            raise ValueError('ffmpeg must name an executable')


class PCMFramer:
    def __init__(self, stream_id, frame_ms=20, stamp_ns=None):
        if frame_ms not in (10, 20, 40, 80):
            raise ValueError('unsupported frame duration')
        self.stream_id = stream_id
        self.frame_bytes = 16000 * frame_ms // 1000 * 2
        self.start_ns = stamp_ns
        self.sequence = 0
        self._buffer = bytearray()

    def feed(self, pcm):
        if not isinstance(pcm, bytes) or len(pcm) > 65536:
            raise ValueError('decoder block must be bytes of at most 65536 bytes')
        self._buffer.extend(pcm)
        if self.start_ns is None and len(self._buffer) >= self.frame_bytes:
            self.start_ns = time.time_ns() - self.frame_bytes // 2 * 1_000_000_000 // 16000
        chunks = []
        while len(self._buffer) >= self.frame_bytes:
            data = bytes(self._buffer[:self.frame_bytes])
            del self._buffer[:self.frame_bytes]
            offset = self.sequence * (self.frame_bytes // 2)
            chunks.append(AudioChunk(
                stream_id=self.stream_id, sequence=self.sequence,
                stamp_ns=self.start_ns + offset * 1_000_000_000 // 16000,
                pcm=data, discontinuity=self.sequence == 0,
                estimated_timestamp=True,
            ))
            self.sequence += 1
        return chunks


class LatestAudioQueue:
    def __init__(self, capacity, *, clock=time.monotonic):
        if not isinstance(capacity, int) or isinstance(capacity, bool) or capacity <= 0:
            raise ValueError('queue capacity must be positive')
        self.capacity = capacity
        self._clock = clock
        self._items = deque()
        self._condition = threading.Condition()
        self._gap = False
        self.dropped = 0

    def put(self, chunk):
        with self._condition:
            if len(self._items) == self.capacity:
                self._items.popleft()
                self.dropped += 1
                self._gap = True
            self._items.append((chunk, self._clock()))
            self._condition.notify()

    def clear(self):
        with self._condition:
            self.dropped += len(self._items)
            self._items.clear()
            self._gap = True
            self._condition.notify_all()

    def get(self, timeout=None, *, max_age_s=None):
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            raise ValueError('invalid timeout')
        if max_age_s is not None and (not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError('invalid maximum frame age')
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while True:
                while self._items:
                    chunk, inserted = self._items.popleft()
                    if max_age_s is not None and self._clock() - inserted > max_age_s:
                        self.dropped += 1
                        self._gap = True
                        continue
                    if self._gap:
                        chunk = replace(chunk, discontinuity=True)
                        self._gap = False
                    return chunk
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise queue.Empty
                self._condition.wait(remaining)


class FFmpegDecoder:
    def __init__(self, executable='ffmpeg', stall_timeout=3):
        self.stall_timeout = stall_timeout
        args = [executable, '-nostdin', '-hide_banner', '-loglevel', 'error',
                '-max_alloc', '8388608', '-probesize', '32', '-analyzeduration', '0',
                '-protocol_whitelist', 'pipe', '-f', 'ogg', '-blocksize', '4096',
                '-i', 'pipe:0', '-map', '0:a:0', '-vn', '-threads', '1',
                '-ac', '1', '-ar', '16000', '-c:a', 'pcm_s16le', '-f', 's16le', 'pipe:1']
        try:
            self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL, start_new_session=True)
        except OSError:
            raise CaptureError('decoder_unavailable') from None

    def feed(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def end_input(self):
        try:
            self.process.stdin.close()
        except (OSError, ValueError):
            pass

    def pcm_blocks(self, stop):
        fd = self.process.stdout.fileno()
        os.set_blocking(fd, False)
        last_data = time.monotonic()
        with selectors.DefaultSelector() as selector:
            selector.register(fd, selectors.EVENT_READ)
            while not stop.is_set():
                ready = selector.select(.1)
                if not ready:
                    if time.monotonic() - last_data > self.stall_timeout:
                        raise CaptureError('stalled')
                    continue
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    try:
                        result = self.process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        raise CaptureError('decoder_error') from None
                    if result != 0 and not stop.is_set():
                        raise CaptureError('decoder_error')
                    return
                last_data = time.monotonic()
                yield data

    def interrupt(self):
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except ProcessLookupError:
                pass

    def close(self):
        self.interrupt()
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=1)
        for pipe in (self.process.stdin, self.process.stdout):
            try:
                pipe.close()
            except (OSError, ValueError):
                pass


class CameraAudioSource:
    def __init__(self, config=None, *, opener=open_audio_stream, decoder_factory=FFmpegDecoder):
        self.config = config or CaptureConfig()
        self._opener = opener
        self._decoder_factory = decoder_factory
        self._queue = LatestAudioQueue(self.config.queue_size)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._decoder = None
        self._state = 'stopped'
        self._error = None
        self._last_failure = None
        self._last_frame_at = None
        self._stream_id = None
        self._reconnects = 0
        self._level = None
        self._clipping = 0

    def _set_state(self, state, error=None):
        with self._lock:
            self._state = state
            self._error = error

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._queue.clear()
        with self._lock:
            self._last_failure = None
            self._last_frame_at = None
            self._stream_id = None
            self._reconnects = 0
        self._set_state('connecting')
        self._thread = threading.Thread(target=self._run, name='camera-audio', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        with self._lock:
            decoder = self._decoder
        if decoder is not None:
            decoder.interrupt()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(self.config.connect_timeout + 3)
            if self._thread.is_alive():
                self._set_state('error', 'stop_timeout')
                return
        self._queue.clear()
        self._set_state('stopped')

    def get_chunk(self, timeout=None):
        return self._queue.get(timeout, max_age_s=self.config.max_frame_age_s)

    def state(self):
        with self._lock:
            age = None if self._last_frame_at is None else max(0, time.monotonic() - self._last_frame_at)
            state = self._state
            if state == 'streaming' and age is not None and age > self.config.stall_timeout:
                state = 'stalled'
            return {
                'source': 'openipc_camera', 'state': state,
                'error': self._error, 'last_failure': self._last_failure,
                'last_frame_age_s': age, 'stream_id': self._stream_id,
                'reconnects': self._reconnects, 'dropped_frames': self._queue.dropped,
                'sample_rate': 16000, 'channels': 1, 'sample_format': 's16le',
                'estimated_timestamp': True, 'signal_dbfs': self._level,
                'clipping_fraction': self._clipping, 'microphone_verified': False,
            }

    def _run(self):
        for attempt in range(self.config.max_reconnects + 1):
            if self._stop.is_set():
                return
            if attempt:
                with self._lock:
                    self._reconnects = attempt
                delay = min(self.config.max_backoff_s, self.config.backoff_s * 2 ** (attempt - 1))
                self._set_state('reconnecting')
                if self._stop.wait(delay):
                    return
            self._set_state('connecting')
            try:
                self._session()
                failure = 'disconnected'
            except CaptureError as error:
                failure = error.code
            except (OSError, ValueError, subprocess.SubprocessError):
                failure = 'decoder_error'
            if self._stop.is_set():
                return
            with self._lock:
                self._last_failure = failure
            self._set_state('stalled' if failure == 'stalled' else 'disconnected', failure)
        self._set_state('error', 'reconnect_exhausted')

    def _session(self):
        response = self._opener(self.config.url, self.config.netrc_path, timeout=self.config.connect_timeout)
        decoder = None
        feeder = None
        feed_failure = []
        session_stop = threading.Event()
        try:
            decoder = self._decoder_factory(self.config.ffmpeg, stall_timeout=self.config.stall_timeout)
            with self._lock:
                self._decoder = decoder
                self._stream_id = uuid.uuid4().hex
                self._last_frame_at = None
                stream_id = self._stream_id
            self._queue.clear()
            framer = PCMFramer(stream_id, frame_ms=self.config.frame_ms)

            def feed():
                try:
                    while not self._stop.is_set() and not session_stop.is_set():
                        data = response.read1(4096)
                        if not data:
                            break
                        decoder.feed(data)
                except (TimeoutError, OSError, ValueError, AttributeError):
                    # AttributeError: teardown closed the response under this
                    # read (fp becomes None) — that is a stop, not a failure.
                    if not self._stop.is_set() and not session_stop.is_set():
                        feed_failure.append(CaptureError('stalled'))
                finally:
                    decoder.end_input()

            feeder = threading.Thread(target=feed, name='camera-audio-http', daemon=True)
            feeder.start()
            for data in decoder.pcm_blocks(self._stop):
                for chunk in framer.feed(data):
                    level = analyze_pcm(chunk.pcm)
                    with self._lock:
                        self._last_frame_at = time.monotonic()
                        self._level = level['ac_dbfs']
                        self._clipping = level['clipping_fraction']
                    self._queue.put(chunk)
                    self._set_state('streaming')
        except CaptureError:
            if feed_failure:
                raise feed_failure[0] from None
            raise
        finally:
            session_stop.set()
            # Stop the reader before closing the socket it reads from:
            # join first, then close, so read1 never races response.close().
            if feeder is not None:
                session_stop.set()
                feeder.join(self.config.connect_timeout + .2)
            if decoder is not None:
                decoder.close()
            response.close()
            with self._lock:
                self._decoder = None
        if feed_failure:
            raise feed_failure[0]
