"""Bounded, independently cancellable speaker playback without ROS imports."""
from dataclasses import dataclass, field
import hashlib
import heapq
import io
import math
import os
from pathlib import Path
import re
import selectors
import stat
import subprocess
import threading
import time
import uuid
import wave


class AudioError(Exception):
    def __init__(self, code, message=None):
        self.code = code
        super().__init__(message or code)


def validate_wav(data, max_bytes=2_000_000, max_duration_s=30):
    if not isinstance(data, bytes) or len(data) > max_bytes:
        raise AudioError('invalid_audio')
    try:
        with wave.open(io.BytesIO(data), 'rb') as audio:
            rate, channels, width = audio.getframerate(), audio.getnchannels(), audio.getsampwidth()
            frames = audio.getnframes()
            if (rate, channels, width, audio.getcomptype()) != (16000, 1, 2, 'NONE'):
                raise AudioError('unsupported_audio_format')
            if not 0 < frames <= rate * max_duration_s:
                raise AudioError('invalid_audio_duration')
            pcm = audio.readframes(frames)
            if len(pcm) != frames * 2:
                raise AudioError('truncated_audio')
            return pcm, rate, frames / rate
    except (wave.Error, EOFError, ValueError) as exc:
        raise AudioError('invalid_audio') from None


def normalize_wav(data, max_bytes=2_000_000, max_duration_s=30):
    if not isinstance(data, bytes) or len(data) > max_bytes:
        raise AudioError('invalid_audio')
    try:
        with wave.open(io.BytesIO(data), 'rb') as audio:
            rate, channels, width, count = audio.getframerate(), audio.getnchannels(), audio.getsampwidth(), audio.getnframes()
            if not 8000 <= rate <= 96000 or channels not in (1, 2) or width not in (1, 2, 3, 4) or audio.getcomptype() != 'NONE':
                raise AudioError('unsupported_audio_format')
            if not 0 < count <= rate * max_duration_s or len(audio.readframes(count)) != count * channels * width:
                raise AudioError('invalid_audio_duration')
    except (wave.Error, EOFError, ValueError):
        raise AudioError('invalid_audio') from None
    if (rate, channels, width) == (16000, 1, 2):
        validate_wav(data, max_bytes, max_duration_s)
        return data
    try:
        converted = subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
                    '-protocol_whitelist', 'pipe', '-f', 'wav', '-i', 'pipe:0', '-vn',
                    '-t', str(max_duration_s), '-ac', '1', '-ar', '16000', '-f', 's16le', 'pipe:1'],
                    input=data, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=5, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        raise AudioError('audio_conversion_failed') from None
    result = io.BytesIO()
    with wave.open(result, 'wb') as audio:
        audio.setnchannels(1); audio.setsampwidth(2); audio.setframerate(16000); audio.writeframes(converted)
    normalized = result.getvalue()
    validate_wav(normalized, max_bytes, max_duration_s)
    return normalized


@dataclass(frozen=True)
class AudioAsset:
    asset_id: str
    path: Path
    pcm: bytes
    sample_rate: int
    duration_s: float


class AssetStore:
    def __init__(self, root, max_bytes=2_000_000, max_duration_s=30, ttl_s=3600, clock=time.time):
        self.root = Path(root).expanduser()
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        if self.root.is_symlink() or not self.root.is_dir():
            raise AudioError('unsafe_asset_store')
        self.root.chmod(0o700)
        self.max_bytes, self.max_duration_s, self.ttl_s, self.clock = max_bytes, max_duration_s, ttl_s, clock
        self._created = {}
        self._lock = threading.RLock()

    def purge(self):
        removed = 0
        with self._lock:
            for path in self.root.glob('*.wav'):
                if not re.fullmatch('[a-f0-9]{64}', path.stem):
                    continue
                try:
                    metadata = path.lstat()
                    created = self._created.get(path.stem, metadata.st_mtime)
                    if stat.S_ISREG(metadata.st_mode) and metadata.st_uid == os.getuid() and self.clock() - created > self.ttl_s:
                        path.unlink(); self._created.pop(path.stem, None); removed += 1
                except FileNotFoundError:
                    continue
        return removed

    def put(self, data):
        self.purge()
        pcm, rate, duration = validate_wav(data, self.max_bytes, self.max_duration_s)
        asset_id = hashlib.sha256(data).hexdigest()
        path = self.root / (asset_id + '.wav')
        with self._lock:
            if path.exists() or path.is_symlink():
                if path.is_symlink() or not path.is_file():
                    raise AudioError('unsafe_asset')
                self.get(asset_id)
            else:
                try:
                    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
                    with os.fdopen(fd, 'wb') as f:
                        f.write(data)
                except OSError:
                    raise AudioError('asset_store_failed') from None
            self._created[asset_id] = self.clock()
        return AudioAsset(asset_id, path, pcm, rate, duration)

    def get(self, asset_id):
        if not isinstance(asset_id, str) or not re.fullmatch('[a-f0-9]{64}', asset_id):
            raise AudioError('invalid_asset_id')
        path = self.root / (asset_id + '.wav')
        with self._lock:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
                with os.fdopen(fd, 'rb') as f:
                    st = os.fstat(f.fileno())
                    created = self._created.get(asset_id, st.st_mtime)
                    if self.clock() - created > self.ttl_s:
                        self.purge()
                        raise AudioError('asset_expired')
                    if not stat.S_ISREG(st.st_mode) or st.st_size > self.max_bytes or st.st_uid != os.getuid() or st.st_mode & 0o077:
                        raise AudioError('unsafe_asset')
                    data = f.read(self.max_bytes + 1)
            except OSError:
                raise AudioError('asset_unavailable') from None
        if hashlib.sha256(data).hexdigest() != asset_id:
            raise AudioError('invalid_asset')
        pcm, rate, duration = validate_wav(data, self.max_bytes, self.max_duration_s)
        return AudioAsset(asset_id, path, pcm, rate, duration)


@dataclass(frozen=True)
class TransportResult:
    outcome: str
    message: str = ''


@dataclass
class PlaybackJob:
    task_id: str
    asset_id: str
    volume: float
    priority: int
    created_at: float
    max_queue_wait_s: float
    state: str = 'queued'
    error_code: str = ''
    cancel_event: threading.Event = field(default_factory=threading.Event, repr=False)


class PlaybackQueue:
    def __init__(self, store, transport, max_queue=8, clock=time.monotonic, on_state=None, on_reference=None,
                 history_limit=1024, history_ttl_s=300):
        if type(history_limit) is not int or not 1 <= history_limit <= 65536:
            raise ValueError('invalid playback history limit')
        if not isinstance(history_ttl_s, (int, float)) or not math.isfinite(history_ttl_s) or not 0 < history_ttl_s <= 86400:
            raise ValueError('invalid playback history retention')
        self.store, self.transport, self.max_queue, self.clock = store, transport, max_queue, clock
        self.history_limit, self.history_ttl_s = history_limit, history_ttl_s
        self.on_state, self.on_reference = on_state, on_reference
        self._jobs, self._pending, self._counter = {}, [], 0
        self._inflight = set()
        self._lock = threading.RLock()
        self._runner = threading.Lock()

    def _notify(self, job):
        if self.on_state:
            self.on_state(job)

    def submit(self, task_id, asset_id, volume=.35, priority=0, max_queue_wait_s=5):
        if not isinstance(task_id, str) or not re.fullmatch('[A-Za-z0-9_.:-]{1,128}', task_id):
            raise AudioError('invalid_task_id')
        if not isinstance(volume, (int, float)) or not math.isfinite(volume) or not 0 <= volume <= 1:
            raise AudioError('invalid_volume')
        if not isinstance(priority, int) or not 0 <= priority <= 255:
            raise AudioError('invalid_priority')
        if not isinstance(max_queue_wait_s, (int, float)) or not math.isfinite(max_queue_wait_s) or not 0 < max_queue_wait_s <= 300:
            raise AudioError('invalid_queue_timeout')
        self.store.get(asset_id)
        with self._lock:
            if task_id in self._jobs:
                previous = self._jobs[task_id]
                if (previous.asset_id, previous.volume, previous.priority, previous.max_queue_wait_s) != (asset_id, volume, priority, max_queue_wait_s):
                    raise AudioError('task_id_conflict')
                return previous
            if sum(job.state == 'queued' for job in self._jobs.values()) >= self.max_queue:
                raise AudioError('queue_full')
            if len(self._jobs) >= self.history_limit:
                expired = [key for key, old in self._jobs.items()
                           if old.state not in ('queued', 'playing') and key not in self._inflight
                           and self.clock() - old.created_at > self.history_ttl_s]
                for key in expired:
                    del self._jobs[key]
                self._pending = [entry for entry in self._pending if entry[2] in self._jobs]
                heapq.heapify(self._pending)
            if len(self._jobs) >= self.history_limit:
                raise AudioError('task_history_full')
            job = PlaybackJob(task_id, asset_id, volume, priority, self.clock(), max_queue_wait_s)
            self._jobs[task_id] = job
            self._counter += 1
            heapq.heappush(self._pending, (-priority, self._counter, task_id))
            self._notify(job)
            return job

    def get(self, task_id):
        with self._lock:
            return self._jobs.get(task_id)

    def cancel(self, task_id):
        with self._lock:
            job = self._jobs.get(task_id)
            if not job or job.state not in ('queued', 'playing'):
                return False
            job.cancel_event.set()
            if job.state == 'queued' and task_id not in self._inflight:
                job.state = 'cancelled'; self._notify(job)
            return True

    def run_next(self):
        if not self._runner.acquire(blocking=False):
            return None
        active_id = None
        try:
            with self._lock:
                while self._pending:
                    _, _, task_id = heapq.heappop(self._pending)
                    job = self._jobs[task_id]
                    if job.state != 'queued':
                        continue
                    if self.clock() - job.created_at > job.max_queue_wait_s:
                        job.state = 'expired'; job.error_code = 'queue_expired'; self._notify(job)
                        return job
                    active_id = task_id
                    self._inflight.add(task_id)
                    break
                else:
                    return None
            try:
                asset = self.store.get(job.asset_id)
                if job.cancel_event.is_set():
                    result = TransportResult('cancelled')
                else:
                    def started():
                        job.state = 'playing'; self._notify(job)
                    result = self.transport.play(asset, job.volume, job.cancel_event,
                                                 on_started=started, on_reference=self.on_reference)
                job.state = {'completed': 'completed', 'cancelled': 'cancelled',
                             'unknown': 'unknown'}.get(result.outcome, 'failed')
                job.error_code = '' if result.outcome == 'completed' else result.outcome
            except AudioError as exc:
                job.state, job.error_code = 'failed', exc.code
            except Exception:
                job.state, job.error_code = 'unknown', 'transport_failed'
            self._notify(job)
            return job
        finally:
            with self._lock:
                if active_id is not None:
                    self._inflight.discard(active_id)
            self._runner.release()


class SSHPlaybackTransport:
    def __init__(self, host='robot', helper_path='/tmp/robot-audio/robot-audio', ssh='ssh', timeout_s=35):
        if not isinstance(host, str) or not re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', host):
            raise AudioError('invalid_ssh_host')
        if not isinstance(helper_path, str) or not re.fullmatch('/[A-Za-z0-9_./-]+', helper_path) or '..' in helper_path.split('/'):
            raise AudioError('invalid_helper_path')
        self.host, self.helper_path, self.ssh, self.timeout_s = host, helper_path, ssh, timeout_s

    def play(self, asset, volume, cancel_event, on_started=None, on_reference=None):
        if cancel_event.is_set():
            return TransportResult('cancelled')
        token = uuid.uuid4().hex
        base = [self.ssh, '-T', '-oBatchMode=yes', '-oConnectTimeout=5',
                '-oServerAliveInterval=2', '-oServerAliveCountMax=2', self.host]
        command = base + [f'{self.helper_path} play {float(volume):.6f} {token}']
        started, completed, cancelled, busy = False, False, False, False
        cancellation_sent = False
        deadline = time.monotonic() + min(self.timeout_s, asset.duration_s + 10)
        try:
            with asset.path.open('rb') as audio:
                process = subprocess.Popen(command, stdin=audio, stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL, start_new_session=True)
            selector = selectors.DefaultSelector(); selector.register(process.stdout, selectors.EVENT_READ)
            buffer = b''
            try:
                while time.monotonic() < deadline:
                    if cancel_event.is_set() and process.poll() is None and not cancellation_sent:
                        cancellation_sent = True
                        acknowledgement = subprocess.run(base + [f'{self.helper_path} cancel {token}'],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    timeout=4)
                        cancelled = acknowledgement.returncode == 0 and b'CANCELLED' in acknowledgement.stdout
                        if not cancelled:
                            process.terminate()
                    events = selector.select(.05)
                    for key, _ in events:
                        data = os.read(key.fileobj.fileno(), 4096)
                        if not data:
                            selector.unregister(key.fileobj); continue
                        buffer += data
                        if len(buffer) > 16384:
                            raise AudioError('invalid_helper_response')
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            if line == b'STARTED' and not started:
                                started = True
                                if on_started: on_started()
                                if on_reference: on_reference(asset.pcm, asset.sample_rate, time.monotonic())
                            elif line == b'COMPLETED': completed = True
                            elif line == b'CANCELLED': cancelled = True
                            elif line == b'BUSY': busy = True
                    if process.poll() is not None and not selector.get_map():
                        break
                if process.poll() is None:
                    process.kill(); process.wait(timeout=2)
                    return TransportResult('unknown', 'playback outcome not confirmed')
                if completed and process.returncode == 0 and started:
                    return TransportResult('completed')
                if cancelled:
                    return TransportResult('cancelled')
                if busy:
                    return TransportResult('busy')
                return TransportResult('unknown' if started or cancel_event.is_set() else 'error')
            finally:
                selector.close(); process.stdout.close()
                if process.poll() is None:
                    process.kill(); process.wait(timeout=2)
        except (OSError, subprocess.SubprocessError):
            return TransportResult('unknown' if started else 'error')
