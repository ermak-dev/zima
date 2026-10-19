"""Explicit microphone power/gain contexts; importing this module does no I/O.

Use ``with CameraSession(explicit=True, gain=CameraGainLease(ssh)) as session:``
and call ``session.check()`` frequently while supervising the audio service.
``remaining_s`` includes startup; defaults are 600 s overall, 60 s startup,
maximum explicit TTL 3600 s. This module never starts audio or sends velocity.

SIGINT/SIGTERM unwind contexts on the main thread. Cleanup has its own bounded
budget and failures are errors, not ready states. Host SIGKILL/power loss cannot
guarantee stock enable: the existing mask-owner guard restores only standby.
The independent camera guard can restore gain without a surviving host.

On unverified gain cleanup a surviving session keeps LDS powered, checks hold
liveness and retries until at most TTL + 10 s after the start request returned
(including unknown outcomes). This grace is independent of the outer session
TTL: it gives the camera guard its remaining TTL and a bounded restore chance,
not proof of restoration. Hold death aborts the grace. The guard must have been
launched and the camera must remain operational; stuck remote commands, delayed
remote launch after transport loss, hardware/power failure or failed YAML/HUP
cannot be guaranteed away. Unverified cleanup still releases hold and attempts
stock restore/enable/status, then raises; it never silently keeps stock disabled.
After gain cleanup, hold termination is bounded by 5 + 2 s and each of the three
robot cleanup commands by 15 s. An unreachable robot can only be reported, not
claimed enabled. A standalone gain lease cannot promise power and does not wait
out a failed restore; only CameraSession supplies the powered recovery grace.
"""
import fcntl
import math
import netrc
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import tempfile
import threading
import time
import uuid

from .capture import CaptureError, open_audio_stream


class CameraSessionError(RuntimeError):
    """Sanitized error; never contains remote output, credentials or scripts."""


def _integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in {low}..{high}')
    return value


def _seconds(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 3600:
        raise ValueError(f'{name} must be finite in (0, 3600]')
    return value


def _volume(text):
    if not isinstance(text, str) or not re.fullmatch(r'(?:0|[1-9][0-9]?|100)\n?', text):
        raise CameraSessionError('invalid_volume_readback')
    return int(text)


class _Signals:
    def __init__(self): self.previous = {}
    def install(self):
        if threading.current_thread() is not threading.main_thread():
            raise CameraSessionError('session_requires_main_thread')
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.previous[sig] = signal.getsignal(sig)
            signal.signal(sig, self.interrupt)
    @staticmethod
    def interrupt(sig, frame):
        raise CameraSessionError('session_interrupted')
    def block(self):
        for sig in self.previous: signal.signal(sig, signal.SIG_IGN)
    def restore(self):
        for sig, handler in self.previous.items(): signal.signal(sig, handler)
        self.previous.clear()


class CameraSession:
    """Own LDS power only after explicit consent and enabled stock controller.

    Single-use, main-thread context. ``check`` raises on TTL or hold death; the
    caller must leave the context on failure (normally simply let it propagate).
    Startup waits for a holding ACK, not just an alive ssh process. No local
    background thread silently takes over the caller's audio lifecycle.
    """
    def __init__(self, *, explicit=False, ttl_s=600, startup_s=60, gain=None,
                 runner=subprocess, clock=time.monotonic, sleep=time.sleep,
                 killpg=os.killpg, lock_path=None):
        if explicit is not True: raise ValueError('explicit camera power consent required')
        self.ttl_s = _integer(ttl_s, 1, 3600, 'ttl_s')
        self.startup_s = _seconds(startup_s, 'startup_s')
        self.gain = gain
        self._runner, self._clock, self._sleep, self._killpg = runner, clock, sleep, killpg
        self._robot = str(Path(__file__).resolve().parents[2] / 'bin' / 'robot')
        self._env = dict(os.environ, ROBOT_SSH_TIMEOUT_S=str(ttl_s + 120))
        self._signals = _Signals()
        self._owned = self._used = self._gain_entered = False
        self._hold = self._log = None
        self._lock_path = Path(lock_path).expanduser() if lock_path is not None else Path.home()/'.cache/robot/audio/session.lock'
        self._lock_fd = None
        self._deadline = None

    @property
    def remaining_s(self):
        return 0 if self._deadline is None else max(0, self._deadline - self._clock())

    def _run(self, *command, timeout=15):
        try:
            result = self._runner.run([self._robot, *command], env=self._env,
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      text=True, timeout=timeout, check=True)
            if result.returncode: raise CameraSessionError('robot_command_failed')
            return result.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            raise CameraSessionError('robot_command_failed') from None

    def _acquire_lock(self):
        fd = None
        try:
            parent = self._lock_path.parent
            parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            info = parent.stat()
            if parent.is_symlink() or info.st_uid != os.getuid() or info.st_mode & 0o022:
                raise OSError('unsafe lock directory')
            fd = os.open(self._lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o600:
                raise OSError('unsafe session lock')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_fd = fd
        except OSError:
            if fd is not None: os.close(fd)
            raise CameraSessionError('session_lock_unavailable') from None

    def __enter__(self):
        if self._used: raise CameraSessionError('session_single_use')
        self._used = True
        self._deadline = self._clock() + self.ttl_s
        startup = min(self._deadline, self._clock() + self.startup_s)
        self._signals.install()
        try:
            self._acquire_lock()
            if self._run('status', timeout=self._budget(startup)) != 'enabled':
                raise CameraSessionError('stock_controller_not_enabled')
            # A failed disable may have changed the robot before the connection
            # failed. Ownership begins before sending it, so that case rolls back.
            self._owned = True
            self._run('disable', timeout=self._budget(startup))
            self._log = tempfile.TemporaryFile()
            self._hold = self._runner.Popen(
                [self._robot, 'mask', 'hold', '--mask', '10f8ff'],
                env=self._env, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=self._log, start_new_session=True)
            while True:
                self._budget(startup)
                if self._hold.poll() is not None:
                    raise CameraSessionError('mask_hold_exited')
                log = os.pread(self._log.fileno(), 65537, 0)
                if len(log) > 65536: raise CameraSessionError('mask_hold_output_limit')
                if b'mask-owner: holding;' in log: break
                self._sleep(min(.1, self._budget(startup)))
            if self.gain is not None:
                if isinstance(self.gain, CameraGainLease):
                    self.gain._outer_deadline = startup
                    self.gain._outer_overall_deadline = self._deadline
                    self.gain._power_healthy = lambda: self._hold.poll() is None
                self.gain.__enter__()
                self._gain_entered = True
            self._budget(startup)
            if self._hold.poll() is not None: raise CameraSessionError('mask_hold_exited')
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def _budget(self, deadline):
        remaining = deadline - self._clock()
        if remaining <= 0: raise CameraSessionError('session_deadline')
        # bin/robot includes legitimate 30s controller waits. Unlike individual
        # camera SSH requests, robot startup owns the whole remaining allowance.
        return remaining

    def check(self):
        if not self._owned or self._hold is None:
            raise CameraSessionError('session_not_active')
        self._budget(self._deadline)
        if self._hold.poll() is not None: raise CameraSessionError('mask_hold_exited')
        if self.gain is not None: self.gain.check()
        self._budget(self._deadline)
        if self._hold.poll() is not None: raise CameraSessionError('mask_hold_exited')

    def __exit__(self, exc_type, exc, tb):
        errors = []
        self._signals.block()
        try:
            if self._gain_entered:
                try: self.gain.__exit__(exc_type, exc, tb)
                except BaseException: errors.append('gain')
                self._gain_entered = False
                # Nested context restores its enclosing signal handlers.
                self._signals.block()
            if self._hold is not None:
                try:
                    # Always signal our own group, including children remaining
                    # after an early shell exit. Never signal a discovered PID.
                    try: self._killpg(self._hold.pid, signal.SIGTERM)
                    except ProcessLookupError: pass
                    try: self._hold.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        try: self._killpg(self._hold.pid, signal.SIGKILL)
                        except ProcessLookupError: pass
                        self._hold.wait(timeout=2)
                except (OSError, subprocess.SubprocessError): errors.append('hold')
                self._hold = None
            if self._owned:
                for command in (('mask', 'restore'), ('enable',)):
                    try: self._run(*command)
                    except CameraSessionError: errors.append('robot')
                try:
                    if self._run('status') != 'enabled': errors.append('status')
                except CameraSessionError: errors.append('status')
                self._owned = False
        finally:
            if self._log is not None:
                self._log.close(); self._log = None
            if self._lock_fd is not None:
                os.close(self._lock_fd); self._lock_fd = None
            self._signals.restore()
        if errors: raise CameraSessionError('session_restore_unverified') from None
        return False


def _lease_dir(lease_id):
    if not isinstance(lease_id, str) or not re.fullmatch(r'[0-9a-f]{32}', lease_id):
        raise ValueError('lease id must be UUID hex')
    return '/tmp/robot-audio-camera-' + lease_id


# No shell supplied by callers is accepted. Only these scripts, validated UUIDs,
# integer volumes and TTLs cross SSH. Never copy/restore the full Majestic YAML.
_SHELL_VOLUME = '''valid_volume() {
    case "$1" in ''|*[!0-9]*) return 1;; esac
    [ "${#1}" -le 3 ] && [ "$1" -le 100 ]
}
read_volume() { yaml-cli -g .audio.volume; }
reload_audio() {
    pids=$(pidof majestic) || return 1
    [ -n "$pids" ] || return 1
    for pid in $pids; do
        case "$pid" in ''|*[!0-9]*) return 1;; esac
        [ "$pid" -gt 1 ] || return 1
    done
    kill -HUP $pids
}
'''


class CameraSSH:
    """Fixed camera operations using root + the explicit admin netrc password.

    SSH requires an already trusted host key (StrictHostKeyChecking=yes), ignores
    SSH config overrides, and uses normal known_hosts files. No accept-new or
    host-key bypass. HTTP Basic is suitable only on a trusted private network.
    Runner is subprocess-compatible; tests inject one without contacting hosts.
    """
    def __init__(self, *, host, netrc_path, runner=subprocess):
        if not isinstance(host, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', host):
            raise ValueError('explicit safe camera hostname required')
        self.host, self.netrc_path, self._runner = host, Path(netrc_path).expanduser(), runner
        fd = None
        try:
            fd = os.open(self.netrc_path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            info = os.fstat(fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
                    stat.S_IMODE(info.st_mode) != 0o600 or info.st_size > 65536):
                raise ValueError
            entry = netrc.netrc(f'/proc/self/fd/{fd}').hosts.get(host)
            if not entry or entry[0] != 'admin' or not entry[2] or any(c in entry[2] for c in '\r\n\x00'):
                raise ValueError
            self._password = entry[2].encode()
            if len(self._password) > 1024: raise ValueError
        except (OSError, ValueError, netrc.NetrcParseError):
            raise CameraSessionError('camera_credentials') from None
        finally:
            if fd is not None: os.close(fd)

    def http_ready(self, timeout):
        try:
            response = open_audio_stream(f'http://{self.host}/', self.netrc_path, timeout=timeout)
            response.close()  # No images, configuration or secret response body.
            return True
        except CaptureError: return False

    def _execute(self, script, timeout):
        read_fd, write_fd = os.pipe()
        try:
            os.write(write_fd, self._password + b'\n')
            os.close(write_fd); write_fd = None
            args = ['sshpass', '-d', str(read_fd), 'ssh', '-F', '/dev/null',
                    '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=3',
                    '-o', 'ConnectionAttempts=1', '-o', 'ServerAliveInterval=2',
                    '-o', 'ServerAliveCountMax=2', '-o', 'PreferredAuthentications=password',
                    '-o', 'PubkeyAuthentication=no', '-o', 'NumberOfPasswordPrompts=1',
                    '-o', 'LogLevel=ERROR', f'root@{self.host}', 'sh', '-s']
            result = self._runner.run(args, pass_fds=(read_fd,), input=script.encode(),
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      timeout=timeout, check=True)
            if result.returncode: raise CameraSessionError('camera_ssh_failed')
            output = result.stdout.decode('ascii') if isinstance(result.stdout, bytes) else result.stdout
            if len(output) > 128: raise CameraSessionError('camera_readback_invalid')
            return output
        except (OSError, UnicodeError, subprocess.SubprocessError):
            raise CameraSessionError('camera_ssh_failed') from None
        finally:
            os.close(read_fd)
            if write_fd is not None: os.close(write_fd)

    def start(self, lease_id, volume, ttl_s, *, timeout=10):
        directory = _lease_dir(lease_id)
        _integer(volume, 0, 100, 'volume'); _integer(ttl_s, 1, 3600, 'ttl_s')
        # One camera-local lock across all UUID leases. A stale/failed guard is
        # deliberately not stolen; recovery must be explicit, never hijacked.
        script = _SHELL_VOLUME + f'''umask 077
mkdir /tmp/robot-audio-camera.lock 2>/dev/null || exit 1
# Until launch, this shell owns the lock and only a successfully created UUID
# directory. Once launch is possible, only the guard may release that lock.
d={directory}
created=0
rollback() {{
    if [ "$created" = 1 ]; then rm -rf "$d"; fi
    rmdir /tmp/robot-audio-camera.lock
}}
trap rollback EXIT
trap 'exit 1' HUP INT TERM
mkdir "$d" || exit 1
created=1
original=$(read_volume) || exit 1
valid_volume "$original" || exit 1
printf '%s\\n' "$original" > "$d/original" || exit 1
cat > "$d/guard.sh" <<'ROBOT_AUDIO_GUARD' || exit 1
''' + _SHELL_VOLUME + f'''d={directory}
trap '' HUP
original=$(cat "$d/original") || exit 1
valid_volume "$original" || exit 1
# This guard is the sole writer, serializing apply and restore at TTL/Done.
started=$(date +%s) || exit 1
deadline=$((started + {ttl_s}))
if yaml-cli -s .audio.volume {volume} && reload_audio; then
    actual=$(read_volume)
    if [ "$actual" = {volume} ]; then
        printf '%s %s\\n' "$original" "$actual" > "$d/ready"
        n=0
        while [ ! -e "$d/done" ] && [ "$n" -lt {ttl_s} ]; do
            now=$(date +%s) || break
            [ "$now" -lt "$deadline" ] || break
            sleep 1
            n=$((n + 1))
        done
    fi
fi
if yaml-cli -s .audio.volume "$original" && reload_audio; then
    actual=$(read_volume)
    if [ "$actual" = "$original" ]; then
        printf '%s\\n' "$original" > "$d/restored"
        rmdir /tmp/robot-audio-camera.lock
        exit 0
    fi
fi
# No success marker on failed restore. Keep lock and data for diagnosis.
exit 1
ROBOT_AUDIO_GUARD
# Transfer conservatively BEFORE launch: an interrupted/unknown launch must not
# delete a possibly running guard's data or release its global lock.
trap - EXIT HUP INT TERM
nohup sh "$d/guard.sh" </dev/null >/dev/null 2>&1 &
printf '%s\\n' "$original"
'''
        return _volume(self._execute(script, timeout))

    def ready(self, lease_id, *, timeout=10):
        directory = _lease_dir(lease_id)
        return self._execute(_SHELL_VOLUME + f'''d={directory}
[ ! -e "$d/restored" ] || exit 1
[ -f "$d/ready" ] || exit 0
original=$(cat "$d/original") || exit 1
valid_volume "$original" || exit 1
actual=$(read_volume) || exit 1
valid_volume "$actual" || exit 1
printf '%s %s\\n' "$original" "$actual"
''', timeout).strip()

    def restore(self, lease_id, *, timeout=10):
        directory = _lease_dir(lease_id)
        return self._execute(_SHELL_VOLUME + f'''d={directory}
[ -d "$d" ] && [ ! -L "$d" ] || exit 1
: > "$d/done" || exit 1
n=0
while [ ! -f "$d/restored" ] && [ "$n" -lt 5 ]; do sleep 1; n=$((n + 1)); done
original=$(cat "$d/original") || exit 1
valid_volume "$original" || exit 1
marker=$(cat "$d/restored") || exit 1
[ "$marker" = "$original" ] || exit 1
actual=$(read_volume) || exit 1
valid_volume "$actual" || exit 1
printf '%s %s\\n' "$original" "$actual"
''', timeout).strip()


class CameraGainLease:
    """Temporary YAML gain with independent camera-local rollback guard.

    Use inside CameraSession to power/wait for camera. Can also be used alone
    when power is already explicitly managed. Default desired volume 90 is the
    measured +28 dB firmware setting, not an HTTP API configuration illusion.
    Readback proves YAML + successful HUP, not calibrated physical ACODEC gain.
    """
    def __init__(self, ssh, *, volume=90, ttl_s=600, startup_s=60,
                 readiness=None, clock=time.monotonic, sleep=time.sleep):
        self.volume = _integer(volume, 0, 100, 'volume')
        self.ttl_s = _integer(ttl_s, 1, 3600, 'ttl_s')
        self.startup_s = _seconds(startup_s, 'startup_s')
        self.ssh, self._ready = ssh, readiness or ssh.http_ready
        self._clock, self._sleep = clock, sleep
        self.lease_id = uuid.uuid4().hex
        self.original_volume = None
        self._attempted = self._used = self._active = False
        self._outer_deadline = self._outer_overall_deadline = self._deadline = None
        self._signals = _Signals()
        self._power_healthy = None
        self._guard_deadline = None

    def _budget(self, deadline):
        remaining = deadline - self._clock()
        if remaining <= 0: raise CameraSessionError('camera_deadline')
        return min(10, remaining)

    def __enter__(self):
        if self._used: raise CameraSessionError('gain_lease_single_use')
        self._used = True
        self._deadline = self._clock() + self.ttl_s
        if self._outer_overall_deadline is not None:
            self._deadline = min(self._deadline, self._outer_overall_deadline)
        startup = min(self._deadline, self._clock() + self.startup_s)
        if self._outer_deadline is not None: startup = min(startup, self._outer_deadline)
        self._signals.install()
        try:
            while not self._ready(min(2, self._budget(startup))):
                self._sleep(min(.2, self._budget(startup)))
            self._attempted = True  # Even an unknown SSH outcome must roll back.
            try:
                self.original_volume = self.ssh.start(self.lease_id, self.volume, self.ttl_s,
                                                      timeout=self._budget(startup))
            finally:
                # Include an unknown start outcome. Start's transport has now
                # returned; allow a full remote TTL plus a bounded restore grace.
                self._guard_deadline = self._clock() + self.ttl_s + 10
            while True:
                pair = self.ssh.ready(self.lease_id, timeout=self._budget(startup))
                if pair:
                    if pair != f'{self.original_volume} {self.volume}':
                        raise CameraSessionError('camera_gain_readback')
                    break
                self._sleep(min(.2, self._budget(startup)))
            self._budget(startup)
            self._active = True
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def check(self):
        if not self._active: raise CameraSessionError('gain_lease_not_active')
        pair = self.ssh.ready(self.lease_id, timeout=self._budget(self._deadline))
        if pair != f'{self.original_volume} {self.volume}':
            raise CameraSessionError('camera_gain_readback')
        self._budget(self._deadline)

    def __exit__(self, exc_type, exc, tb):
        self._signals.block()
        try:
            self._active = False
            if self._attempted:
                self._attempted = False
                while True:
                    try:
                        timeout = (self._budget(self._guard_deadline)
                                   if self._power_healthy is not None else 10)
                        pair = self.ssh.restore(self.lease_id, timeout=timeout)
                        parts = pair.split()
                        if len(parts) != 2: raise CameraSessionError('gain_restore_unverified')
                        original, actual = map(_volume, parts)
                        if original != actual or (self.original_volume is not None and original != self.original_volume):
                            raise CameraSessionError('gain_restore_unverified')
                        break
                    except (CameraSessionError, OSError, subprocess.SubprocessError):
                        remaining = self._guard_deadline - self._clock()
                        if (self._power_healthy is None or not self._power_healthy()
                                or remaining <= 0):
                            raise CameraSessionError('gain_restore_unverified') from None
                        self._sleep(min(1, remaining))
        except (CameraSessionError, OSError, subprocess.SubprocessError):
            raise CameraSessionError('gain_restore_unverified') from None
        finally:
            self._signals.restore()
        return False
