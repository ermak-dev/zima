"""A bounded read-only voice demonstration, not a general command executor."""
from collections import deque
from pathlib import Path
import queue
import re
import subprocess
import threading
import time

from .commands import BatterySnapshot, handle_command, is_battery_command, snapshot_from_json


class RobotBatteryReader:
    def __init__(self, *, runner=subprocess.run, timeout_s=4.5):
        self.runner, self.timeout_s = runner, timeout_s
        self.command = [str(Path(__file__).resolve().parents[2] / 'bin' / 'robot'), 'battery']

    def __call__(self):
        try:
            result = self.runner(self.command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, text=True, timeout=self.timeout_s)
            if result.returncode:
                return BatterySnapshot()
            return snapshot_from_json(result.stdout)
        except (OSError, subprocess.SubprocessError, TypeError, ValueError):
            return BatterySnapshot()


class VoiceController:
    def __init__(self, reader, speaker, finish, *, allow_reported=False, max_queue_s=5,
                 clock=time.monotonic):
        self.reader, self.speaker, self.finish = reader, speaker, finish
        self.allow_reported, self.max_queue_s, self.clock = allow_reported is True, max_queue_s, clock
        self._queue = queue.Queue(maxsize=1)
        self._seen = deque(maxlen=256)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._active_cancel = None
        self._thread = None
        self.last_error = ''

    def start(self):
        if self._thread is not None or self._stop.is_set():
            return
        self._thread = threading.Thread(target=self._run, name='readonly-voice-controller', daemon=True)
        self._thread.start()

    def submit(self, event):
        if (not isinstance(event, dict) or event.get('is_final', True) is not True or
                event.get('status') != 'final' or event.get('error')):
            return False
        session_id, text = event.get('session_id'), event.get('text')
        if (not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,128}', session_id) or
                not isinstance(text, str) or len(text) > 8192):
            return False
        acknowledge = False
        with self._lock:
            if self._stop.is_set() or session_id in self._seen:
                return False
            self._seen.append(session_id)
            if not is_battery_command(text):
                self.last_error = 'unsupported_intent'
                acknowledge = True
            else:
                try:
                    self._queue.put_nowait((session_id, text, self.clock()))
                    return True
                except queue.Full:
                    self.last_error = 'controller_busy'
                    acknowledge = True
        if acknowledge:
            self._finish(session_id)
        return False

    def _finish(self, session_id):
        try:
            self.finish(session_id)
        except Exception:
            self.last_error = 'finish_failed'

    def _run(self):
        while not self._stop.is_set():
            try:
                session_id, text, created = self._queue.get(timeout=.05)
            except queue.Empty:
                continue
            cancel = threading.Event()
            with self._lock:
                self._active_cancel = cancel
            try:
                if self._stop.is_set() or self.clock() - created > self.max_queue_s:
                    self.last_error = 'command_expired'
                    continue
                reply = handle_command(text, self.reader, allow_reported=self.allow_reported)
                if not self._stop.is_set():
                    self.speaker(reply, session_id, cancel)
            except Exception:
                self.last_error = 'reply_failed'
            finally:
                with self._lock:
                    self._active_cancel = None
                self._finish(session_id)

    def close(self):
        self._stop.set()
        with self._lock:
            if self._active_cancel is not None:
                self._active_cancel.set()
        while True:
            try:
                session_id, _, _ = self._queue.get_nowait()
            except queue.Empty:
                break
            self._finish(session_id)
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=6)
            if self._thread.is_alive():
                raise RuntimeError('voice controller shutdown not confirmed')
