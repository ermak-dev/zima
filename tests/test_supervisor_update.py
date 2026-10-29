#!/usr/bin/env python3
# FEATURE: automatic supervisor updates before controller handoff.
# SPEC: running-ELF identity, authenticated idle shutdown, staged hashes and safe legacy migration.
# IMPLEMENTATION: compare /proc ELF, stage by digest, quiesce or reserve legacy slots, then promote.
# RESULT: IMPLEMENTED — local listener migration, native restart and refusal paths pass.
# STATUS: DONE
import hashlib
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

# This local fixture is executed by a private copy of Python's ELF so /proc/pid/exe
# has the same path/identity semantics as the deployed supervisor, not a script interpreter.
SERVER = r'''
import os, socket, sys, threading
port, mode, state = int(sys.argv[1]), sys.argv[2], sys.argv[3]
listener = socket.socket()
listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listener.bind(('127.0.0.1', port))
listener.listen(64)
slots = threading.BoundedSemaphore(32 if mode != 'unknown-capacity' else 64)
count = 0
lock = threading.Lock()
def serve(conn):
    global count
    try:
        conn.settimeout(10)
        file = conn.makefile('rb')
        if file.readline() != b'AUTH update-test-secret\n':
            conn.sendall(b'UNAUTHORIZED\n')
            return
        conn.sendall(b'AUTHENTICATED\n')
        request = file.readline()
        if request == b'PING\n':
            conn.sendall(('PONG ' + open(state).read().strip() + '\n').encode())
        elif request == b'SHUTDOWN\n' and mode == 'modern':
            with lock:
                busy = count != 1 or open(state).read().strip() != 'IDLE'
            if busy:
                conn.sendall(b'BUSY\n')
            else:
                conn.sendall(b'STOPPING\n')
                os._exit(0)
        else:
            conn.sendall(b'REFUSED\n')
    except (OSError, ValueError):
        pass
    finally:
        conn.close()
        with lock:
            count -= 1
        slots.release()
print('READY', flush=True)
while True:
    conn, _ = listener.accept()
    if not slots.acquire(blocking=False):
        conn.close()
        continue
    with lock:
        count += 1
    threading.Thread(target=serve, args=(conn,), daemon=True).start()
'''


class SupervisorUpdateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.remote = self.directory / 'robot-supervisor'
        shutil.copy2(Path(sys.executable).resolve(), self.remote)
        self.original = self.remote.read_bytes()
        self.new = b'new supervisor fixture\n'
        self.sha = hashlib.sha256(self.new).hexdigest()
        self.stage = self.directory / f'robot-supervisor.{self.sha}.next'
        self.stage.write_bytes(self.new)
        self.stage.chmod(0o700)
        self.state = self.directory / 'state'
        self.state.write_text('IDLE\n')
        self.script = self.directory / 'listener.py'
        self.script.write_text(SERVER)
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('ROBOT_')}
        self.env['ROBOT_SUPERVISOR_TOKEN'] = 'update-test-secret'
        text = (ROOT / 'bin/robot').read_text()
        function = text[text.index('supervisor_request() {'):text.index('\nsupervisor_start() {')]
        self.harness = '''set -euo pipefail
ssh_host=fake
ssh_wrap() { shift; "$@"; }
''' + function.replace('/tmp/mask-owner', str(self.directory)).replace(
            '/tmp/robot-supervisor.log', str(self.directory / 'supervisor.log')) + '\nsupervisor_request "$@"\n'

    def start(self, mode='modern', foreign=False):
        with socket.socket() as reserve:
            reserve.bind(('127.0.0.1', 0))
            self.port = reserve.getsockname()[1]
        binary = self.remote
        if foreign:
            binary = self.directory / 'foreign-supervisor'
            shutil.copy2(self.remote, binary)
        self.process = subprocess.Popen(
            [str(binary), '-I', str(self.script), str(self.port), mode, str(self.state)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(self.stop)
        self.assertEqual(self.process.stdout.readline(), 'READY\n')

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=3)
        self.process.stdout.close()
        self.process.stderr.close()

    def request(self, mode, sha=None):
        return subprocess.run(['bash', '-c', self.harness, 'test', str(self.port), '5000',
                               mode, sha or self.sha], env=self.env,
                              capture_output=True, text=True, timeout=15)

    def assert_preserved(self, result):
        self.assertNotEqual(result.returncode, 0, result.stderr)
        self.assertIsNone(self.process.poll())
        self.assertEqual(hashlib.sha256(self.remote.read_bytes()).digest(),
                         hashlib.sha256(self.original).digest())

    def test_probe_compares_running_elf_and_reuses_identical_build(self):
        self.start()
        same = hashlib.sha256(self.original).hexdigest()
        self.assertEqual(self.request('check', same).returncode, 0)
        self.assertEqual(self.request('check').returncode, 5)
        self.assertIsNone(self.process.poll())

    def test_new_file_on_disk_does_not_disguise_old_running_elf(self):
        self.start()
        replacement = self.directory / 'replacement'
        replacement.write_bytes(self.new)
        replacement.chmod(0o700)
        replacement.replace(self.remote)
        self.assertEqual(self.request('check').returncode, 5)
        self.assertIsNone(self.process.poll())

    def test_modern_idle_update_stops_only_listener_and_promotes_verified_stage(self):
        self.start()
        result = self.request('update')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.remote.read_bytes(), self.new)
        self.process.wait(timeout=3)
        self.assertFalse(self.stage.exists())

    def test_legacy_idle_daemon_is_updated_without_shutdown_command(self):
        self.start('legacy')
        result = self.request('update')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.remote.read_bytes(), self.new)
        self.process.wait(timeout=3)

    @unittest.skipUnless((ROOT / 'crates/target/debug/robot-supervisor').is_file(),
                         'requires the local native supervisor build')
    def test_updated_real_supervisor_starts_and_passes_running_hash_probe(self):
        self.new = (ROOT / 'crates/target/debug/robot-supervisor').read_bytes()
        self.sha = hashlib.sha256(self.new).hexdigest()
        self.stage = self.directory / f'robot-supervisor.{self.sha}.next'
        self.stage.write_bytes(self.new)
        self.stage.chmod(0o700)
        self.start('legacy')
        result = self.request('update')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.process.wait(timeout=3)

        def cleanup_native():
            for exe in Path('/proc').glob('[0-9]*/exe'):
                try:
                    if os.readlink(exe) == str(self.remote):
                        os.kill(int(exe.parent.name), signal.SIGTERM)
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    pass
        self.addCleanup(cleanup_native)
        result = self.request('auto')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.request('check').returncode, 0)
        self.assertIn('listening on', (self.directory / 'supervisor.log').read_text())

    def test_active_owner_is_preserved(self):
        self.start()
        self.state.write_text('ACTIVE\n')
        self.assert_preserved(self.request('update'))

    def test_other_clients_prevent_both_modern_and_legacy_update(self):
        for mode in ('modern', 'legacy'):
            with self.subTest(mode=mode):
                self.start(mode)
                observer = socket.create_connection(('127.0.0.1', self.port), timeout=2)
                observer.sendall(b'AUTH update-test-secret\n')
                self.assertEqual(observer.recv(256), b'AUTHENTICATED\n')
                try:
                    self.assert_preserved(self.request('update'))
                finally:
                    observer.close()
                    self.stop()

    def test_foreign_listener_and_unknown_legacy_capacity_are_preserved(self):
        self.start(foreign=True)
        self.assert_preserved(self.request('update'))
        self.stop()
        self.start('unknown-capacity')
        self.assert_preserved(self.request('update'))

    def test_legacy_reservation_deadline_is_checked_after_identity_revalidation(self):
        self.start('legacy')
        self.harness = self.harness.replace(
            '    [[ $(listener_pid) == "$daemon_pid" ]]',
            '    sleep 4\n    [[ $(listener_pid) == "$daemon_pid" ]]')
        self.assert_preserved(self.request('update'))

    def test_corrupt_stage_is_rejected_before_stopping_daemon(self):
        self.start()
        self.stage.write_bytes(b'corrupt upload')
        self.assert_preserved(self.request('update'))

    def test_authentication_failure_never_stops_or_replaces_daemon(self):
        self.start()
        self.env['ROBOT_SUPERVISOR_TOKEN'] = 'wrong'
        self.assert_preserved(self.request('update'))


if __name__ == '__main__':
    unittest.main()
