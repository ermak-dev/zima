#!/usr/bin/env python3
# FEATURE: One supervised native server replaces the ROS launch stack.
# SPEC: No ROS sourcing; preflight before handoff; restore on exit, failure and signals.
# IMPLEMENTATION: Direct server dispatch under one session lock and controller recovery trap.
# RESULT: IMPLEMENTED — fakes verify normal exit, failures, refusal and SIGTERM recovery.
# STATUS: DONE
import fcntl
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ServerCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.log = self.directory / 'log'
        self.log.touch()
        self.state = self.directory / 'state'
        self.state.write_text('enabled\n')
        self.env = {key: value for key, value in os.environ.items() if not key.startswith('ROBOT_')}
        self.env.update(ROBOT_TEST_LOG=str(self.log), ROBOT_TEST_STATE=str(self.state),
                        ROBOT_SESSION_LOCK=str(self.directory / 'session.lock'),
                        ROBOT_ROS_SETUP='/nonexistent/ros-setup',
                        ROBOT_WS_SETUP='/nonexistent/workspace')
        self.executable('supervisor', '#!/bin/sh\nexit 0\n')
        self.executable('server', '''#!/bin/bash
printf 'server:%s heartbeat=%s\n' "$*" "${ROBOT_SERVER_HEARTBEAT:-}" >> "$ROBOT_TEST_LOG"
trap 'echo server-stopped >> "$ROBOT_TEST_LOG"; exit 0' TERM INT
if [[ ${SERVER_HOLD:-0} == 1 ]]; then while :; do sleep .05; done; fi
exit "${SERVER_EXIT:-0}"
''')
        self.executable('ssh', '''#!/bin/bash
if [[ $1 == -G ]]; then
    echo resolve >> "$ROBOT_TEST_LOG"
    [[ ${FAIL_RESOLVE:-0} == 0 ]] || exit 1
    echo 'hostname 127.0.0.1'; exit 0
fi
if [[ "$*" == 'robot bash -s' ]]; then
    script=$(cat)
    case "$script" in
        *ROBOT_STALE_SUPERVISOR*)
            echo stale-check >> "$ROBOT_TEST_LOG"
            [[ ${SUPERVISOR_ACTIVE:-0} == 0 ]] || { echo 'refusing to start another' >&2; exit 1; } ;;
        *ROBOT_DISABLE_CONTROLLER*)
            echo disable >> "$ROBOT_TEST_LOG"
            echo disabled > "$ROBOT_TEST_STATE"
            [[ ${FAIL_DISABLE:-0} == 0 ]] || exit 1 ;;
        *ROBOT_ENABLE_CONTROLLER*)
            echo enable >> "$ROBOT_TEST_LOG"
            echo enabled > "$ROBOT_TEST_STATE" ;;
        *ROBOT_STATUS_COMMANDS*) cat "$ROBOT_TEST_STATE" ;;
    esac
elif [[ "$*" == *'robot-supervisor listen'* ]]; then
    echo supervisor >> "$ROBOT_TEST_LOG"
    [[ ${SUPERVISOR_BUSY:-0} == 0 ]] || { echo 'Address already in use' >&2; exit 1; }
elif [[ "$*" == *'cat > '* ]]; then
    cat >/dev/null
fi
exit 0
''')
        self.env.update(ROBOT_SERVER_BIN=str(self.directory / 'server'),
                        ROBOT_SUPERVISOR_BIN=str(self.directory / 'supervisor'),
                        ROBOT_SSH=str(self.directory / 'ssh'))

    def executable(self, name, content):
        path = self.directory / name
        path.write_text(content)
        path.chmod(0o700)
        return path

    def run_cli(self, *args, **overrides):
        return subprocess.run([str(ROOT / 'bin/robot'), *args], env=self.env | overrides,
                              capture_output=True, text=True, timeout=10)

    def test_run_panel_and_bridge_use_same_native_server_and_restore(self):
        for command in ('run', 'panel', 'bridge'):
            with self.subTest(command=command):
                self.log.write_text('')
                result = self.run_cli(command, '--port', '8099', '--listen', '127.0.0.1')
                self.assertEqual(result.returncode, 0, result.stderr)
                lines = self.log.read_text().splitlines()
                server = next(line for line in lines if line.startswith('server:'))
                self.assertIn('--player-host 127.0.0.1', server)
                self.assertIn('--port 8099', server)
                self.assertIn('--listen 127.0.0.1', server)
                self.assertIn('heartbeat=127.0.0.1:7665', server)
                self.assertLess(lines.index('supervisor'), lines.index('disable'))
                self.assertLess(lines.index('disable'), lines.index(server))
                self.assertIn('enable', lines)
                self.assertEqual(self.state.read_text().strip(), 'enabled')

    def test_server_failure_preserves_status_and_restores_controller(self):
        result = self.run_cli('run', SERVER_EXIT='7')
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertEqual(self.state.read_text().strip(), 'enabled')
        self.assertIn('enable', self.log.read_text().splitlines())

    def test_removed_modes_and_invalid_arguments_never_contact_robot(self):
        for args in (('run', '--slam'), ('run', '--sim'), ('panel', '--slam'),
                     ('run', '--port'), ('run', '--port', '0'), ('panel', '--port', '65536'),
                     ('run', '--port', 'abc'), ('bridge', '--timeout-ms', 'x'),
                     ('run', '--heartbeat-port', '0'), ('run', '--listen'), ('run', 'extra')):
            with self.subTest(args=args):
                self.log.write_text('')
                result = self.run_cli(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.log.read_text(), '', result.stderr)

    def test_missing_server_fails_before_ssh(self):
        result = self.run_cli('run', ROBOT_SERVER_BIN=str(self.directory / 'missing'))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('server', result.stderr.lower())
        self.assertEqual(self.log.read_text(), '')

    def test_an_active_local_session_is_not_disturbed(self):
        with open(self.env['ROBOT_SESSION_LOCK'], 'w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.run_cli('run')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('already running', result.stderr)
        self.assertEqual(self.log.read_text(), '')

    def test_supervisor_refusal_does_not_disable_or_restore_someone_elses_session(self):
        for flag in ('SUPERVISOR_ACTIVE', 'SUPERVISOR_BUSY'):
            with self.subTest(flag=flag):
                self.log.write_text('')
                result = self.run_cli('run', **{flag: '1'})
                self.assertNotEqual(result.returncode, 0)
                lines = self.log.read_text().splitlines()
                self.assertNotIn('disable', lines)
                self.assertNotIn('enable', lines)
                self.assertFalse(any(line.startswith('server:') for line in lines))

    def test_failed_disable_still_runs_controller_recovery(self):
        result = self.run_cli('run', FAIL_DISABLE='1')
        self.assertNotEqual(result.returncode, 0)
        lines = self.log.read_text().splitlines()
        self.assertIn('disable', lines)
        self.assertIn('enable', lines)
        self.assertFalse(any(line.startswith('server:') for line in lines))

    def test_failed_host_resolution_does_not_change_controller(self):
        result = self.run_cli('run', FAIL_RESOLVE='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('cannot resolve', result.stderr)
        self.assertEqual(self.log.read_text().splitlines(), ['resolve'])

    def test_sigterm_stops_child_and_restores_controller(self):
        process = subprocess.Popen([str(ROOT / 'bin/robot'), 'run'],
                                   env=self.env | {'SERVER_HOLD': '1'},
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(lambda: process.poll() is None and process.kill())
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and 'server:' not in self.log.read_text():
            if process.poll() is not None:
                self.fail('server wrapper exited before starting native server')
            time.sleep(.02)
        self.assertIn('server:', self.log.read_text())
        process.terminate()
        self.assertEqual(process.wait(timeout=5), 143)
        self.assertIn('server-stopped', self.log.read_text())
        self.assertEqual(self.state.read_text().strip(), 'enabled')

    def test_audio_dispatch_does_not_source_ros(self):
        self.executable('python3', '#!/bin/sh\nprintf "python:%s\\n" "$*" >> "$ROBOT_TEST_LOG"\n')
        result = self.run_cli('audio', 'status', PATH=str(self.directory) + ':' + self.env['PATH'])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('bin/audio/cli.py status', self.log.read_text())


if __name__ == '__main__':
    unittest.main()
