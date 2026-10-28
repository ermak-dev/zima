#!/usr/bin/env python3
# FEATURE: One supervised native server replaces the ROS launch stack.
# SPEC: No ROS sourcing; preflight before handoff; restore on exit, failure and signals.
# IMPLEMENTATION: Direct server dispatch under one session lock and controller recovery trap.
# RESULT: IMPLEMENTED — fakes verify normal exit, failures, refusal and SIGTERM recovery.
# STATUS: DONE
import fcntl
import os
import shutil
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
        config = self.directory / 'robot.toml'
        config.write_text('')
        config.chmod(0o600)
        self.env.update(ROBOT_CONFIG=str(config),
                        ROBOT_TEST_LOG=str(self.log), ROBOT_TEST_STATE=str(self.state),
                        ROBOT_SESSION_LOCK=str(self.directory / 'session.lock'),
                        ROBOT_ROS_SETUP='/nonexistent/ros-setup',
                        ROBOT_WS_SETUP='/nonexistent/workspace')
        self.executable('supervisor', '#!/bin/sh\nexit 0\n')
        self.env['ROBOT_SLAM_ADAPTER'] = str(self.executable('default-adapter', '#!/bin/sh\nexit 0\n'))
        self.env['ROBOT_YOLO_ADAPTER'] = str(self.executable('default-yolo-adapter', '#!/bin/sh\nexit 0\n'))
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
        *ROBOT_SUPERVISOR_START*)
            echo supervisor >> "$ROBOT_TEST_LOG"
            [[ ${SUPERVISOR_ACTIVE:-0} == 0 && ${SUPERVISOR_BUSY:-0} == 0 ]] || { echo 'refusing to use busy or unavailable supervisor' >&2; exit 2; }
            [[ ${SUPERVISOR_FOREIGN:-0} == 0 ]] || exit 4
            if [[ $script == *mode=check* && ${SUPERVISOR_MISSING:-0} == 1 ]]; then exit 3; fi
            if [[ $script == *mode=auto* || $script == *mode=start* ]]; then
                echo supervisor-start >> "$ROBOT_TEST_LOG"
                [[ ${FAIL_SUPERVISOR_START:-0} == 0 ]] || exit 1
            elif [[ ${SUPERVISOR_MISSING:-0} == 1 ]]; then
                exit 3
            fi ;;
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
    echo upload >> "$ROBOT_TEST_LOG"
    [[ ${FAIL_UPLOAD:-0} == 0 ]] || exit 1
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

    # FEATURE: normal sessions start panel MRPT SLAM by default.
    # SPEC: missing adapters fail before handoff; --no-slam explicitly opts out.
    # IMPLEMENTATION: repository-relative default with environment and CLI overrides.
    # RESULT: IMPLEMENTED — defaults, overrides and preflight verified with fakes.
    # STATUS: DONE
    def test_slam_enabled_by_default(self):
        result = self.run_cli('run')
        self.assertEqual(result.returncode, 0, result.stderr)
        server = next(line for line in self.log.read_text().splitlines()
                      if line.startswith('server:'))
        self.assertIn(f'--slam-adapter {self.env["ROBOT_SLAM_ADAPTER"]}', server)

    def test_default_adapter_path_is_relative_to_repository(self):
        repo = self.directory / 'repo'
        (repo / 'bin').mkdir(parents=True)
        cli = repo / 'bin/robot'
        cli.write_text((ROOT / 'bin/robot').read_text())
        cli.chmod(0o700)
        shutil.copy2(ROOT / 'bin/config.py', repo / 'bin/config.py')
        (repo / 'python').mkdir()
        shutil.copy2(ROOT / 'python/robot_config.py', repo / 'python/robot_config.py')
        env = self.env.copy()
        env.pop('ROBOT_SLAM_ADAPTER')
        result = subprocess.run([str(cli), 'run'], env=env,
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(str(repo / 'tools/mrpt-adapter/adapter'), result.stderr)
        self.assertEqual(self.log.read_text(), '')

    def test_missing_default_adapter_fails_before_ssh(self):
        result = self.run_cli('run', ROBOT_SLAM_ADAPTER=str(self.directory / 'missing'))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('adapter', result.stderr.lower())
        self.assertEqual(self.log.read_text(), '')

    def test_no_slam_explicitly_disables_default_and_skips_adapter_check(self):
        result = self.run_cli('run', '--no-slam',
                              ROBOT_SLAM_ADAPTER=str(self.directory / 'missing'))
        self.assertEqual(result.returncode, 0, result.stderr)
        server = next(line for line in self.log.read_text().splitlines()
                      if line.startswith('server:'))
        self.assertNotIn('--slam-adapter', server)

    # FEATURE: normal sessions enable MRPT and YOLO with explicit opt-outs.
    # SPEC: repository-relative defaults, environment/CLI overrides and executable
    # preflight before SSH; --no-yolo disables only object detection.
    # IMPLEMENTATION: repository defaults with environment/CLI overrides and opt-outs.
    # RESULT: IMPLEMENTED — fake sessions verify defaults, refusal and independent disabling.
    # STATUS: DONE
    def test_yolo_adapter_flag_reaches_server(self):
        adapter = self.executable('yolo-adapter', '#!/bin/sh\nexit 0\n')
        result = self.run_cli('run', '--yolo-adapter', str(adapter))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f'--yolo-adapter {adapter}', self.log.read_text())
        self.assertIn('--slam-adapter', self.log.read_text())

    def test_yolo_adapter_missing_or_not_executable_fails_before_ssh(self):
        not_executable = self.directory / 'not-executable'
        not_executable.write_text('not an executable')
        for path in (self.directory / 'missing-yolo', not_executable):
            with self.subTest(path=path):
                self.log.write_text('')
                result = self.run_cli('run', '--yolo-adapter', str(path))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('YOLO', result.stderr)
                self.assertEqual(self.log.read_text(), '')
        result = self.run_cli('run', '--yolo-adapter')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.log.read_text(), '')

    def test_yolo_enabled_by_default(self):
        result = self.run_cli('run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f'--yolo-adapter {self.env["ROBOT_YOLO_ADAPTER"]}', self.log.read_text())
        self.assertIn(f'--slam-adapter {self.env["ROBOT_SLAM_ADAPTER"]}', self.log.read_text())

    def test_yolo_default_path_is_relative_to_repository(self):
        repo = self.directory / 'repo'
        (repo / 'bin').mkdir(parents=True)
        cli = repo / 'bin/robot'
        cli.write_text((ROOT / 'bin/robot').read_text())
        cli.chmod(0o700)
        shutil.copy2(ROOT / 'bin/config.py', repo / 'bin/config.py')
        (repo / 'python').mkdir()
        shutil.copy2(ROOT / 'python/robot_config.py', repo / 'python/robot_config.py')
        env = self.env.copy()
        env.pop('ROBOT_YOLO_ADAPTER')
        result = subprocess.run([str(cli), 'run'], env=env,
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(str(repo / 'tools/yolo/adapter'), result.stderr)
        self.assertEqual(self.log.read_text(), '')

    def test_yolo_default_preflight_rejects_missing_files_and_directories(self):
        for path in (self.directory / 'missing-yolo', self.directory):
            with self.subTest(path=path):
                self.log.write_text('')
                result = self.run_cli('run', ROBOT_YOLO_ADAPTER=str(path))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('YOLO', result.stderr)
                self.assertEqual(self.log.read_text(), '')

    def test_no_yolo_skips_preflight_and_keeps_slam(self):
        result = self.run_cli('run', '--no-yolo',
                              ROBOT_YOLO_ADAPTER=str(self.directory / 'missing-yolo'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('--yolo-adapter', self.log.read_text())
        self.assertIn('--slam-adapter', self.log.read_text())

    def test_both_adapters_can_be_disabled(self):
        result = self.run_cli('run', '--no-slam', '--no-yolo',
                              ROBOT_YOLO_ADAPTER=str(self.directory / 'missing-yolo'),
                              ROBOT_SLAM_ADAPTER=str(self.directory / 'missing-slam'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('--yolo-adapter', self.log.read_text())
        self.assertNotIn('--slam-adapter', self.log.read_text())

    def test_yolo_cli_override_takes_precedence_over_environment(self):
        adapter = self.executable('override-yolo-adapter', '#!/bin/sh\nexit 0\n')
        result = self.run_cli('run', '--yolo-adapter', str(adapter),
                              ROBOT_YOLO_ADAPTER=str(self.directory / 'missing-yolo'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f'--yolo-adapter {adapter}', self.log.read_text())

    def test_slam_adapter_flag_reaches_server(self):
        self.executable('slam-adapter', '#!/bin/sh\nexit 0\n')
        result = self.run_cli('run', '--slam-adapter', str(self.directory / 'slam-adapter'))
        self.assertEqual(result.returncode, 0, result.stderr)
        server = next(line for line in self.log.read_text().splitlines()
                      if line.startswith('server:'))
        self.assertIn('--slam-adapter', server)

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
                self.assertIn(f'--slam-adapter {self.env["ROBOT_SLAM_ADAPTER"]}', server)
                self.assertIn(f'--yolo-adapter {self.env["ROBOT_YOLO_ADAPTER"]}', server)
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

    # BUG: robot run lost its automatic supervisor bootstrap after adding TTS.
    # HYPOTHESIS: run selects probe-only mode and requires a separate daemon command.
    # SOLUTION: probe first; only an absent listener permits automatic deployment/start.
    # RESULT: FIXED — bootstrap precedes handoff; reuse and foreign-session refusal are preserved.
    # STATUS: FIXED
    def test_run_bootstraps_missing_supervisor_before_controller_handoff(self):
        for command in ('run', 'panel', 'bridge'):
            with self.subTest(command=command):
                self.log.write_text('')
                result = self.run_cli(command, SUPERVISOR_MISSING='1')
                self.assertEqual(result.returncode, 0, result.stderr)
                lines = self.log.read_text().splitlines()
                self.assertIn('upload', lines)
                self.assertIn('supervisor-start', lines)
                self.assertLess(lines.index('upload'), lines.index('supervisor-start'))
                self.assertLess(lines.index('supervisor-start'), lines.index('disable'))
                self.assertTrue(any(line.startswith('server:') for line in lines))
                self.assertEqual(self.state.read_text().strip(), 'enabled')

    def test_run_reuses_idle_supervisor_without_upload_or_local_binary(self):
        result = self.run_cli('run', ROBOT_SUPERVISOR_BIN=str(self.directory / 'missing'))
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = self.log.read_text().splitlines()
        self.assertLess(lines.index('supervisor'), lines.index('disable'))
        self.assertNotIn('supervisor-start', lines)
        self.assertNotIn('upload', lines[:lines.index('disable')])

    def test_failed_bootstrap_never_changes_controller(self):
        for override in ({'FAIL_UPLOAD':'1'}, {'FAIL_SUPERVISOR_START':'1'},
                         {'ROBOT_SUPERVISOR_BIN':str(self.directory / 'missing')}):
            with self.subTest(override=override):
                self.log.write_text('')
                result = self.run_cli('run', SUPERVISOR_MISSING='1', **override)
                self.assertNotEqual(result.returncode, 0)
                lines = self.log.read_text().splitlines()
                self.assertNotIn('disable', lines)
                self.assertNotIn('enable', lines)
                self.assertFalse(any(line.startswith('server:') for line in lines))
                self.assertEqual(self.state.read_text().strip(), 'enabled')

    def test_foreign_supervisor_is_not_uploaded_over_or_used_for_handoff(self):
        result = self.run_cli('run', SUPERVISOR_FOREIGN='1')
        self.assertNotEqual(result.returncode, 0)
        lines = self.log.read_text().splitlines()
        self.assertNotIn('upload', lines)
        self.assertNotIn('supervisor-start', lines)
        self.assertNotIn('disable', lines)
        self.assertNotIn('enable', lines)

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

    # FEATURE: Persistent-daemon liveness is not permission to steal a session.
    # SPEC: A pre-existing disabled controller must not be disabled or restored.
    def test_existing_controller_handoff_is_not_disturbed(self):
        self.state.write_text('disabled\n')
        result = self.run_cli('run')
        self.assertNotEqual(result.returncode, 0)
        lines = self.log.read_text().splitlines()
        self.assertNotIn('disable', lines)
        self.assertNotIn('enable', lines)
        self.assertFalse(any(line.startswith('server:') for line in lines))
        self.assertEqual(self.state.read_text(), 'disabled\n')

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
        audio = self.executable('audio', '#!/bin/sh\nprintf "audio:%s\\n" "$*" >> "$ROBOT_TEST_LOG"\n')
        result = self.run_cli('audio', 'status', ROBOT_AUDIO_BIN=str(audio))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('audio:status', self.log.read_text())


if __name__ == '__main__':
    unittest.main()
